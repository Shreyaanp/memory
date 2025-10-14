#include "AntiSpoofing.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

namespace mdai {

// ============================================================================
// Helper Functions
// ============================================================================

void rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b, float& h, float& s, float& v) {
    float rf = r / 255.0f;
    float gf = g / 255.0f;
    float bf = b / 255.0f;
    
    float max_val = std::max({rf, gf, bf});
    float min_val = std::min({rf, gf, bf});
    float delta = max_val - min_val;
    
    v = max_val;
    
    if (delta < 0.00001f) {
        s = 0.0f;
        h = 0.0f;
        return;
    }
    
    s = delta / max_val;
    
    if (max_val == rf) {
        h = 60.0f * fmod(((gf - bf) / delta), 6.0f);
    } else if (max_val == gf) {
        h = 60.0f * ((bf - rf) / delta + 2.0f);
    } else {
        h = 60.0f * ((rf - gf) / delta + 4.0f);
    }
    
    if (h < 0.0f) h += 360.0f;
}

float calculate_depth_smoothness(const FrameBox* frame, int face_x, int face_y, int face_w, int face_h) {
    if (!frame || frame->depth_data.empty()) return 0.0f;
    
    float total_gradient = 0.0f;
    int gradient_count = 0;
    
    for (int y = face_y + 1; y < face_y + face_h - 1 && y < frame->depth_height - 1; y++) {
        for (int x = face_x + 1; x < face_x + face_w - 1 && x < frame->depth_width - 1; x++) {
            int center_idx = y * frame->depth_width + x;
            int right_idx = y * frame->depth_width + (x + 1);
            int down_idx = (y + 1) * frame->depth_width + x;
            
            if (center_idx < frame->depth_data.size() && 
                right_idx < frame->depth_data.size() && 
                down_idx < frame->depth_data.size()) {
                
                float center_depth = frame->depth_data[center_idx] * frame->depth_scale * 1000.0f;
                float right_depth = frame->depth_data[right_idx] * frame->depth_scale * 1000.0f;
                float down_depth = frame->depth_data[down_idx] * frame->depth_scale * 1000.0f;
                
                if (center_depth > 100.0f && center_depth < 5000.0f &&
                    right_depth > 100.0f && right_depth < 5000.0f &&
                    down_depth > 100.0f && down_depth < 5000.0f) {
                    
                    float grad_x = std::abs(right_depth - center_depth);
                    float grad_y = std::abs(down_depth - center_depth);
                    
                    total_gradient += grad_x + grad_y;
                    gradient_count++;
                }
            }
        }
    }
    
    if (gradient_count == 0) return 0.0f;
    
    float avg_gradient = total_gradient / gradient_count;
    
    if (avg_gradient > 5.0f && avg_gradient < 50.0f) {
        return 1.0f;
    } else if (avg_gradient > 2.0f && avg_gradient < 100.0f) {
        return 0.5f;
    } else {
        return 0.0f;
    }
}

// ============================================================================
// QualityGate Implementation
// ============================================================================

QualityGate::QualityGate(const AntiSpoofingConfig& config) : config_(config) {
#ifdef USE_MEDIAPIPE
    // Try MediaPipe GPU-accelerated face detection (if built from source)
    try {
        // MediaPipe face detection graph with GPU support
        std::string graph_config = R"(
            input_stream: "input_video"
            output_stream: "face_detections"
            
            node {
              calculator: "ImageTransformationCalculator"
              input_stream: "IMAGE:input_video"
              output_stream: "IMAGE:transformed_image"
            }
            
            node {
              calculator: "FaceDetectionShortRangeGpu"
              input_stream: "IMAGE:transformed_image"
              output_stream: "DETECTIONS:face_detections"
            }
        )";
        
        mediapipe::CalculatorGraphConfig mp_config;
        if (mediapipe::ParseTextProto(graph_config, &mp_config)) {
            mediapipe_graph_ = std::make_unique<mediapipe::CalculatorGraph>();
            auto status = mediapipe_graph_->Initialize(mp_config);
            if (status.ok()) {
                status = mediapipe_graph_->StartRun({});
                if (status.ok()) {
                    use_mediapipe_ = true;
                    std::cout << "✓ MediaPipe GPU face detection initialized" << std::endl;
                    return;
                }
            }
        }
        std::cerr << "INFO: MediaPipe GPU initialization failed, using OpenCV" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "INFO: MediaPipe error (" << e.what() << "), using OpenCV" << std::endl;
    }
#endif

#ifdef HAVE_OPENCV
    // OpenCV Haar Cascade (primary detection method)
    std::string cascade_path = "haarcascade_frontalface_alt.xml";
    face_cascade_loaded_ = face_cascade_.load(cascade_path);
    
    if (!face_cascade_loaded_) {
        std::cerr << "ERROR: OpenCV face detection failed!" << std::endl;
        std::cerr << "Face detection will be disabled." << std::endl;
    } else {
        std::cout << "✓ OpenCV face detection loaded" << std::endl;
    }
#else
    std::cerr << "ERROR: No face detection available (build with OpenCV or MediaPipe C++)!" << std::endl;
#endif
}

FaceROI QualityGate::detect_face(const FrameBox* frame) {
    FaceROI face;
    face.detected = false;
    
    if (!frame || frame->color_data.empty()) {
        return face;
    }

    // OPTIONAL: Use MediaPipe GPU if available (requires C++ build from source)
#ifdef USE_MEDIAPIPE
    if (use_mediapipe_ && mediapipe_graph_) {
        cv::Mat color_mat = frame->get_color_mat();
        if (!color_mat.empty()) {
            try {
                // Convert to MediaPipe ImageFrame (GPU processing)
                auto input_frame = mediapipe::formats::MatView(&color_mat);
                auto packet = mediapipe::MakePacket<mediapipe::ImageFrame>(
                    mediapipe::ImageFormat::SRGB, 
                    color_mat.cols, 
                    color_mat.rows
                ).At(mediapipe::Timestamp(0));
                
                // Send to GPU pipeline
                auto status = mediapipe_graph_->AddPacketToInputStream("input_video", packet);
                if (status.ok()) {
                    // Get detections from GPU
                    mediapipe::Packet detection_packet;
                    if (mediapipe_graph_->GetOutputStream("face_detections").GetPacket(&detection_packet)) {
                        // Parse MediaPipe face detections and return best face
                        // MediaPipe provides bbox + 6 keypoints (eyes, nose, mouth corners)
                        // This gives us REAL landmarks unlike Haar cascade estimates
                        face.detected = true;
                        // TODO: Parse detection_packet for bbox and landmarks
                        return face;
                    }
                }
            } catch (const std::exception& e) {
                // Fall through to OpenCV backup on error
                static bool warned = false;
                if (!warned) {
                    std::cerr << "MediaPipe detection error: " << e.what() << std::endl;
                    warned = true;
                }
            }
        }
    }
#endif
    
    // PRIMARY: OpenCV Haar Cascade (cross-platform, reliable)
#ifdef HAVE_OPENCV
    if (!face_cascade_loaded_) {
        return face;
    }
    
    cv::Mat color_mat = frame->get_color_mat();
    if (color_mat.empty()) {
        return face;
    }
    
    cv::Mat gray;
    cv::cvtColor(color_mat, gray, cv::COLOR_BGR2GRAY);
    
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    cv::Mat enhanced;
    clahe->apply(gray, enhanced);
    
    std::vector<cv::Rect> faces;
    std::vector<int> num_detections;
    face_cascade_.detectMultiScale(
        enhanced,
        faces,
        num_detections,
        1.1,
        3,
        cv::CASCADE_SCALE_IMAGE,
        cv::Size(30, 30),
        cv::Size(300, 300)
    );
    
    if (faces.empty()) {
        return face;
    }
    
    int best_idx = 0;
    float best_score = 0.0f;
    for (size_t i = 0; i < faces.size(); ++i) {
        float size_score = faces[i].area() / (100.0f * 100.0f);
        float detection_score = num_detections[i] / 10.0f;
        float total_score = (size_score + detection_score) / 2.0f;
        
        if (total_score > best_score) {
            best_score = total_score;
            best_idx = i;
        }
    }
    
    face.detected = true;
    face.bbox = faces[best_idx];
    face.confidence = std::min(1.0f, best_score);
    
    float cx = face.bbox.x + face.bbox.width * 0.5f;
    float eye_y = face.bbox.y + face.bbox.height * 0.35f;
    float nose_y = face.bbox.y + face.bbox.height * 0.6f;
    float mouth_y = face.bbox.y + face.bbox.height * 0.8f;
    float eye_width = face.bbox.width * 0.25f;
    float mouth_width = face.bbox.width * 0.4f;
    
    face.landmarks = {
        cv::Point2f(cx - eye_width, eye_y),
        cv::Point2f(cx + eye_width, eye_y),
        cv::Point2f(cx, nose_y),
        cv::Point2f(cx - mouth_width/2, mouth_y),
        cv::Point2f(cx + mouth_width/2, mouth_y)
    };
    
#endif
    
    return face;
}

bool QualityGate::process_frame(FrameBox* frame) {
    if (!frame) return false;
    
    frame->metadata.quality_gate = FrameBoxMetadata::QualityGateResults{};
    
    FaceROI face = detect_face(frame);
    if (!face.detected && !face_history_.empty()) {
        const FaceROI& last = face_history_.back();
        if (last.detected && last.confidence >= 0.7f) {
            face = last;
        }
    }
    
    face_history_.push_back(face);
    if (face_history_.size() > HISTORY_SIZE) {
        face_history_.pop_front();
    }
    
    frame->metadata.face_detected = face.detected;
    if (face.detected) {
        frame->metadata.face_x = face.bbox.x;
        frame->metadata.face_y = face.bbox.y;
        frame->metadata.face_w = face.bbox.width;
        frame->metadata.face_h = face.bbox.height;
        frame->metadata.face_detection_confidence = face.confidence;
    }

    if (!face.detected) {
        frame->metadata.quality_gate.quality_gate_passed = false;
        frame->metadata.quality_gate.quality_issues = "No face detected";
        return false;
    }
    
    frame->metadata.quality_gate.lighting_score = analyze_lighting_quality(frame, face);
    frame->metadata.quality_gate.motion_score = analyze_motion_stability(frame, face);
    frame->metadata.quality_gate.positioning_score = analyze_face_positioning(frame, face);
    frame->metadata.quality_gate.synchronization_score = analyze_sensor_synchronization(frame);
    frame->metadata.quality_gate.stability_score = analyze_camera_stability(frame);
    
    frame->metadata.quality_gate.lighting_ok = frame->metadata.quality_gate.lighting_score >= config_.min_lighting_score;
    frame->metadata.quality_gate.motion_stable = frame->metadata.quality_gate.motion_score >= config_.min_motion_score;
    frame->metadata.quality_gate.face_positioned = frame->metadata.quality_gate.positioning_score >= config_.min_positioning_score;
    frame->metadata.quality_gate.sensors_synced = frame->metadata.quality_gate.synchronization_score >= config_.min_synchronization_score;
    frame->metadata.quality_gate.camera_stable = frame->metadata.quality_gate.stability_score >= config_.min_stability_score;
    
    frame->metadata.quality_gate.overall_quality_score = 
        (frame->metadata.quality_gate.lighting_score * 0.25f +
         frame->metadata.quality_gate.motion_score * 0.20f +
         frame->metadata.quality_gate.positioning_score * 0.25f +
         frame->metadata.quality_gate.synchronization_score * 0.15f +
         frame->metadata.quality_gate.stability_score * 0.15f);
    
    frame->metadata.quality_gate.quality_gate_passed = 
        frame->metadata.quality_gate.overall_quality_score >= config_.min_overall_quality;
    
    if (!frame->metadata.quality_gate.quality_gate_passed) {
        frame->metadata.quality_gate.quality_issues = generate_quality_issues(frame);
    }
    
    frame->metadata.quality_gate_processed = true;
    return frame->metadata.quality_gate.quality_gate_passed;
}

float QualityGate::analyze_lighting_quality(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected) {
        return 0.0f;
    }
    
#ifdef HAVE_OPENCV
    cv::Mat color_mat = frame->get_color_mat();
    if (color_mat.empty()) {
        return 0.0f;
    }
    
    cv::Rect roi = face.bbox;
    if (roi.x < 0 || roi.y < 0 || 
        roi.x + roi.width > color_mat.cols || 
        roi.y + roi.height > color_mat.rows) {
        return 0.0f;
    }
    
    cv::Mat face_region = color_mat(roi);
    
    cv::Mat gray;
    cv::cvtColor(face_region, gray, cv::COLOR_BGR2GRAY);
    
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    cv::Mat enhanced;
    clahe->apply(gray, enhanced);
    
    cv::Scalar mean, stddev;
    cv::meanStdDev(enhanced, mean, stddev);
    
    float brightness = static_cast<float>(mean[0]) / 255.0f;
    float contrast = static_cast<float>(stddev[0]) / 255.0f;
    
    float brightness_score = 1.0f;
    if (brightness < 0.15f || brightness > 0.85f) brightness_score = 0.0f;
    else if (brightness < 0.25f || brightness > 0.75f) brightness_score = 0.5f;
    else if (brightness >= 0.35f && brightness <= 0.65f) brightness_score = 1.0f;
    else brightness_score = 0.8f;
    
    float contrast_score = 1.0f;
    if (contrast < 0.08f) contrast_score = 0.3f;
    else if (contrast < 0.15f) contrast_score = 0.7f;
    else contrast_score = 1.0f;
    
    return (brightness_score * 0.6f + contrast_score * 0.4f);
#else
    return 0.5f;
#endif
}

float QualityGate::analyze_motion_stability(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected) {
        return 0.0f;
    }
    
#ifdef HAVE_OPENCV
    cv::Mat color_mat = frame->get_color_mat();
    if (color_mat.empty()) {
        return 0.0f;
    }
    
    cv::Rect roi = face.bbox;
    if (roi.x < 0 || roi.y < 0 || 
        roi.x + roi.width > color_mat.cols || 
        roi.y + roi.height > color_mat.rows) {
        return 0.0f;
    }
    
    cv::Mat face_region = color_mat(roi);
    
    cv::Mat gray;
    cv::cvtColor(face_region, gray, cv::COLOR_BGR2GRAY);
    
    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    float laplacian_variance = static_cast<float>(stddev[0] * stddev[0]);
    
    float motion_score = 1.0f;
    if (laplacian_variance < 50.0f) motion_score = 0.0f;
    else if (laplacian_variance < 100.0f) motion_score = 0.3f;
    else if (laplacian_variance < 200.0f) motion_score = 0.6f;
    else if (laplacian_variance >= 300.0f) motion_score = 1.0f;
    else motion_score = 0.8f;
    
    return motion_score;
#else
    return 0.5f;
#endif
}

float QualityGate::analyze_face_positioning(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected) {
        return 0.0f;
    }
    
#ifdef HAVE_OPENCV
    cv::Mat color_mat = frame->get_color_mat();
    if (color_mat.empty()) {
        return 0.0f;
    }
    
    float img_center_x = color_mat.cols / 2.0f;
    float img_center_y = color_mat.rows / 2.0f;
    
    float face_center_x = face.bbox.x + face.bbox.width / 2.0f;
    float face_center_y = face.bbox.y + face.bbox.height / 2.0f;
    
    float offset_x = std::abs(face_center_x - img_center_x) / img_center_x;
    float offset_y = std::abs(face_center_y - img_center_y) / img_center_y;
    
    float face_size_ratio = (face.bbox.width * face.bbox.height) / (float)(color_mat.cols * color_mat.rows);
    
    // FIXED: Much more lenient centering - real humans move around
    float centering_score = 1.0f;
    if (offset_x > 0.7f || offset_y > 0.7f) centering_score = 0.3f;  // Very off-screen
    else if (offset_x > 0.5f || offset_y > 0.5f) centering_score = 0.6f;  // Quite off-center
    else centering_score = 1.0f;  // Acceptable - humans don't sit perfectly still
    
    // FIXED: More lenient size requirements
    float size_score = 1.0f;
    if (face_size_ratio < 0.01f) size_score = 0.1f;  // Too tiny
    else if (face_size_ratio < 0.03f) size_score = 0.6f;  // Small but usable
    else if (face_size_ratio > 0.6f) size_score = 0.3f;  // Too close (filling screen)
    else if (face_size_ratio > 0.4f) size_score = 0.7f;  // Close but usable
    else size_score = 1.0f;  // Good range: 3-40% of image
    
    return (centering_score * 0.5f + size_score * 0.5f);
#else
    return 0.8f;  // Fallback when no OpenCV
#endif
}

float QualityGate::analyze_sensor_synchronization(const FrameBox* frame) {
    if (!frame) {
        return 0.0f;
    }

    bool has_depth = !frame->depth_data.empty();
    bool has_color = !frame->color_data.empty();
    bool has_ir_left = !frame->ir_left_data.empty();
    bool has_ir_right = !frame->ir_right_data.empty();

    float presence = ((has_depth?1.0f:0.0f) + (has_color?1.0f:0.0f) + (has_ir_left?1.0f:0.0f) + (has_ir_right?1.0f:0.0f)) / 4.0f;

    double times[4]; int n=0;
    if (has_depth) times[n++] = frame->time_depth;
    if (has_color) times[n++] = frame->time_color;
    if (has_ir_left) times[n++] = frame->time_ir_left;
    if (has_ir_right) times[n++] = frame->time_ir_right;
    
    float sync_score = 0.0f;
    if (n >= 2) {
        double min_t = times[0], max_t = times[0];
        for (int i=1;i<n;++i){ min_t = std::min(min_t, times[i]); max_t = std::max(max_t, times[i]); }
        double delta = std::abs(max_t - min_t);
        if (delta <= 10.0) sync_score = 1.0f;
        else if (delta <= 30.0) sync_score = 0.6f;
        else sync_score = 0.2f;
    }

    return 0.5f * presence + 0.5f * sync_score;
}

float QualityGate::analyze_camera_stability(const FrameBox* frame) {
    if (!frame) {
        return 0.0f;
    }

    float score = 1.0f;

    if (frame->exposure <= 100.0f || frame->exposure >= 60000.0f) {
        score *= 0.5f;
    }

    if (frame->gain <= 0.0f || frame->gain > 128.0f) {
        score *= 0.7f;
    }

    if (frame->emitter_state == 0) {
        score *= 0.6f;
    }

    return std::max(0.0f, std::min(1.0f, score));
}

std::string QualityGate::generate_quality_issues(const FrameBox* frame) {
    std::string issues = "";
    
    if (!frame->metadata.quality_gate.lighting_ok) {
        issues += "Poor lighting; ";
    }
    if (!frame->metadata.quality_gate.motion_stable) {
        issues += "Too much movement; ";
    }
    if (!frame->metadata.quality_gate.face_positioned) {
        issues += "Face not properly positioned; ";
    }
    if (!frame->metadata.quality_gate.sensors_synced) {
        issues += "Sensor synchronization issues; ";
    }
    if (!frame->metadata.quality_gate.camera_stable) {
        issues += "Camera stability issues; ";
    }
    
    return issues;
}

// ============================================================================
// AntiSpoofingDetector Implementation
// ============================================================================

AntiSpoofingDetector::AntiSpoofingDetector(const AntiSpoofingConfig& config) : config_(config) {
}

bool AntiSpoofingDetector::process_frame(FrameBox* frame) {
    if (!frame) return false;
    
    frame->metadata.anti_spoofing = FrameBoxMetadata::AntiSpoofingResults{};
    
    FaceROI face;
    face.detected = frame->metadata.face_detected;
    if (face.detected) {
        face.bbox = cv::Rect(frame->metadata.face_x, frame->metadata.face_y,
                              frame->metadata.face_w, frame->metadata.face_h);
        face.confidence = frame->metadata.face_detection_confidence;
    }
    
    if (!face.detected) {
        frame->metadata.anti_spoofing.is_live = false;
        frame->metadata.anti_spoofing.confidence = 0.0f;
        frame->metadata.anti_spoofing.rejection_reason = "No face detected";
        return false;
    }
    
    face_history_.push_back(face);
    if (face_history_.size() > TEMPORAL_WINDOW) {
        face_history_.pop_front();
    }
    
    frame->metadata.anti_spoofing.depth_analysis_score = analyze_depth_geometry(frame, face);
    frame->metadata.anti_spoofing.ir_texture_score = analyze_ir_texture(frame, face);
    frame->metadata.anti_spoofing.cross_modal_score = analyze_cross_modal_consistency(frame, face);
    frame->metadata.anti_spoofing.temporal_consistency_score = analyze_temporal_consistency(face);
    
    frame->metadata.anti_spoofing.depth_anomaly_detected = 
        frame->metadata.anti_spoofing.depth_analysis_score < config_.min_depth_analysis_score;
    frame->metadata.anti_spoofing.ir_material_mismatch = 
        frame->metadata.anti_spoofing.ir_texture_score < config_.min_ir_texture_score;
    frame->metadata.anti_spoofing.cross_modal_disagreement = 
        frame->metadata.anti_spoofing.cross_modal_score < config_.min_cross_modal_score;
    
    frame->metadata.anti_spoofing.overall_liveness_score = 
        (frame->metadata.anti_spoofing.depth_analysis_score * 0.40f +
         frame->metadata.anti_spoofing.ir_texture_score * 0.30f +
         frame->metadata.anti_spoofing.temporal_consistency_score * 0.15f +
         frame->metadata.anti_spoofing.cross_modal_score * 0.15f);
    
    frame->metadata.anti_spoofing.detected_attack_type = detect_attack_type(frame, face);
    
    frame->metadata.anti_spoofing.confidence = calculate_confidence(frame, face);
    
    bool depth_ok = frame->metadata.anti_spoofing.depth_analysis_score >= config_.min_depth_analysis_score;
    bool ir_ok = frame->metadata.anti_spoofing.ir_texture_score >= config_.min_ir_texture_score;
    bool xmodal_ok = frame->metadata.anti_spoofing.cross_modal_score >= config_.min_cross_modal_score;
    bool temporal_ok = frame->metadata.anti_spoofing.temporal_consistency_score >= config_.min_temporal_consistency_score;

    bool logic_ok = (depth_ok && temporal_ok) || (depth_ok && ir_ok) || (xmodal_ok && ir_ok && temporal_ok);
    bool passes_confidence = frame->metadata.anti_spoofing.confidence >= config_.min_confidence;
    
    frame->metadata.anti_spoofing.is_live = logic_ok && passes_confidence;
    
    if (!frame->metadata.anti_spoofing.is_live) {
        frame->metadata.anti_spoofing.rejection_reason = generate_rejection_reason(frame);
    }
    
    frame->metadata.anti_spoofing_processed = true;
    return frame->metadata.anti_spoofing.is_live;
}

float AntiSpoofingDetector::process_temporal_analysis(const std::vector<FrameBox*>& frames, FrameBox* current_frame) {
    if (frames.empty() || !current_frame) {
        return 0.0f;
    }
    
    current_frame->metadata.anti_spoofing.temporal_consistency_score = 0.8f;
    current_frame->metadata.anti_spoofing.temporal_inconsistency = false;
    
    return current_frame->metadata.anti_spoofing.temporal_consistency_score;
}

float AntiSpoofingDetector::analyze_depth_geometry(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || frame->depth_data.empty() || frame->depth_width <= 0 || frame->depth_height <= 0) {
        return 0.0f;
    }

#ifdef HAVE_OPENCV
    cv::Rect depth_roi;
    if (frame->color_width > 0 && frame->color_height > 0) {
        depth_roi.x = static_cast<int>(face.bbox.x * static_cast<float>(frame->depth_width) / static_cast<float>(frame->color_width));
        depth_roi.y = static_cast<int>(face.bbox.y * static_cast<float>(frame->depth_height) / static_cast<float>(frame->color_height));
        depth_roi.width = static_cast<int>(face.bbox.width * static_cast<float>(frame->depth_width) / static_cast<float>(frame->color_width));
        depth_roi.height = static_cast<int>(face.bbox.height * static_cast<float>(frame->depth_height) / static_cast<float>(frame->color_height));
    } else {
        depth_roi = face.bbox;
    }
    
    depth_roi.x = std::max(0, std::min(depth_roi.x, frame->depth_width - 1));
    depth_roi.y = std::max(0, std::min(depth_roi.y, frame->depth_height - 1));
    depth_roi.width = std::min(depth_roi.width, frame->depth_width - depth_roi.x);
    depth_roi.height = std::min(depth_roi.height, frame->depth_height - depth_roi.y);
    
    if (depth_roi.width <= 0 || depth_roi.height <= 0) {
        return 0.0f;
    }
    
    std::vector<cv::Point3f> face_points_all;
    std::vector<float> depth_values_all;
    face_points_all.reserve(static_cast<size_t>(depth_roi.width * depth_roi.height / 4));
    
    for (int y = depth_roi.y; y < depth_roi.y + depth_roi.height; y += 2) {
        for (int x = depth_roi.x; x < depth_roi.x + depth_roi.width; x += 2) {
            int idx = y * frame->depth_width + x;
            if (idx < (int)frame->depth_data.size()) {
                uint16_t depth_raw = frame->depth_data[idx];
                if (depth_raw > 0) {
                    float z = depth_raw * frame->depth_scale;
                    float pix[2] = { static_cast<float>(x), static_cast<float>(y) };
                    float p3[3];
                    rs2_deproject_pixel_to_point(p3, &frame->depth_intrinsics, pix, z);
                    face_points_all.emplace_back(p3[0], p3[1], p3[2]);
                    depth_values_all.push_back(p3[2]);
                }
            }
        }
    }
    
    if (face_points_all.size() < 10) {
        return 0.0f;
    }

    std::vector<float> tmp = depth_values_all;
    std::nth_element(tmp.begin(), tmp.begin() + tmp.size()/2, tmp.end());
    float median_z = tmp[tmp.size()/2];
    const float band = 0.10f;
    
    std::vector<cv::Point3f> face_points;
    std::vector<float> depth_values;
    face_points.reserve(face_points_all.size());
    depth_values.reserve(depth_values_all.size());
    
    for (size_t i=0;i<face_points_all.size();++i){
        if (std::abs(depth_values_all[i]-median_z) <= band){
            face_points.push_back(face_points_all[i]);
            depth_values.push_back(depth_values_all[i]);
        }
    }
    
    if (face_points.size() < 10) return 0.0f;
    
    float geometry_score = 0.0f;
    
    // 1. Plane fitting
    if (face_points.size() >= 3) {
        cv::Mat A((int)face_points.size(), 3, CV_32F);
        cv::Mat B((int)face_points.size(), 1, CV_32F);
        for (size_t i = 0; i < face_points.size(); i++) {
            A.at<float>((int)i, 0) = face_points[i].x;
            A.at<float>((int)i, 1) = face_points[i].y;
            A.at<float>((int)i, 2) = 1.0f;
            B.at<float>((int)i, 0) = face_points[i].z;
        }
        cv::Mat plane_params;
        cv::solve(A, B, plane_params, cv::DECOMP_SVD);

        float total_error = 0.0f;
        for (size_t i = 0; i < face_points.size(); i++) {
            float predicted_z = plane_params.at<float>(0) * face_points[i].x +
                                plane_params.at<float>(1) * face_points[i].y +
                                plane_params.at<float>(2);
            total_error += std::abs(face_points[i].z - predicted_z);
        }
        float avg_plane_error = total_error / face_points.size();
        
        if (avg_plane_error < 0.008f) {
            return 0.0f;
        } else if (avg_plane_error < 0.015f) {
            geometry_score += 0.3f;
        } else if (avg_plane_error < 0.025f) {
            geometry_score += 0.7f;
        } else {
            geometry_score += 1.0f;
        }
    }
    
    // 2. Depth range
    if (depth_values.size() >= 5) {
        float min_depth = *std::min_element(depth_values.begin(), depth_values.end());
        float max_depth = *std::max_element(depth_values.begin(), depth_values.end());
        float depth_range = max_depth - min_depth;
        
        if (depth_range < 0.025f) {
            geometry_score += 0.0f;
        } else if (depth_range < 0.035f) {
            geometry_score += 0.4f;
        } else if (depth_range > 0.15f) {
            geometry_score += 0.2f;
        } else {
            geometry_score += 1.0f;
        }
    }
    
    // 3. Surface normal variance
    if (face_points.size() >= 9) {
        float normal_variance = 0.0f;
        int valid_normals = 0;
        
        for (int y = 1; y < depth_roi.height - 1; y++) {
            for (int x = 1; x < depth_roi.width - 1; x++) {
                int center_idx = y * frame->depth_width + (depth_roi.x + x);
                int right_idx = center_idx + 1;
                int down_idx = center_idx + frame->depth_width;
                
                if (center_idx < frame->depth_data.size() && 
                    right_idx < frame->depth_data.size() && 
                    down_idx < frame->depth_data.size()) {
                    
                    uint16_t center_depth = frame->depth_data[center_idx];
                    uint16_t right_depth = frame->depth_data[right_idx];
                    uint16_t down_depth = frame->depth_data[down_idx];
                    
                    if (center_depth > 0 && right_depth > 0 && down_depth > 0) {
                        float dx = (right_depth - center_depth) * 0.001f;
                        float dy = (down_depth - center_depth) * 0.001f;
                        float gradient_magnitude = std::sqrt(dx*dx + dy*dy);
                        normal_variance += gradient_magnitude;
                        valid_normals++;
                    }
                }
            }
        }
        
        if (valid_normals > 0) {
            normal_variance /= valid_normals;
            
            if (normal_variance < 0.002f) {
                geometry_score += 0.0f;
            } else if (normal_variance > 0.05f) {
                geometry_score += 0.3f;
            } else {
                geometry_score += 1.0f;
            }
        }
    }
    
    return geometry_score / 3.0f;
#else
    return 0.5f;
#endif
}

float AntiSpoofingDetector::analyze_ir_texture(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || frame->ir_left_data.empty() || frame->ir_width <= 0 || frame->ir_height <= 0) {
        return 0.0f;
    }

#ifdef HAVE_OPENCV
    cv::Rect ir_roi;
    if (frame->color_width > 0 && frame->color_height > 0) {
        ir_roi.x = static_cast<int>(face.bbox.x * static_cast<float>(frame->ir_width) / static_cast<float>(frame->color_width));
        ir_roi.y = static_cast<int>(face.bbox.y * static_cast<float>(frame->ir_height) / static_cast<float>(frame->color_height));
        ir_roi.width = static_cast<int>(face.bbox.width * static_cast<float>(frame->ir_width) / static_cast<float>(frame->color_width));
        ir_roi.height = static_cast<int>(face.bbox.height * static_cast<float>(frame->ir_height) / static_cast<float>(frame->color_height));
    } else {
        ir_roi = face.bbox;
    }
    
    ir_roi.x = std::max(0, std::min(ir_roi.x, frame->ir_width - 1));
    ir_roi.y = std::max(0, std::min(ir_roi.y, frame->ir_height - 1));
    ir_roi.width = std::min(ir_roi.width, frame->ir_width - ir_roi.x);
    ir_roi.height = std::min(ir_roi.height, frame->ir_height - ir_roi.y);
    
    if (ir_roi.width <= 0 || ir_roi.height <= 0) {
        return 0.0f;
    }
    
    cv::Mat ir_left(frame->ir_height, frame->ir_width, CV_8UC1, 
                    const_cast<uint8_t*>(frame->ir_left_data.data()));
    cv::Mat face_ir = ir_left(ir_roi);

    float texture_score = 0.0f;

    // 1. IR standard deviation check
    cv::Scalar mean_ir, std_ir;
    cv::meanStdDev(face_ir, mean_ir, std_ir);
    float ir_std = static_cast<float>(std_ir[0]);
    float mean_intensity = static_cast<float>(mean_ir[0]);
    
    // FIXED: Make thresholds more realistic for real humans
    if (ir_std < 12.0f) {
        texture_score += 0.0f;  // Too uniform (screen)
    } else if (ir_std < 18.0f) {
        texture_score += 0.5f;  // Borderline - could be real human
    } else if (ir_std < 35.0f) {
        texture_score += 1.0f;  // Good skin texture
    } else if (ir_std < 60.0f) {
        texture_score += 0.7f;  // High texture (textured mask possible)
    } else {
        texture_score += 0.3f;  // Very noisy
    }
    
    // 2. Mean intensity check
    if (mean_intensity > 200.0f) {
        texture_score += 0.0f;  // Screen emission
    } else if (mean_intensity < 20.0f) {
        texture_score += 0.0f;  // Too dark
    } else if (mean_intensity >= 30.0f && mean_intensity <= 150.0f) {
        texture_score += 1.0f;  // Normal skin reflection
    } else {
        texture_score += 0.5f;  // Borderline
    }
    
    // 3. Texture pattern analysis (LBP)
    cv::Mat lbp = cv::Mat::zeros(face_ir.size(), CV_8UC1);
    for (int y = 1; y < face_ir.rows - 1; y++) {
        for (int x = 1; x < face_ir.cols - 1; x++) {
            uint8_t center = face_ir.at<uint8_t>(y, x);
            uint8_t pattern = 0;
            
            if (face_ir.at<uint8_t>(y-1, x-1) >= center) pattern |= 1;
            if (face_ir.at<uint8_t>(y-1, x) >= center) pattern |= 2;
            if (face_ir.at<uint8_t>(y-1, x+1) >= center) pattern |= 4;
            if (face_ir.at<uint8_t>(y, x+1) >= center) pattern |= 8;
            if (face_ir.at<uint8_t>(y+1, x+1) >= center) pattern |= 16;
            if (face_ir.at<uint8_t>(y+1, x) >= center) pattern |= 32;
            if (face_ir.at<uint8_t>(y+1, x-1) >= center) pattern |= 64;
            if (face_ir.at<uint8_t>(y, x-1) >= center) pattern |= 128;
            
            lbp.at<uint8_t>(y, x) = pattern;
        }
    }
    
    cv::Mat hist;
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    cv::calcHist(&lbp, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);
    cv::normalize(hist, hist, 0, 1, cv::NORM_L1);
    
    float uniform_patterns = 0.0f;
    for (int i = 0; i < 256; i++) {
        int transitions = 0;
        for (int j = 0; j < 8; j++) {
            int bit1 = (i >> j) & 1;
            int bit2 = (i >> ((j + 1) % 8)) & 1;
            if (bit1 != bit2) transitions++;
        }
        
        if (transitions <= 2) {
            uniform_patterns += hist.at<float>(i);
        }
    }
    
    if (uniform_patterns < 0.5f) {
        texture_score += 0.0f;
    } else if (uniform_patterns > 0.88f) {
        texture_score += 0.0f;
    } else if (uniform_patterns >= 0.6f && uniform_patterns <= 0.85f) {
        texture_score += 1.0f;
    } else {
        texture_score += 0.5f;
    }
    
    return texture_score / 3.0f;
#else
    return 0.5f;
#endif
}

float AntiSpoofingDetector::analyze_temporal_consistency(const FaceROI& face) {
    if (!face.detected || face_history_.size() < 3) {
        return 0.7f;  // Give benefit of doubt early on
    }
    
    float temporal_score = 0.0f;
    
    // 1. Micro-motion detection - FIXED: Be more lenient with steady subjects
    float total_motion = 0.0f;
    int motion_samples = 0;
    for (size_t i = 1; i < face_history_.size(); i++) {
        const FaceROI& prev = face_history_[i-1];
        const FaceROI& curr = face_history_[i];
        
        if (prev.detected && curr.detected) {
            float dx = curr.bbox.x - prev.bbox.x;
            float dy = curr.bbox.y - prev.bbox.y;
            float motion = std::sqrt(dx*dx + dy*dy);
            total_motion += motion;
            motion_samples++;
        }
    }
    
    float avg_motion = (motion_samples > 0) ? total_motion / motion_samples : 0.0f;
    
    // FIXED: Real humans can be steady - don't penalize stillness heavily
    if (avg_motion < 0.02f) {
        temporal_score += 0.7f;  // Very steady - could be photo, but also focused human
    } else if (avg_motion > 20.0f) {
        temporal_score += 0.5f;  // Too much motion - unstable or evasion
    } else {
        temporal_score += 1.0f;  // Natural micro-motion range
    }
    
    // 2. Face size variance (breathing/natural head movement)
    if (face_history_.size() >= 10) {
        float size_variance = 0.0f;
        float avg_size = 0.0f;
        int size_samples = 0;
        
        for (const auto& f : face_history_) {
            if (f.detected) {
                float size = f.bbox.width * f.bbox.height;
                avg_size += size;
                size_samples++;
            }
        }
        
        if (size_samples > 0) {
            avg_size /= size_samples;
            
            for (const auto& f : face_history_) {
                if (f.detected) {
                    float size = f.bbox.width * f.bbox.height;
                    float diff = size - avg_size;
                    size_variance += diff * diff;
                }
            }
            size_variance /= size_samples;
            
            // Natural breathing and subtle head movement cause size variance
            float normalized_variance = size_variance / (avg_size * avg_size + 1e-6f);
            
            if (normalized_variance > 0.0001f) {
                temporal_score += 1.0f;  // Natural variance detected
            } else {
                temporal_score += 0.7f;  // Very stable - not necessarily bad
            }
        } else {
            temporal_score += 0.7f;
        }
    } else {
        temporal_score += 0.8f;  // Not enough history yet
    }
    
    return temporal_score / 2.0f;
}

float AntiSpoofingDetector::analyze_cross_modal_consistency(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected) {
        return 0.0f;
    }

#ifdef HAVE_OPENCV
    float score = 0.0f;
    int checks = 0;

    // OPTIMIZED: Depth vs Color edge consistency with better handling
    if (!frame->depth_data.empty() && !frame->color_data.empty()) {
        cv::Mat color = frame->get_color_mat();
        if (!color.empty()) {
            cv::Rect croi = face.bbox & cv::Rect(0,0,color.cols,color.rows);
            if (croi.width > 10 && croi.height > 10) {
                // Extract color edges
                cv::Mat gray, edges;
                cv::cvtColor(color(croi), gray, cv::COLOR_BGR2GRAY);
                cv::Canny(gray, edges, 50, 150);
                float color_edge_density = static_cast<float>(cv::countNonZero(edges)) / (edges.total() + 1e-6f);

                // Map to depth space with alignment
                int dx = (frame->color_width>0)? static_cast<int>(croi.x * (float)frame->depth_width / (float)frame->color_width) : croi.x;
                int dy = (frame->color_height>0)? static_cast<int>(croi.y * (float)frame->depth_height / (float)frame->color_height) : croi.y;
                int dw = (frame->color_width>0)? static_cast<int>(croi.width * (float)frame->depth_width / (float)frame->color_width) : croi.width;
                int dh = (frame->color_height>0)? static_cast<int>(croi.height * (float)frame->depth_height / (float)frame->color_height) : croi.height;
                dx = std::max(0, std::min(dx, frame->depth_width-1));
                dy = std::max(0, std::min(dy, frame->depth_height-1));
                dw = std::min(dw, frame->depth_width - dx);
                dh = std::min(dh, frame->depth_height - dy);
                
                if (dw>10 && dh>10) {
                    // OPTIMIZED: Sample depth instead of full ROI processing
                    std::vector<float> depth_samples;
                    depth_samples.reserve(dw * dh / 4);
                    
                    for (int j=0; j<dh; j+=2){  // Sample every 2nd pixel
                        for (int i=0; i<dw; i+=2){
                            int idx = (dy+j)*frame->depth_width + (dx+i);
                            if (idx < (int)frame->depth_data.size() && frame->depth_data[idx] > 0) {
                                float z = frame->depth_data[idx] * frame->depth_scale;
                                if (z > 0.1f && z < 5.0f) {  // Valid range
                                    depth_samples.push_back(z);
                                }
                            }
                        }
                    }
                    
                    if (depth_samples.size() > 20) {
                        // Calculate depth variance as proxy for edges/structure
                        float mean = std::accumulate(depth_samples.begin(), depth_samples.end(), 0.0f) / depth_samples.size();
                        float variance = 0.0f;
                        for (float d : depth_samples) {
                            variance += (d - mean) * (d - mean);
                        }
                        variance /= depth_samples.size();
                        
                        float depth_structure = std::sqrt(variance) * 100.0f;  // Convert to cm
                        
                        // Cross-modal logic: Real faces have BOTH color and depth structure
                        // Flat attacks have color edges but minimal depth structure
                        if (color_edge_density > 0.12f && depth_structure < 2.0f) {
                            score += 0.0f;  // High color edges, flat depth = ATTACK
                        } else if (depth_structure > 3.0f) {
                            score += 1.0f;  // Good 3D structure = REAL
                        } else if (depth_structure > 1.5f) {
                            score += 0.7f;  // Moderate structure
                        } else {
                            score += 0.4f;  // Borderline
                        }
                        checks++;
                    }
                }
            }
        }
    }

    // Return with fallback for insufficient data
    if (checks == 0) return 0.6f;  // Give benefit of doubt when data unavailable
    return score / checks;
#else
    return 0.5f;
#endif
}

bool AntiSpoofingDetector::detect_blink(const std::vector<cv::Point2f>& landmarks) {
    // Placeholder for blink detection
    return false;
}

float AntiSpoofingDetector::calculate_micro_motion() {
    // Placeholder for micro-motion calculation
    return 0.5f;
}

float AntiSpoofingDetector::analyze_depth_breathing(const FrameBox* frame, const FaceROI& face) {
    // Placeholder for breathing analysis
    return 0.5f;
}

std::string AntiSpoofingDetector::detect_attack_type(const FrameBox* frame, const FaceROI& face) {
    if (!frame) {
        return "unknown";
    }
    
    if (frame->metadata.anti_spoofing.depth_analysis_score < 0.4f) {
        return "flat_surface";
    } else if (frame->metadata.anti_spoofing.ir_texture_score < 0.4f) {
        return "material_mismatch";
    } else if (frame->metadata.anti_spoofing.cross_modal_score < 0.4f) {
        return "cross_modal_attack";
    }
    
    return "none";
}

std::string AntiSpoofingDetector::generate_rejection_reason(const FrameBox* frame) {
    std::vector<std::string> reasons;
    
    if (frame->metadata.anti_spoofing.depth_anomaly_detected) {
        reasons.push_back("flat surface detected");
    }
    if (frame->metadata.anti_spoofing.ir_material_mismatch) {
        reasons.push_back("material mismatch");
    }
    if (frame->metadata.anti_spoofing.temporal_inconsistency) {
        reasons.push_back("no natural movement");
    }
    if (frame->metadata.anti_spoofing.cross_modal_disagreement) {
        reasons.push_back("sensor disagreement");
    }
    
    if (reasons.empty()) {
        return "low confidence";
    }
    
    std::string result = "";
    for (size_t i = 0; i < reasons.size(); i++) {
        result += reasons[i];
        if (i < reasons.size() - 1) result += ", ";
    }
    return result;
}

float AntiSpoofingDetector::calculate_confidence(const FrameBox* frame, const FaceROI& face) {
    if (!frame) {
        return 0.0f;
    }

    float d = frame->metadata.anti_spoofing.depth_analysis_score;
    float ir = frame->metadata.anti_spoofing.ir_texture_score;
    float cm = frame->metadata.anti_spoofing.cross_modal_score;
    float t = frame->metadata.anti_spoofing.temporal_consistency_score;

    float min_comp = std::min(std::min(d, ir), std::min(cm, t));
    float mean_comp = (d + ir + cm + t) / 4.0f;

    float confidence = 0.6f * min_comp + 0.4f * mean_comp;
    return confidence;
}

// ============================================================================
// AntiSpoofingPipeline Implementation
// ============================================================================

AntiSpoofingPipeline::AntiSpoofingPipeline(const AntiSpoofingConfig& config) 
    : config_(config) {
    quality_gate_ = std::make_unique<QualityGate>(config_);
    anti_spoofing_detector_ = std::make_unique<AntiSpoofingDetector>(config_);
}

bool AntiSpoofingPipeline::process_frame(FrameBox* frame) {
    if (!frame) return false;
    
    bool quality_passed = quality_gate_->process_frame(frame);
    
    bool anti_spoofing_passed = false;
    if (quality_passed) {
        anti_spoofing_passed = anti_spoofing_detector_->process_frame(frame);
    }
    
    bool overall_passed = quality_passed && anti_spoofing_passed;
    
    frame->metadata.is_ready_for_processing = overall_passed;
    if (!overall_passed) {
        if (!quality_passed) {
            frame->metadata.final_rejection_reason = frame->metadata.quality_gate.quality_issues;
        } else {
            frame->metadata.final_rejection_reason = frame->metadata.anti_spoofing.rejection_reason;
        }
    }
    
    update_stats(quality_passed, anti_spoofing_passed, overall_passed);
    
    return overall_passed;
}

bool AntiSpoofingPipeline::process_frame_with_temporal_context(FrameBox* frame, const std::vector<FrameBox*>& recent_frames) {
    if (!frame) return false;
    
    bool quality_passed = quality_gate_->process_frame(frame);
    
    bool anti_spoofing_passed = false;
    if (quality_passed) {
        anti_spoofing_passed = anti_spoofing_detector_->process_frame(frame);
        
        float temporal_score = anti_spoofing_detector_->process_temporal_analysis(recent_frames, frame);
        frame->metadata.anti_spoofing.temporal_consistency_score = temporal_score;
        
        frame->metadata.anti_spoofing.overall_liveness_score = 
            (frame->metadata.anti_spoofing.depth_analysis_score * 0.30f +
             frame->metadata.anti_spoofing.ir_texture_score * 0.25f +
             frame->metadata.anti_spoofing.cross_modal_score * 0.20f +
             frame->metadata.anti_spoofing.temporal_consistency_score * 0.25f);
        
        anti_spoofing_passed = 
            frame->metadata.anti_spoofing.overall_liveness_score >= config_.min_overall_liveness &&
            frame->metadata.anti_spoofing.confidence >= config_.min_confidence;
    }
    
    bool overall_passed = quality_passed && anti_spoofing_passed;
    
    frame->metadata.is_ready_for_processing = overall_passed;
    if (!overall_passed) {
        if (!quality_passed) {
            frame->metadata.final_rejection_reason = frame->metadata.quality_gate.quality_issues;
        } else {
            frame->metadata.final_rejection_reason = frame->metadata.anti_spoofing.rejection_reason;
        }
    }
    
    update_stats(quality_passed, anti_spoofing_passed, overall_passed);
    
    return overall_passed;
}

void AntiSpoofingPipeline::update_stats(bool quality_passed, bool anti_spoofing_passed, bool overall_passed) {
    stats_.total_frames_processed++;
    
    if (quality_passed) {
        stats_.quality_gate_passed++;
    } else {
        stats_.quality_gate_failed++;
    }
    
    if (anti_spoofing_passed) {
        stats_.anti_spoofing_passed++;
    } else {
        stats_.anti_spoofing_failed++;
    }
    
    if (overall_passed) {
        stats_.overall_passed++;
    } else {
        stats_.overall_failed++;
    }
    
    if (stats_.total_frames_processed > 0) {
        stats_.quality_gate_pass_rate = static_cast<float>(stats_.quality_gate_passed) / stats_.total_frames_processed;
        stats_.anti_spoofing_pass_rate = static_cast<float>(stats_.anti_spoofing_passed) / stats_.total_frames_processed;
        stats_.overall_pass_rate = static_cast<float>(stats_.overall_passed) / stats_.total_frames_processed;
    }
}

// ============================================================================
// AdaptiveThresholdManager Implementation
// ============================================================================

AdaptiveThresholdManager::AdaptiveThresholdManager() {
}

AntiSpoofingConfig AdaptiveThresholdManager::get_adaptive_config(
    const AntiSpoofingConfig& base_config,
    const std::map<std::string, float>& environmental_factors) {
    
    AntiSpoofingConfig adapted_config = base_config;
    return adapted_config;
}

void AdaptiveThresholdManager::learn_from_result(const FrameBox* frame, bool success) {
    learning_stats_.total_samples++;
    if (success) {
        learning_stats_.successful_samples++;
    }
    
    learning_stats_.success_rate = static_cast<float>(learning_stats_.successful_samples) / learning_stats_.total_samples;
}

} // namespace mdai
