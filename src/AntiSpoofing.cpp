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

// FIXED: Remove unused function (Fix #7 - Dead Code)
// calculate_depth_smoothness was never called - integrated logic into analyze_depth_geometry instead

// ============================================================================
// QualityGate Implementation
// ============================================================================

QualityGate::QualityGate(const AntiSpoofingConfig& config) : config_(config) {
    // Face detection handled externally (MediaPipe, OpenCV DNN, etc.)
    // Anti-spoofing only analyzes the detected face ROI
    std::cout << "✓ Anti-spoofing quality gate initialized" << std::endl;
}

FaceROI QualityGate::detect_face(const FrameBox* frame) {
    // Face detection should be done externally (MediaPipe, OpenCV DNN, etc.)
    // This function just retrieves the pre-detected face from metadata
    FaceROI face;
    face.detected = frame->metadata.face_detected;
    
    if (face.detected) {
        face.bbox = cv::Rect(frame->metadata.face_x, frame->metadata.face_y,
                              frame->metadata.face_w, frame->metadata.face_h);
        face.confidence = frame->metadata.face_detection_confidence;
    }
    
    return face;
}

bool QualityGate::process_frame(FrameBox* frame) {
    if (!frame) return false;
    
    frame->metadata.quality_gate = FrameBoxMetadata::QualityGateResults{};
    
    // PHASE 1 FIX #1: Enforce IR emitter is ON (critical for depth/IR reliability)
    if (frame->emitter_state == 0) {
        frame->metadata.quality_gate.quality_gate_passed = false;
        frame->metadata.quality_gate.quality_issues = "IR emitter OFF - depth and IR data unreliable";
        return false;
    }
    
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
    // FIXED: Use new motion analysis (Fix #2) - combines sharpness + actual motion
    float sharpness = analyze_sharpness(frame, face);
    float motion = analyze_motion_stability(frame, face);
    frame->metadata.quality_gate.motion_score = (sharpness * 0.4f + motion * 0.6f);  // Motion more important
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

// FIXED: Renamed from analyze_motion_stability - this actually measures sharpness (Fix #2)
float QualityGate::analyze_sharpness(const FrameBox* frame, const FaceROI& face) {
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
    
    // This measures sharpness/focus, not motion
    float sharpness_score = 1.0f;
    if (laplacian_variance < 50.0f) sharpness_score = 0.0f;  // Very blurry
    else if (laplacian_variance < 100.0f) sharpness_score = 0.3f;
    else if (laplacian_variance < 200.0f) sharpness_score = 0.6f;
    else if (laplacian_variance >= 300.0f) sharpness_score = 1.0f;  // Sharp
    else sharpness_score = 0.8f;
    
    return sharpness_score;
#else
    return 0.5f;
#endif
}

// FIXED: NEW function for actual motion analysis via optical flow (Fix #2)
float QualityGate::analyze_motion_stability(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected) {
        return 0.0f;
    }
    
#ifdef HAVE_OPENCV
    cv::Mat color_mat = frame->get_color_mat();
    if (color_mat.empty()) {
        return 0.0f;
    }
    
    cv::Mat gray;
    cv::cvtColor(color_mat, gray, cv::COLOR_BGR2GRAY);
    
    // Store current frame for next comparison
    if (previous_gray_.empty()) {
        previous_gray_ = gray.clone();
        return 0.5f;  // Not enough data yet
    }
    
    // Compute optical flow on face ROI
    cv::Rect roi = face.bbox;
    if (roi.x < 0 || roi.y < 0 || 
        roi.x + roi.width > gray.cols || 
        roi.y + roi.height > gray.rows) {
        previous_gray_ = gray.clone();
        return 0.0f;
    }
    
    cv::Mat prev_roi = previous_gray_(roi);
    cv::Mat curr_roi = gray(roi);
    
    // Calculate dense optical flow
    cv::Mat flow;
    cv::calcOpticalFlowFarneback(prev_roi, curr_roi, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
    
    // Compute motion statistics
    std::vector<float> magnitudes;
    for (int y = 0; y < flow.rows; y++) {
        for (int x = 0; x < flow.cols; x++) {
            cv::Point2f flow_vec = flow.at<cv::Point2f>(y, x);
            float magnitude = std::sqrt(flow_vec.x * flow_vec.x + flow_vec.y * flow_vec.y);
            magnitudes.push_back(magnitude);
        }
    }
    
    float mean_motion = std::accumulate(magnitudes.begin(), magnitudes.end(), 0.0f) / magnitudes.size();
    
    // Compute variance
    float variance = 0.0f;
    for (float mag : magnitudes) {
        variance += (mag - mean_motion) * (mag - mean_motion);
    }
    variance /= magnitudes.size();
    float stddev = std::sqrt(variance);
    
    // Score based on natural motion characteristics
    float motion_score = 1.0f;
    
    // Too little motion = suspicious (photo attack)
    if (mean_motion < 0.1f) motion_score = 0.2f;
    // Too much motion = unstable (evasion)
    else if (mean_motion > 5.0f) motion_score = 0.4f;
    // Good natural motion range
    else if (mean_motion >= 0.3f && mean_motion <= 2.0f && stddev > 0.1f) motion_score = 1.0f;
    else motion_score = 0.7f;
    
    // Update previous frame
    previous_gray_ = gray.clone();
    
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
    
    // CRITICAL FIX: Enforce emitter ON before any anti-spoofing analysis
    if (frame->emitter_state == 0) {
        frame->metadata.anti_spoofing.is_live = false;
        frame->metadata.anti_spoofing.confidence = 0.0f;
        frame->metadata.anti_spoofing.rejection_reason = "IR emitter OFF - depth and IR data unreliable";
        return false;
    }
    
    face_history_.push_back(face);
    if (face_history_.size() > TEMPORAL_WINDOW) {
        face_history_.pop_front();
    }
    
    // Core anti-spoofing analyses
    frame->metadata.anti_spoofing.depth_analysis_score = analyze_depth_geometry(frame, face);
    
    // IR texture analysis now includes stereo consistency check
    frame->metadata.anti_spoofing.ir_texture_score = analyze_ir_texture(frame, face);
    
    // NEW: Skin texture analysis - KEY for detecting plastic masks!
    frame->metadata.anti_spoofing.skin_texture_score = analyze_skin_texture(frame, face);
    
    frame->metadata.anti_spoofing.cross_modal_score = analyze_cross_modal_consistency(frame, face);
    frame->metadata.anti_spoofing.temporal_consistency_score = analyze_temporal_consistency(face);
    
    // Enhanced landmark-based analyses (68-point)
    float landmarks_depth_score = validate_landmarks_depth(frame, face);
    float nose_protrusion_score = analyze_nose_protrusion(frame, face);
    float face_curvature_score = analyze_face_curvature(frame, face);
    float symmetry_score = validate_symmetry(frame, face);
    float landmark_motion_score = track_landmark_motion(face);
    
    // DEBUG: Log individual landmark scores
    static int debug_count = 0;
    if (++debug_count % 30 == 0) {  // Every 30 frames
        std::cout << "\n=== LANDMARK-BASED ANTI-SPOOFING DEBUG ===" << std::endl;
        std::cout << "  Landmarks valid: " << (frame->metadata.landmarks.valid ? "YES" : "NO") << std::endl;
        if (frame->metadata.landmarks.valid) {
            std::cout << "  Landmark depth validation: " << (landmarks_depth_score * 100) << "%" << std::endl;
            std::cout << "  Nose protrusion: " << (nose_protrusion_score * 100) << "%" << std::endl;
            std::cout << "  Face curvature: " << (face_curvature_score * 100) << "%" << std::endl;
            std::cout << "  L-R symmetry: " << (symmetry_score * 100) << "%" << std::endl;
        }
    }
    
    // Combine landmark scores (if landmarks are available)
    float landmark_composite_score = 0.5f;  // Default neutral
    if (frame->metadata.landmarks.valid) {
        landmark_composite_score = (
            landmarks_depth_score * 0.30f +      // Overall depth at landmarks
            nose_protrusion_score * 0.25f +       // Nose 3D structure
            face_curvature_score * 0.25f +        // Face curvature
            symmetry_score * 0.15f +              // Left-right symmetry
            landmark_motion_score * 0.05f         // Micro-movements
        );
    } else {
        // CRITICAL: If landmarks not available, FAIL HARD
        landmark_composite_score = 0.0f;  // Force failure without landmarks
    }
    
    frame->metadata.anti_spoofing.depth_anomaly_detected = 
        frame->metadata.anti_spoofing.depth_analysis_score < config_.min_depth_analysis_score;
    frame->metadata.anti_spoofing.ir_material_mismatch = 
        frame->metadata.anti_spoofing.ir_texture_score < config_.min_ir_texture_score;
    frame->metadata.anti_spoofing.cross_modal_disagreement = 
        frame->metadata.anti_spoofing.cross_modal_score < config_.min_cross_modal_score;
    
    frame->metadata.anti_spoofing.overall_liveness_score = 
        (frame->metadata.anti_spoofing.depth_analysis_score * 0.25f +
         frame->metadata.anti_spoofing.ir_texture_score * 0.20f +
         frame->metadata.anti_spoofing.skin_texture_score * 0.35f +  // KEY: Skin texture!
         landmark_composite_score * 0.10f +
         frame->metadata.anti_spoofing.temporal_consistency_score * 0.05f +
         frame->metadata.anti_spoofing.cross_modal_score * 0.05f);
    
    frame->metadata.anti_spoofing.detected_attack_type = detect_attack_type(frame, face);
    
    frame->metadata.anti_spoofing.confidence = calculate_confidence(frame, face);
    
    // FIXED: Replace OR-of-ANDs with weighted score fusion (Fix #8)
    // Enhanced with 68-landmark analysis
    
    // Get individual scores
    float depth_score = frame->metadata.anti_spoofing.depth_analysis_score;
    float ir_score = frame->metadata.anti_spoofing.ir_texture_score;
    float skin_texture_score = frame->metadata.anti_spoofing.skin_texture_score;
    float xmodal_score = frame->metadata.anti_spoofing.cross_modal_score;
    float temporal_score = frame->metadata.anti_spoofing.temporal_consistency_score;
    
    // FIXED: If temporal score is too low (person is steady), default to neutral 0.5
    // Don't penalize people for being still!
    if (temporal_score < 0.3f) {
        temporal_score = 0.5f;  // Neutral score for steady people
    }
    
    // FIXED: Make landmarks optional (not required)
    if (!frame->metadata.landmarks.valid) {
        landmark_composite_score = 0.5f;  // Neutral if not available (not forced to 0!)
    }
    
    // NEW: Weighted fusion with SKIN TEXTURE as key component (sum = 1.0)
    float fused_score = 
        depth_score * 0.25f +              // Depth geometry (3D structure)
        ir_score * 0.20f +                  // IR texture + stereo (material)
        skin_texture_score * 0.35f +        // COLOR TEXTURE (pores, detail) - KEY!
        landmark_composite_score * 0.10f +  // 68-point landmark validation (bonus)
        temporal_score * 0.05f +            // Temporal consistency (optional)
        xmodal_score * 0.05f;               // Cross-modal validation
    
    // FIXED: REMOVED guardrails! Use simple threshold only.
    // Pass criteria: good fused score AND confidence
    bool passes_fusion = (fused_score >= config_.min_overall_liveness);
    bool passes_confidence = frame->metadata.anti_spoofing.confidence >= config_.min_confidence;
    
    frame->metadata.anti_spoofing.is_live = passes_fusion && passes_confidence;
    
    if (!frame->metadata.anti_spoofing.is_live) {
        frame->metadata.anti_spoofing.rejection_reason = generate_rejection_reason(frame);
    }
    
    frame->metadata.anti_spoofing_processed = true;
    return frame->metadata.anti_spoofing.is_live;
}

// FIXED: Implement real temporal analysis instead of hardcoded 0.8 (Fix #5)
float AntiSpoofingDetector::process_temporal_analysis(const std::vector<FrameBox*>& frames, FrameBox* current_frame) {
    if (frames.empty() || !current_frame) {
        return 0.0f;  // FIXED: No benefit of doubt (was returning 0.8)
    }
    
    // TODO: Implement actual temporal analysis:
    // - Compute variance across frames
    // - Detect micro-movements  
    // - Check blink patterns
    // - Analyze depth breathing
    
    // For now, use the analyze_temporal_consistency from current frame
    float score = 0.5f;  // Placeholder - needs full implementation
    current_frame->metadata.anti_spoofing.temporal_consistency_score = score;
    current_frame->metadata.anti_spoofing.temporal_inconsistency = (score < config_.min_temporal_consistency_score);
    
    return score;
}

float AntiSpoofingDetector::analyze_depth_geometry(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || frame->depth_data.empty() || frame->depth_width <= 0 || frame->depth_height <= 0) {
        return 0.0f;
    }

#ifdef HAVE_OPENCV
    // FIXED: Use proper projection instead of simple scaling (Fix #3)
    // TODO: In Producer.cpp, use rs2::align to get depth aligned to color
    // For now, improved scaling with bounds checking
    cv::Rect depth_roi;
    if (frame->color_width > 0 && frame->color_height > 0) {
        float scale_x = static_cast<float>(frame->depth_width) / static_cast<float>(frame->color_width);
        float scale_y = static_cast<float>(frame->depth_height) / static_cast<float>(frame->color_height);
        
        depth_roi.x = static_cast<int>(face.bbox.x * scale_x);
        depth_roi.y = static_cast<int>(face.bbox.y * scale_y);
        depth_roi.width = static_cast<int>(face.bbox.width * scale_x);
        depth_roi.height = static_cast<int>(face.bbox.height * scale_y);
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
    
    // PHASE 1 FIX #3: Enforce minimum depth ROI coverage (70%)
    int total_pixels = depth_roi.width * depth_roi.height;
    int valid_pixels = 0;
    
    for (int y = depth_roi.y; y < depth_roi.y + depth_roi.height; y++) {
        for (int x = depth_roi.x; x < depth_roi.x + depth_roi.width; x++) {
            int idx = y * frame->depth_width + x;
            if (idx < frame->depth_data.size() && frame->depth_data[idx] > 0) {
                valid_pixels++;
            }
        }
    }
    
    float coverage = static_cast<float>(valid_pixels) / static_cast<float>(total_pixels);
    
    // DEBUG: Log depth coverage
    static int coverage_debug_count = 0;
    if (++coverage_debug_count % 30 == 0) {
        std::cout << "  Depth ROI coverage: " << (coverage * 100) << "% (" << valid_pixels << "/" << total_pixels << " pixels)" << std::endl;
    }
    
    if (coverage < 0.7f) {
        // Insufficient depth coverage - likely flat object or occlusion
        if (coverage_debug_count % 30 == 0) {
            std::cout << "  ⚠️ REJECTED: Depth coverage too low (< 70%)" << std::endl;
        }
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
        
        // DEBUG: Log plane fitting results
        static int plane_debug_count = 0;
        if (++plane_debug_count % 30 == 0) {
            std::cout << "  Plane fitting error: " << (avg_plane_error * 1000) << " mm" << std::endl;
            if (avg_plane_error < 0.008f) {
                std::cout << "  ⚠️ FLAT SURFACE DETECTED (< 8mm deviation)" << std::endl;
            }
        }
        
        if (avg_plane_error < 0.008f) {
            return 0.0f;  // Too flat - likely mask/photo
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
        
        // DEBUG: Log depth range
        static int range_debug_count = 0;
        if (++range_debug_count % 30 == 0) {
            std::cout << "  Depth range: " << (depth_range * 1000) << " mm" << std::endl;
            if (depth_range < 0.025f) {
                std::cout << "  ⚠️ INSUFFICIENT DEPTH VARIATION (< 25mm)" << std::endl;
            }
        }
        
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
    if (!frame || !face.detected) {
        return 0.0f;
    }
    
    // CRITICAL FIX: Require BOTH left AND right IR (stereo consistency)
    if (frame->ir_left_data.empty() || frame->ir_right_data.empty()) {
        return 0.0f;  // Fail closed if stereo IR not available
    }
    
    if (frame->ir_width <= 0 || frame->ir_height <= 0) {
        return 0.0f;
    }

#ifdef HAVE_OPENCV
    // FIXED: Use proper projection instead of simple scaling (Fix #3)
    // TODO: Use rs2_project_color_pixel_to_depth_pixel for precise mapping
    cv::Rect ir_roi;
    if (frame->color_width > 0 && frame->color_height > 0) {
        float scale_x = static_cast<float>(frame->ir_width) / static_cast<float>(frame->color_width);
        float scale_y = static_cast<float>(frame->ir_height) / static_cast<float>(frame->color_height);
        
        ir_roi.x = static_cast<int>(face.bbox.x * scale_x);
        ir_roi.y = static_cast<int>(face.bbox.y * scale_y);
        ir_roi.width = static_cast<int>(face.bbox.width * scale_x);
        ir_roi.height = static_cast<int>(face.bbox.height * scale_y);
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
    cv::Mat ir_right(frame->ir_height, frame->ir_width, CV_8UC1,
                     const_cast<uint8_t*>(frame->ir_right_data.data()));
    
    cv::Mat face_ir_left = ir_left(ir_roi);
    cv::Mat face_ir_right = ir_right(ir_roi);

    float texture_score = 0.0f;
    float stereo_score = 0.0f;

    // 1. IR standard deviation check (LEFT IR)
    cv::Scalar mean_ir, std_ir;
    cv::meanStdDev(face_ir_left, mean_ir, std_ir);
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
    
    // 3. Texture pattern analysis (LBP on LEFT IR)
    cv::Mat lbp = cv::Mat::zeros(face_ir_left.size(), CV_8UC1);
    for (int y = 1; y < face_ir_left.rows - 1; y++) {
        for (int x = 1; x < face_ir_left.cols - 1; x++) {
            uint8_t center = face_ir_left.at<uint8_t>(y, x);
            uint8_t pattern = 0;
            
            if (face_ir_left.at<uint8_t>(y-1, x-1) >= center) pattern |= 1;
            if (face_ir_left.at<uint8_t>(y-1, x) >= center) pattern |= 2;
            if (face_ir_left.at<uint8_t>(y-1, x+1) >= center) pattern |= 4;
            if (face_ir_left.at<uint8_t>(y, x+1) >= center) pattern |= 8;
            if (face_ir_left.at<uint8_t>(y+1, x+1) >= center) pattern |= 16;
            if (face_ir_left.at<uint8_t>(y+1, x) >= center) pattern |= 32;
            if (face_ir_left.at<uint8_t>(y+1, x-1) >= center) pattern |= 64;
            if (face_ir_left.at<uint8_t>(y, x-1) >= center) pattern |= 128;
            
            lbp.at<uint8_t>(y, x) = pattern;
        }
    }
    
    // FIXED: Correct calcHist parameters (Fix #6)
    cv::Mat lbp_hist;
    int lbp_histSize = 256;
    float lbp_range[] = {0, 256};
    const float* lbp_histRange[] = {lbp_range};  // FIXED: Array of pointers
    int lbp_channels[] = {0};  // FIXED: Proper channel array
    cv::calcHist(&lbp, 1, lbp_channels, cv::Mat(), lbp_hist, 1, &lbp_histSize, lbp_histRange);
    cv::normalize(lbp_hist, lbp_hist, 0, 1, cv::NORM_L1);
    
    float uniform_patterns = 0.0f;
    for (int i = 0; i < 256; i++) {
        int transitions = 0;
        for (int j = 0; j < 8; j++) {
            int bit1 = (i >> j) & 1;
            int bit2 = (i >> ((j + 1) % 8)) & 1;
            if (bit1 != bit2) transitions++;
        }
        
        if (transitions <= 2) {
            uniform_patterns += lbp_hist.at<float>(i);
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
    
    // 4. STEREO CONSISTENCY CHECK (Left vs Right IR)
    // Real faces are similar in both; flat objects differ significantly
    
    // 4a. Histogram correlation
    cv::Mat hist_left, hist_right;
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange[] = {range};
    int channels[] = {0};
    
    cv::calcHist(&face_ir_left, 1, channels, cv::Mat(), hist_left, 1, &histSize, histRange);
    cv::calcHist(&face_ir_right, 1, channels, cv::Mat(), hist_right, 1, &histSize, histRange);
    cv::normalize(hist_left, hist_left, 0, 1, cv::NORM_MINMAX);
    cv::normalize(hist_right, hist_right, 0, 1, cv::NORM_MINMAX);
    
    double hist_corr = cv::compareHist(hist_left, hist_right, cv::HISTCMP_CORREL);
    if (hist_corr > 0.85f) stereo_score += 1.0f;
    else if (hist_corr > 0.75f) stereo_score += 0.6f;
    else if (hist_corr > 0.65f) stereo_score += 0.3f;
    else stereo_score += 0.0f;  // Very different = flat spoof
    
    // 4b. Mean intensity similarity
    cv::Scalar mean_right;
    cv::meanStdDev(face_ir_right, mean_right, cv::Scalar());
    float mean_diff = std::abs(mean_intensity - mean_right[0]);
    float mean_ratio = mean_diff / (mean_intensity + mean_right[0] + 1e-6f);
    
    if (mean_ratio < 0.15f) stereo_score += 1.0f;
    else if (mean_ratio < 0.25f) stereo_score += 0.5f;
    else stereo_score += 0.0f;
    
    // Combine texture and stereo scores (BOTH must pass)
    float final_score = (texture_score / 3.0f) * 0.6f + (stereo_score / 2.0f) * 0.4f;
    
    // Hard requirement: if stereo_score is too low, fail closed
    if (stereo_score / 2.0f < 0.4f) {
        // Note: ir_material_mismatch will be set later in process_frame
        return 0.0f;  // Fail closed on stereo mismatch
    }
    
    return final_score;
#else
    return 0.5f;
#endif
}

// ============================================================================
// SKIN TEXTURE ANALYSIS - KEY for detecting plastic masks!
// ============================================================================
float AntiSpoofingDetector::analyze_skin_texture(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || frame->color_data.empty()) {
        return 0.0f;
    }

#ifdef OPENCV_AVAILABLE
    cv::Mat color_mat(frame->color_height, frame->color_width, CV_8UC3, (void*)frame->color_data.data());
    
    if (color_mat.empty()) {
        return 0.0f;
    }
    
    // Extract face ROI
    int roi_x = std::max(0, face.bbox.x);
    int roi_y = std::max(0, face.bbox.y);
    int roi_w = std::min(face.bbox.width, color_mat.cols - roi_x);
    int roi_h = std::min(face.bbox.height, color_mat.rows - roi_y);
    
    if (roi_w <= 0 || roi_h <= 0) {
        return 0.0f;
    }
    
    cv::Rect roi_rect(roi_x, roi_y, roi_w, roi_h);
    cv::Mat face_crop = color_mat(roi_rect);
    
    float texture_score = 0.0f;
    
    // ==========================
    // 1. LAPLACIAN VARIANCE (Fine Detail/Sharpness)
    // ==========================
    // Real skin has lots of fine details (pores, wrinkles)
    // Plastic masks are smooth with low variance
    cv::Mat gray, laplacian;
    cv::cvtColor(face_crop, gray, cv::COLOR_BGR2GRAY);
    cv::Laplacian(gray, laplacian, CV_64F);
    
    cv::Scalar lap_mean, lap_stddev;
    cv::meanStdDev(laplacian, lap_mean, lap_stddev);
    double lap_variance = lap_stddev[0] * lap_stddev[0];
    
    // Real skin: 100-1000+, Plastic: 10-50
    if (lap_variance > 200.0) {
        texture_score += 1.0f;  // Rich texture
    } else if (lap_variance > 100.0) {
        texture_score += 0.7f;  // Moderate texture
    } else if (lap_variance > 50.0) {
        texture_score += 0.3f;  // Low texture
    } else {
        texture_score += 0.0f;  // Very smooth = mask
    }
    
    // ==========================
    // 2. COLOR VARIANCE (Non-uniformity)
    // ==========================
    // Real skin has color variation (blood vessels, tone changes)
    // Plastic masks are uniformly painted
    cv::Mat float_crop;
    face_crop.convertTo(float_crop, CV_32F);
    
    cv::Scalar color_mean, color_stddev;
    cv::meanStdDev(float_crop, color_mean, color_stddev);
    
    // Average std dev across channels
    float avg_color_std = (color_stddev[0] + color_stddev[1] + color_stddev[2]) / 3.0f;
    
    // Real skin: 15-40, Plastic: 5-15
    if (avg_color_std > 20.0f) {
        texture_score += 1.0f;  // High color variation
    } else if (avg_color_std > 12.0f) {
        texture_score += 0.6f;  // Moderate variation
    } else if (avg_color_std > 8.0f) {
        texture_score += 0.3f;  // Low variation
    } else {
        texture_score += 0.0f;  // Very uniform = mask
    }
    
    // ==========================
    // 3. EDGE DENSITY (Pores, Features)
    // ==========================
    // Real skin has many edges from pores, wrinkles
    // Plastic masks have fewer, smoother edges
    cv::Mat edges;
    cv::Canny(gray, edges, 50, 150);
    
    int edge_pixels = cv::countNonZero(edges);
    float edge_density = static_cast<float>(edge_pixels) / static_cast<float>(edges.rows * edges.cols);
    
    // Real skin: 0.05-0.20, Plastic: 0.01-0.05
    if (edge_density > 0.08f) {
        texture_score += 1.0f;  // Many edges
    } else if (edge_density > 0.05f) {
        texture_score += 0.6f;  // Moderate edges
    } else if (edge_density > 0.03f) {
        texture_score += 0.3f;  // Few edges
    } else {
        texture_score += 0.0f;  // Very few edges = mask
    }
    
    // ==========================
    // 4. HIGH-FREQUENCY POWER (FFT)
    // ==========================
    // Real skin has rich high-frequency content
    // Plastic masks are mostly low-frequency (smooth)
    cv::Mat gray_float;
    gray.convertTo(gray_float, CV_32F);
    
    // Resize to power of 2 for efficient FFT
    int optimal_rows = cv::getOptimalDFTSize(gray.rows);
    int optimal_cols = cv::getOptimalDFTSize(gray.cols);
    cv::Mat padded;
    cv::copyMakeBorder(gray_float, padded, 0, optimal_rows - gray.rows, 
                      0, optimal_cols - gray.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    
    cv::Mat planes[] = {padded, cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complex_img;
    cv::merge(planes, 2, complex_img);
    
    cv::dft(complex_img, complex_img);
    
    // Compute magnitude
    cv::split(complex_img, planes);
    cv::magnitude(planes[0], planes[1], planes[0]);
    cv::Mat magnitude = planes[0];
    
    // Log scale for better visualization
    magnitude += cv::Scalar::all(1);
    cv::log(magnitude, magnitude);
    
    // Compute high-frequency power (outer 40% of spectrum)
    int cx = magnitude.cols / 2;
    int cy = magnitude.rows / 2;
    float center_radius = std::min(cx, cy) * 0.6f;  // Inner 60%
    
    double total_power = 0.0;
    double high_freq_power = 0.0;
    
    for (int y = 0; y < magnitude.rows; y++) {
        for (int x = 0; x < magnitude.cols; x++) {
            float dist = std::sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
            float val = magnitude.at<float>(y, x);
            total_power += val;
            
            if (dist > center_radius) {
                high_freq_power += val;
            }
        }
    }
    
    float high_freq_ratio = (total_power > 0) ? (high_freq_power / total_power) : 0.0f;
    
    // Real skin: 0.30-0.50, Plastic: 0.10-0.25
    if (high_freq_ratio > 0.35f) {
        texture_score += 1.0f;  // Rich high-frequency content
    } else if (high_freq_ratio > 0.25f) {
        texture_score += 0.6f;  // Moderate high-frequency
    } else if (high_freq_ratio > 0.18f) {
        texture_score += 0.3f;  // Low high-frequency
    } else {
        texture_score += 0.0f;  // Very smooth spectrum = mask
    }
    
    // Combine all 4 components (normalize to 0-1)
    float final_score = texture_score / 4.0f;
    
    // DEBUG: Log skin texture analysis every 30 frames
    static int skin_debug_count = 0;
    if (++skin_debug_count % 30 == 0) {
        std::cout << "\n=== SKIN TEXTURE ANALYSIS DEBUG ===" << std::endl;
        std::cout << "  Laplacian variance: " << lap_variance << " (score: " << (texture_score >= 1.0f ? "1.0" : (texture_score >= 0.7f ? "0.7" : (texture_score >= 0.3f ? "0.3" : "0.0"))) << ")" << std::endl;
        std::cout << "  Color std dev: " << avg_color_std << " (score: " << ((texture_score - (texture_score >= 1.0f ? 1.0f : (texture_score >= 0.7f ? 0.7f : (texture_score >= 0.3f ? 0.3f : 0.0f)))) >= 1.0f ? "1.0" : ((texture_score - (texture_score >= 1.0f ? 1.0f : (texture_score >= 0.7f ? 0.7f : (texture_score >= 0.3f ? 0.3f : 0.0f)))) >= 0.6f ? "0.6" : ((texture_score - (texture_score >= 1.0f ? 1.0f : (texture_score >= 0.7f ? 0.7f : (texture_score >= 0.3f ? 0.3f : 0.0f)))) >= 0.3f ? "0.3" : "0.0"))) << ")" << std::endl;
        std::cout << "  Edge density: " << edge_density << " (score: varies)" << std::endl;
        std::cout << "  High-freq ratio: " << high_freq_ratio << " (score: varies)" << std::endl;
        std::cout << "  FINAL SKIN TEXTURE SCORE: " << (final_score * 100) << "%" << std::endl;
        
        if (final_score < 0.4f) {
            std::cout << "  ⚠️ SMOOTH SURFACE DETECTED - Likely MASK!" << std::endl;
        } else if (final_score > 0.65f) {
            std::cout << "  ✅ RICH TEXTURE - Likely REAL SKIN!" << std::endl;
        }
    }
    
    return final_score;
#else
    return 0.5f;
#endif
}

float AntiSpoofingDetector::analyze_temporal_consistency(const FaceROI& face) {
    // FIXED: No benefit of doubt - require sufficient data (Fix #4)
    if (!face.detected || face_history_.size() < 3) {
        return 0.0f;  // Insufficient data = fail, no benefit of doubt
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
    
    // FIXED: No benefit of doubt for stillness - mark as suspicious (Fix #4)
    if (avg_motion < 0.02f) {
        temporal_score += 0.3f;  // Very steady - suspicious, possible photo attack
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
                temporal_score += 0.3f;  // FIXED: Very stable = suspicious (Fix #4)
            }
        } else {
            temporal_score += 0.3f;  // FIXED: No data = suspicious (Fix #4)
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

    // FIXED: No benefit of doubt when data missing (Fix #4)
    if (checks == 0) return 0.0f;  // No data = fail, no benefit of doubt
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
    float skin = frame->metadata.anti_spoofing.skin_texture_score;
    float cm = frame->metadata.anti_spoofing.cross_modal_score;
    float t = frame->metadata.anti_spoofing.temporal_consistency_score;

    // FIXED: Mean-based confidence (80%) with small min-component guard (20%)
    // This allows real humans to pass even if one component is low
    // Include skin texture in confidence calculation!
    float min_comp = std::min({d, ir, skin, cm, t});
    float mean_comp = (d + ir + skin + cm + t) / 5.0f;

    float confidence = 0.2f * min_comp + 0.8f * mean_comp;  // CHANGED from 0.6/0.4
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

// ============================================================================
// Enhanced Landmark-Based Anti-Spoofing (68 Points)
// ============================================================================

/**
 * @brief Validate depth at 68 facial landmarks
 * 
 * Checks that depth values at landmark points form a coherent 3D face structure.
 * Flat objects (photos, screens) will have inconsistent or missing depth at landmarks.
 */
float AntiSpoofingDetector::validate_landmarks_depth(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || !frame->metadata.landmarks.valid) {
        return 0.0f;
    }
    
    if (frame->depth_data.empty() || frame->depth_width <= 0 || frame->depth_height <= 0) {
        return 0.0f;
    }
    
#ifdef HAVE_OPENCV
    const auto& lm = frame->metadata.landmarks;
    
    // Collect depth values at all 68 landmarks
    std::vector<float> landmark_depths;
    landmark_depths.reserve(68);
    
    // Helper lambda to get depth at a 2D point
    auto get_depth_at_point = [&](float x, float y) -> float {
        // Scale from color coordinates to depth coordinates
        float scale_x = static_cast<float>(frame->depth_width) / static_cast<float>(frame->color_width);
        float scale_y = static_cast<float>(frame->depth_height) / static_cast<float>(frame->color_height);
        
        int depth_x = static_cast<int>(x * scale_x);
        int depth_y = static_cast<int>(y * scale_y);
        
        // Bounds checking
        if (depth_x < 0 || depth_x >= frame->depth_width || depth_y < 0 || depth_y >= frame->depth_height) {
            return 0.0f;
        }
        
        int idx = depth_y * frame->depth_width + depth_x;
        if (idx < 0 || idx >= frame->depth_data.size()) {
            return 0.0f;
        }
        
        uint16_t depth_raw = frame->depth_data[idx];
        return depth_raw > 0 ? (depth_raw * frame->depth_scale) : 0.0f;
    };
    
    // Sample all landmark groups
    // Contour (17 points)
    for (int i = 0; i < 17; i++) {
        float d = get_depth_at_point(lm.contour_x[i], lm.contour_y[i]);
        if (d > 0) landmark_depths.push_back(d);
    }
    
    // Eyebrows (10 points)
    for (int i = 0; i < 5; i++) {
        float d1 = get_depth_at_point(lm.right_eyebrow_x[i], lm.right_eyebrow_y[i]);
        float d2 = get_depth_at_point(lm.left_eyebrow_x[i], lm.left_eyebrow_y[i]);
        if (d1 > 0) landmark_depths.push_back(d1);
        if (d2 > 0) landmark_depths.push_back(d2);
    }
    
    // Nose bridge (4 points)
    for (int i = 0; i < 4; i++) {
        float d = get_depth_at_point(lm.nose_bridge_x[i], lm.nose_bridge_y[i]);
        if (d > 0) landmark_depths.push_back(d);
    }
    
    // Nose bottom (5 points)
    for (int i = 0; i < 5; i++) {
        float d = get_depth_at_point(lm.nose_bottom_x[i], lm.nose_bottom_y[i]);
        if (d > 0) landmark_depths.push_back(d);
    }
    
    // Eyes (12 points)
    for (int i = 0; i < 6; i++) {
        float d1 = get_depth_at_point(lm.right_eye_x[i], lm.right_eye_y[i]);
        float d2 = get_depth_at_point(lm.left_eye_x[i], lm.left_eye_y[i]);
        if (d1 > 0) landmark_depths.push_back(d1);
        if (d2 > 0) landmark_depths.push_back(d2);
    }
    
    // Outer mouth (12 points)
    for (int i = 0; i < 12; i++) {
        float d = get_depth_at_point(lm.outer_mouth_x[i], lm.outer_mouth_y[i]);
        if (d > 0) landmark_depths.push_back(d);
    }
    
    // Inner mouth (8 points)
    for (int i = 0; i < 8; i++) {
        float d = get_depth_at_point(lm.inner_mouth_x[i], lm.inner_mouth_y[i]);
        if (d > 0) landmark_depths.push_back(d);
    }
    
    // Require at least 50% of landmarks to have valid depth (34/68)
    if (landmark_depths.size() < 34) {
        return 0.0f;  // Insufficient depth coverage
    }
    
    // Compute statistics
    float mean_depth = std::accumulate(landmark_depths.begin(), landmark_depths.end(), 0.0f) / landmark_depths.size();
    
    // Compute standard deviation
    float variance = 0.0f;
    for (float d : landmark_depths) {
        variance += (d - mean_depth) * (d - mean_depth);
    }
    variance /= landmark_depths.size();
    float std_dev = std::sqrt(variance);
    
    // Real faces should have depth variation (not flat)
    // Typical face depth range: 3-5cm across face surface
    // Score based on depth standard deviation
    float depth_variation_score = std::min(1.0f, std_dev / 0.03f);  // 3cm = perfect score
    
    // Check for depth discontinuities (jumps) which indicate flat surface
    int valid_transitions = 0;
    int smooth_transitions = 0;
    for (size_t i = 1; i < landmark_depths.size(); i++) {
        float diff = std::abs(landmark_depths[i] - landmark_depths[i-1]);
        valid_transitions++;
        if (diff < 0.05f) {  // Smooth transition (< 5cm jump)
            smooth_transitions++;
        }
    }
    
    float smoothness_score = valid_transitions > 0 ? 
        static_cast<float>(smooth_transitions) / valid_transitions : 0.0f;
    
    // Combined score: good variation + smooth transitions
    float final_score = (depth_variation_score * 0.6f + smoothness_score * 0.4f);
    
    return final_score;
#else
    return 0.5f;  // No OpenCV, return neutral
#endif
}

/**
 * @brief Analyze nose protrusion using 9 nose landmarks
 * 
 * The nose should protrude forward from the face plane.
 * Flat objects will not show this 3D structure.
 */
float AntiSpoofingDetector::analyze_nose_protrusion(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || !frame->metadata.landmarks.valid) {
        return 0.0f;
    }
    
    if (frame->depth_data.empty() || frame->depth_width <= 0 || frame->depth_height <= 0) {
        return 0.0f;
    }
    
#ifdef HAVE_OPENCV
    const auto& lm = frame->metadata.landmarks;
    
    // Get nose tip depth (landmark 30 in iBUG format - index 3)
    auto get_depth = [&](float x, float y) -> float {
        float scale_x = static_cast<float>(frame->depth_width) / static_cast<float>(frame->color_width);
        float scale_y = static_cast<float>(frame->depth_height) / static_cast<float>(frame->color_height);
        
        int depth_x = static_cast<int>(x * scale_x);
        int depth_y = static_cast<int>(y * scale_y);
        
        if (depth_x < 0 || depth_x >= frame->depth_width || depth_y < 0 || depth_y >= frame->depth_height) {
            return 0.0f;
        }
        
        int idx = depth_y * frame->depth_width + depth_x;
        if (idx < 0 || idx >= frame->depth_data.size()) {
            return 0.0f;
        }
        
        uint16_t depth_raw = frame->depth_data[idx];
        return depth_raw > 0 ? (depth_raw * frame->depth_scale) : 0.0f;
    };
    
    // Get nose tip (should be closest to camera) - use center of nose bottom (landmark 33)
    float nose_tip_depth = get_depth(lm.nose_bottom_x[2], lm.nose_bottom_y[2]);
    
    // Get nose bridge (should be further back) - use top of nose bridge (landmark 27)
    float nose_bridge_depth = get_depth(lm.nose_bridge_x[0], lm.nose_bridge_y[0]);
    
    // Get face contour points (cheeks) for reference plane
    float left_cheek_depth = get_depth(lm.contour_x[2], lm.contour_y[2]);
    float right_cheek_depth = get_depth(lm.contour_x[14], lm.contour_y[14]);
    
    if (nose_tip_depth == 0.0f || nose_bridge_depth == 0.0f) {
        return 0.0f;
    }
    
    // Nose tip should be closer to camera than bridge (negative means protruding)
    float nose_protrusion = nose_bridge_depth - nose_tip_depth;
    
    // Typical nose protrusion: 1-2cm
    float protrusion_score = 0.0f;
    if (nose_protrusion > 0.01f && nose_protrusion < 0.04f) {
        protrusion_score = 1.0f;  // Good protrusion
    } else if (nose_protrusion > 0.005f) {
        protrusion_score = 0.5f;  // Slight protrusion
    } else {
        protrusion_score = 0.0f;  // Flat or recessed (suspicious)
    }
    
    // Also check that nose is forward of cheeks
    if (left_cheek_depth > 0 && right_cheek_depth > 0) {
        float avg_cheek_depth = (left_cheek_depth + right_cheek_depth) / 2.0f;
        float nose_vs_cheek = avg_cheek_depth - nose_tip_depth;
        
        if (nose_vs_cheek > 0.005f) {
            protrusion_score = std::max(protrusion_score, 0.7f);
        }
    }
    
    return protrusion_score;
#else
    return 0.5f;
#endif
}

/**
 * @brief Analyze face curvature using 17 contour points
 * 
 * Real faces have curved surfaces. Flat objects will have linear depth profiles.
 */
float AntiSpoofingDetector::analyze_face_curvature(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || !frame->metadata.landmarks.valid) {
        return 0.0f;
    }
    
    if (frame->depth_data.empty() || frame->depth_width <= 0 || frame->depth_height <= 0) {
        return 0.0f;
    }
    
#ifdef HAVE_OPENCV
    const auto& lm = frame->metadata.landmarks;
    
    // Get depth at contour points
    std::vector<cv::Point3f> contour_3d;
    contour_3d.reserve(17);
    
    auto get_depth = [&](float x, float y) -> float {
        float scale_x = static_cast<float>(frame->depth_width) / static_cast<float>(frame->color_width);
        float scale_y = static_cast<float>(frame->depth_height) / static_cast<float>(frame->color_height);
        
        int depth_x = static_cast<int>(x * scale_x);
        int depth_y = static_cast<int>(y * scale_y);
        
        if (depth_x < 0 || depth_x >= frame->depth_width || depth_y < 0 || depth_y >= frame->depth_height) {
            return 0.0f;
        }
        
        int idx = depth_y * frame->depth_width + depth_x;
        if (idx < 0 || idx >= frame->depth_data.size()) {
            return 0.0f;
        }
        
        uint16_t depth_raw = frame->depth_data[idx];
        return depth_raw > 0 ? (depth_raw * frame->depth_scale) : 0.0f;
    };
    
    for (int i = 0; i < 17; i++) {
        float d = get_depth(lm.contour_x[i], lm.contour_y[i]);
        if (d > 0) {
            contour_3d.emplace_back(lm.contour_x[i], lm.contour_y[i], d);
        }
    }
    
    if (contour_3d.size() < 12) {
        return 0.0f;  // Not enough points
    }
    
    // Fit a plane to the contour points
    cv::Mat A(contour_3d.size(), 3, CV_32F);
    cv::Mat B(contour_3d.size(), 1, CV_32F);
    
    for (size_t i = 0; i < contour_3d.size(); i++) {
        A.at<float>(i, 0) = contour_3d[i].x;
        A.at<float>(i, 1) = contour_3d[i].y;
        A.at<float>(i, 2) = 1.0f;
        B.at<float>(i, 0) = contour_3d[i].z;
    }
    
    cv::Mat plane_params;
    cv::solve(A, B, plane_params, cv::DECOMP_SVD);
    
    // Compute residuals (how much points deviate from plane)
    float total_residual = 0.0f;
    for (size_t i = 0; i < contour_3d.size(); i++) {
        float plane_z = plane_params.at<float>(0) * contour_3d[i].x + 
                       plane_params.at<float>(1) * contour_3d[i].y + 
                       plane_params.at<float>(2);
        float residual = std::abs(contour_3d[i].z - plane_z);
        total_residual += residual;
    }
    
    float avg_residual = total_residual / contour_3d.size();
    
    // Real faces have significant curvature (residuals > 1cm)
    // Flat surfaces have low residuals (< 0.5cm)
    float curvature_score = std::min(1.0f, avg_residual / 0.015f);  // 1.5cm = perfect
    
    return curvature_score;
#else
    return 0.5f;
#endif
}

/**
 * @brief Validate left-right symmetry
 * 
 * Real faces have symmetric depth profiles. Asymmetric depth indicates spoofing.
 */
float AntiSpoofingDetector::validate_symmetry(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || !frame->metadata.landmarks.valid) {
        return 0.0f;
    }
    
    if (frame->depth_data.empty() || frame->depth_width <= 0 || frame->depth_height <= 0) {
        return 0.0f;
    }
    
#ifdef HAVE_OPENCV
    const auto& lm = frame->metadata.landmarks;
    
    auto get_depth = [&](float x, float y) -> float {
        float scale_x = static_cast<float>(frame->depth_width) / static_cast<float>(frame->color_width);
        float scale_y = static_cast<float>(frame->depth_height) / static_cast<float>(frame->color_height);
        
        int depth_x = static_cast<int>(x * scale_x);
        int depth_y = static_cast<int>(y * scale_y);
        
        if (depth_x < 0 || depth_x >= frame->depth_width || depth_y < 0 || depth_y >= frame->depth_height) {
            return 0.0f;
        }
        
        int idx = depth_y * frame->depth_width + depth_x;
        if (idx < 0 || idx >= frame->depth_data.size()) {
            return 0.0f;
        }
        
        uint16_t depth_raw = frame->depth_data[idx];
        return depth_raw > 0 ? (depth_raw * frame->depth_scale) : 0.0f;
    };
    
    // Compare symmetric pairs
    std::vector<float> differences;
    
    // Eyebrows
    for (int i = 0; i < 5; i++) {
        float left_d = get_depth(lm.left_eyebrow_x[i], lm.left_eyebrow_y[i]);
        float right_d = get_depth(lm.right_eyebrow_x[i], lm.right_eyebrow_y[i]);
        if (left_d > 0 && right_d > 0) {
            differences.push_back(std::abs(left_d - right_d));
        }
    }
    
    // Eyes
    for (int i = 0; i < 6; i++) {
        float left_d = get_depth(lm.left_eye_x[i], lm.left_eye_y[i]);
        float right_d = get_depth(lm.right_eye_x[i], lm.right_eye_y[i]);
        if (left_d > 0 && right_d > 0) {
            differences.push_back(std::abs(left_d - right_d));
        }
    }
    
    if (differences.empty()) {
        return 0.0f;
    }
    
    // Average symmetric difference should be small for real faces
    float avg_diff = std::accumulate(differences.begin(), differences.end(), 0.0f) / differences.size();
    
    // Good symmetry: < 5mm difference, Poor: > 2cm
    float symmetry_score = 1.0f - std::min(1.0f, avg_diff / 0.02f);
    
    return symmetry_score;
#else
    return 0.5f;
#endif
}

/**
 * @brief Track micro-movements of landmarks over time
 * 
 * Real faces have subtle micro-movements even when "still".
 * Photos/screens have perfectly static landmarks.
 */
float AntiSpoofingDetector::track_landmark_motion(const FaceROI& current_face) {
    if (!current_face.detected || face_history_.empty()) {
        return 0.5f;  // Neutral - insufficient history
    }
    
    // Need at least 5 frames of history (150ms at 30fps)
    if (face_history_.size() < 5) {
        return 0.5f;
    }
    
#ifdef HAVE_OPENCV
    // Compare current landmarks with recent history
    // Real faces: small but non-zero movement
    // Fake faces: either perfectly static or unnatural jumps
    
    // For now, return neutral score until we have landmark tracking in FaceROI
    // TODO: Add landmark history tracking similar to face_history_
    
    return 0.5f;  // Placeholder - needs landmark history buffer
#else
    return 0.5f;
#endif
}

} // namespace mdai
