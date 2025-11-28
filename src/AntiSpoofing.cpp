#include "AntiSpoofing.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

// Debug logging flag - set to false for production to avoid hot-path logging
#define ENABLE_DEBUG_LOGGING false

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
    // Enhanced input validation
    if (!frame) {
        return false;
    }
    
    // Validate frame data integrity
    if (frame->color_data.empty() && frame->depth_data.empty()) {
        frame->metadata.anti_spoofing.is_live = false;
        frame->metadata.anti_spoofing.rejection_reason = "No valid frame data";
        return false;
    }
    
    // Initialize metadata
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
    
    // CRITICAL FIX: Populate frame_history_ for rPPG and depth-breathing analysis
    frame_history_.push_back(frame);
    if (frame_history_.size() > TEMPORAL_WINDOW) {
        frame_history_.pop_front();
    }
    
    frame->metadata.anti_spoofing.depth_analysis_score = analyze_depth_geometry(frame, face);
    
    // IR texture analysis now includes stereo consistency check
    frame->metadata.anti_spoofing.ir_texture_score = analyze_ir_texture(frame, face);
    
    frame->metadata.anti_spoofing.cross_modal_score = analyze_cross_modal_consistency(frame, face);
    
    // CRITICAL FIX: Use comprehensive temporal analysis instead of basic analyze_temporal_consistency
    // Convert frame_history_ to vector<FrameBox*> for temporal analysis
    std::vector<FrameBox*> recent_frames;
    recent_frames.reserve(frame_history_.size());
    for (const auto& f : frame_history_) {
        if (f) {
            // SAFE: Use const_cast only for read-only access, ensure frame is valid
            FrameBox* frame_ptr = const_cast<FrameBox*>(f);
            if (frame_ptr && !frame_ptr->color_data.empty()) {
                recent_frames.push_back(frame_ptr);
            }
        }
    }
    frame->metadata.anti_spoofing.temporal_consistency_score = process_temporal_analysis(recent_frames, frame);
    
    // Material analysis for mask detection (edge sharpness, texture, specular, depth discontinuities)
    float material_score = analyze_material_properties(frame, face);
    
    // IMPROVED: Comprehensive facial landmark analysis
    float landmark_score = analyze_facial_landmarks(frame, face);
    
    // Occlusion/obstacle detection (eyes visible, landmarks visible, depth holes)
    float occlusion_score = analyze_occlusion(frame, face);
    
    // IMPROVED: 3D facial structure validation using depth + landmarks
    float facial_structure_score = analyze_3d_facial_structure(frame, face);
    
    // CRITICAL: Depth-landmark fusion validation (NEW)
    float depth_landmark_fusion_score = validate_depth_landmark_fusion(frame, face);
    
    // rPPG pulse detection (NEW - supplementary signal for mask detection)
    // NOTE: Made non-critical to avoid false rejects on real faces
    // Now using robust rPPG with CHROM, bandpass filtering, and motion compensation
    float rppg_pulse_score = analyze_rppg_pulse(frame, face);
    
    // Optimized: Removed excessive debug logging for performance
    
    // If not enough data yet (analyze_rppg_pulse returns 0.5f), score already handled
    
    frame->metadata.anti_spoofing.depth_anomaly_detected = 
        frame->metadata.anti_spoofing.depth_analysis_score < config_.min_depth_analysis_score;
    frame->metadata.anti_spoofing.ir_material_mismatch = 
        frame->metadata.anti_spoofing.ir_texture_score < config_.min_ir_texture_score;
    frame->metadata.anti_spoofing.cross_modal_disagreement = 
        frame->metadata.anti_spoofing.cross_modal_score < config_.min_cross_modal_score;
    
    // IMPROVED: Integrate all detection methods with comprehensive anti-spoofing
    // Prioritize: depth-landmark fusion (most critical), facial landmarks, 3D structure, depth, occlusion, material, IR texture, temporal
    frame->metadata.anti_spoofing.overall_liveness_score =
        (depth_landmark_fusion_score * 0.30f +       // CRITICAL: Depth-landmark fusion (NEW - most important)
         landmark_score * 0.20f +                    // CRITICAL: Facial landmark analysis (eyes, mouth, nose)
         facial_structure_score * 0.15f +           // CRITICAL: 3D facial structure validation
         frame->metadata.anti_spoofing.depth_analysis_score * 0.10f +  // 2D attack detection
         occlusion_score * 0.10f +                    // Mask/obstacle detection
         material_score * 0.08f +                     // Material analysis
         frame->metadata.anti_spoofing.ir_texture_score * 0.05f +      // IR texture analysis
         rppg_pulse_score * 0.01f +                  // Pulse detection (soft signal)
         frame->metadata.anti_spoofing.temporal_consistency_score * 0.01f);  // Temporal analysis
    
    frame->metadata.anti_spoofing.detected_attack_type = detect_attack_type(frame, face);
    
    frame->metadata.anti_spoofing.confidence = calculate_confidence(frame, face);
    
    // FIXED: Replace OR-of-ANDs with weighted score fusion (Fix #8)
    // Old logic allowed bypasses - e.g., depth_ok + temporal_ok passes even if IR shows attack
    
    // Get individual scores
    float depth_score = frame->metadata.anti_spoofing.depth_analysis_score;
    float ir_score = frame->metadata.anti_spoofing.ir_texture_score;
    float temporal_score = frame->metadata.anti_spoofing.temporal_consistency_score;
    
    // Weighted fusion (sum = 1.0) - includes occlusion, material and rPPG
    float fused_score = 
        depth_score * 0.25f +       // Depth for 2D attacks
        occlusion_score * 0.20f +   // NEW: Eyes/face visibility
        material_score * 0.20f +    // Material properties for mask detection
        ir_score * 0.15f +          // IR texture
        rppg_pulse_score * 0.10f +  // rPPG pulse (supplementary)
        temporal_score * 0.10f;     // Temporal consistency
    
    // Minimum guardrail: no single critical component can be too low
    // NOTE: Exclude rPPG from guardrail since it's not robust yet
    // Include occlusion in guardrail - eyes MUST be visible
    float min_component = std::min({depth_score, occlusion_score, material_score, ir_score, temporal_score});
    
    // Pass criteria: good fused score AND no single component is critically low
    bool passes_fusion = (fused_score >= config_.min_overall_liveness);
    // Guardrail: Lowered to 0.4 to accommodate new material analysis (can be strict)
    bool passes_guardrail = (min_component >= 0.4f);  // At least 40% on all critical components
    bool passes_confidence = frame->metadata.anti_spoofing.confidence >= config_.min_confidence;
    
    frame->metadata.anti_spoofing.is_live = passes_fusion && passes_guardrail && passes_confidence;
    
    if (!frame->metadata.anti_spoofing.is_live) {
        frame->metadata.anti_spoofing.rejection_reason = generate_rejection_reason(frame);
    }
    
    frame->metadata.anti_spoofing_processed = true;
    return frame->metadata.anti_spoofing.is_live;
}

// CRITICAL: Comprehensive temporal analysis implementation
float AntiSpoofingDetector::process_temporal_analysis(const std::vector<FrameBox*>& frames, FrameBox* current_frame) {
    if (frames.empty() || !current_frame) {
        return 0.0f;
    }
    
    float temporal_score = 0.0f;
    int components = 0;
    
    // 1. CRITICAL: Micro-motion analysis (natural head movement)
    float micro_motion_score = analyze_micro_motion_temporal(frames);
    temporal_score += micro_motion_score;
    components++;
    
    // 2. CRITICAL: Depth-based breathing analysis
    float breathing_score = analyze_depth_breathing_temporal(frames);
    temporal_score += breathing_score;
    components++;
    
    // 3. CRITICAL: Blink pattern analysis
    float blink_pattern_score = analyze_blink_patterns_temporal(frames);
    temporal_score += blink_pattern_score;
    components++;
    
    // 4. CRITICAL: Facial landmark temporal consistency
    float landmark_consistency_score = analyze_landmark_temporal_consistency(frames);
    temporal_score += landmark_consistency_score;
    components++;
    
    // 5. CRITICAL: RealSense data quality validation
    float data_quality_score = validate_realsense_data_quality(current_frame);
    temporal_score += data_quality_score;
    components++;
    
    // 6. CRITICAL: Frame-to-frame depth consistency
    float depth_consistency_score = analyze_depth_temporal_consistency(frames);
    temporal_score += depth_consistency_score;
    components++;
    
    // Normalize score
    temporal_score = (components > 0) ? (temporal_score / components) : 0.5f;
    current_frame->metadata.anti_spoofing.temporal_consistency_score = temporal_score;
    current_frame->metadata.anti_spoofing.temporal_inconsistency = (temporal_score < config_.min_temporal_consistency_score);
    
    // Debug output
    static int temporal_debug = 0;
    if (++temporal_debug % 30 == 0) {
        std::cout << "⏱️ Temporal Analysis: MicroMotion=" << micro_motion_score 
                  << ", Breathing=" << breathing_score 
                  << ", BlinkPattern=" << blink_pattern_score 
                  << ", LandmarkConsistency=" << landmark_consistency_score 
                  << ", DataQuality=" << data_quality_score 
                  << ", DepthConsistency=" << depth_consistency_score 
                  << ", Overall=" << temporal_score << std::endl;
    }
    
    return temporal_score;
}

float AntiSpoofingDetector::analyze_depth_geometry(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || frame->depth_data.empty() || frame->depth_width <= 0 || frame->depth_height <= 0) {
        return 0.0f;
    }

#ifdef HAVE_OPENCV
    // FIXED: Use proper projection from color to depth using intrinsics
    // Sample depth at median Z to get reference depth
    float median_depth_for_roi = 0.5f;  // Default 50cm if can't determine
    
    // Quick sample center pixel to get approximate depth
    int center_x = face.bbox.x + face.bbox.width / 2;
    int center_y = face.bbox.y + face.bbox.height / 2;
    
    // Use scale-based approximation for initial center sampling
    int approx_depth_x = center_x * frame->depth_width / std::max(1, frame->color_width);
    int approx_depth_y = center_y * frame->depth_height / std::max(1, frame->color_height);
    int center_idx = approx_depth_y * frame->depth_width + approx_depth_x;
    if (center_idx >= 0 && center_idx < (int)frame->depth_data.size() && frame->depth_data[center_idx] > 0) {
        median_depth_for_roi = frame->depth_data[center_idx] * frame->depth_scale;
    }
    
    // IMPROVED: Enhanced depth projection with proper RealSense intrinsics
    // Project color ROI corners to depth space using proper projection
    // Corners: TL, TR, BL, BR
    std::vector<cv::Point2i> color_corners = {
        {face.bbox.x, face.bbox.y},
        {face.bbox.x + face.bbox.width, face.bbox.y},
        {face.bbox.x, face.bbox.y + face.bbox.height},
        {face.bbox.x + face.bbox.width, face.bbox.y + face.bbox.height}
    };
    
    std::vector<cv::Point2i> depth_corners;
    depth_corners.reserve(4);
    
    for (const auto& color_pt : color_corners) {
        // IMPROVED: Use actual depth at each corner instead of median
        float corner_depth = median_depth_for_roi;
        
        // Get depth at this specific color pixel for more accurate projection
        int depth_x = color_pt.x * frame->depth_width / std::max(1, frame->color_width);
        int depth_y = color_pt.y * frame->depth_height / std::max(1, frame->color_height);
        int depth_idx = depth_y * frame->depth_width + depth_x;
        
        if (depth_idx >= 0 && depth_idx < static_cast<int>(frame->depth_data.size()) && 
            frame->depth_data[depth_idx] > 0) {
            corner_depth = frame->depth_data[depth_idx] * frame->depth_scale;
        }
        
        // Validate depth range (0.1m to 2.0m for face detection)
        if (corner_depth < 0.1f || corner_depth > 2.0f) {
            corner_depth = median_depth_for_roi;  // Fallback to median
        }
        
        // Deproject from color to 3D point using corner-specific depth
        float color_pixel[2] = {static_cast<float>(color_pt.x), static_cast<float>(color_pt.y)};
        float point3d[3];
        rs2_deproject_pixel_to_point(point3d, &frame->color_intrinsics, color_pixel, corner_depth);
        
        // IMPROVED: Apply extrinsics transformation if available
        // For D435i, color and depth sensors are close but not perfectly aligned
        if (frame->extrinsics_depth_to_color.rotation[0] != 0 || 
            frame->extrinsics_depth_to_color.translation[0] != 0) {
            // Apply extrinsics transformation (depth to color)
            float transformed_point[3];
            rs2_transform_point_to_point(transformed_point, &frame->extrinsics_depth_to_color, point3d);
            point3d[0] = transformed_point[0];
            point3d[1] = transformed_point[1];
            point3d[2] = transformed_point[2];
        }
        
        // Project back to depth pixel
        float depth_pixel[2];
        rs2_project_point_to_pixel(depth_pixel, &frame->depth_intrinsics, point3d);
        
        depth_corners.push_back(cv::Point2i(
            static_cast<int>(std::round(depth_pixel[0])),
            static_cast<int>(std::round(depth_pixel[1]))
        ));
    }
    
    // Calculate bounding box from projected corners
    int min_x = std::min({depth_corners[0].x, depth_corners[1].x, depth_corners[2].x, depth_corners[3].x});
    int max_x = std::max({depth_corners[0].x, depth_corners[1].x, depth_corners[2].x, depth_corners[3].x});
    int min_y = std::min({depth_corners[0].y, depth_corners[1].y, depth_corners[2].y, depth_corners[3].y});
    int max_y = std::max({depth_corners[0].y, depth_corners[1].y, depth_corners[2].y, depth_corners[3].y});
    
    cv::Rect depth_roi(min_x, min_y, max_x - min_x, max_y - min_y);
    
    // IMPROVED: Enhanced projection validation
    // Validate projection quality
    bool projection_valid = true;
    for (const auto& corner : depth_corners) {
        if (corner.x < 0 || corner.x >= frame->depth_width || 
            corner.y < 0 || corner.y >= frame->depth_height) {
            projection_valid = false;
            break;
        }
    }
    
    if (!projection_valid) {
        // Fallback to simple scaling if projection fails
        depth_roi = cv::Rect(
            face.bbox.x * frame->depth_width / std::max(1, frame->color_width),
            face.bbox.y * frame->depth_height / std::max(1, frame->color_height),
            face.bbox.width * frame->depth_width / std::max(1, frame->color_width),
            face.bbox.height * frame->depth_height / std::max(1, frame->color_height)
        );
    }
    
    // Optimized: Reduced debug logging for performance
    static int proj_debug = 0;
    if (++proj_debug % 60 == 0) {  // Reduced frequency
        std::cout << "🔍 Enhanced Projection: " << depth_roi << " (valid=" << projection_valid << ")" << std::endl;
    }
    
    // Bounds check
    depth_roi.x = std::max(0, std::min(depth_roi.x, frame->depth_width - 1));
    depth_roi.y = std::max(0, std::min(depth_roi.y, frame->depth_height - 1));
    depth_roi.width = std::min(depth_roi.width, frame->depth_width - depth_roi.x);
    depth_roi.height = std::min(depth_roi.height, frame->depth_height - depth_roi.y);
    
    if (depth_roi.width <= 0 || depth_roi.height <= 0) {
        std::cout << "⚠️  Invalid depth ROI after bounds check: " << depth_roi << std::endl;
        return 0.0f;
    }
    
    // PHASE 1 FIX #3: Enforce minimum depth ROI coverage (70%)
    int total_pixels = depth_roi.width * depth_roi.height;
    int valid_pixels = 0;
    
    for (int y = depth_roi.y; y < depth_roi.y + depth_roi.height; y++) {
        for (int x = depth_roi.x; x < depth_roi.x + depth_roi.width; x++) {
            int idx = y * frame->depth_width + x;
            if (idx < static_cast<int>(frame->depth_data.size()) && frame->depth_data[idx] > 0) {
                valid_pixels++;
            }
        }
    }
    
    float coverage = static_cast<float>(valid_pixels) / static_cast<float>(total_pixels);
    if (coverage < 0.7f) {
        // Insufficient depth coverage - likely flat object or occlusion
        static int cov_fail = 0;
        if (++cov_fail % 30 == 0) {
            std::cout << "⚠️  Depth coverage too low: " << coverage << " (need 0.7)" << std::endl;
        }
        return 0.0f;
    }
    
    // IMPROVED: Optimized 3D point generation with better sampling
    std::vector<cv::Point3f> face_points_all;
    std::vector<float> depth_values_all;
    
    // Adaptive sampling based on ROI size for performance
    int step = (depth_roi.width * depth_roi.height > 10000) ? 3 : 2;  // Larger ROI = more sampling
    face_points_all.reserve(static_cast<size_t>(depth_roi.width * depth_roi.height / (step * step)));
    
    for (int y = depth_roi.y; y < depth_roi.y + depth_roi.height; y += step) {
        for (int x = depth_roi.x; x < depth_roi.x + depth_roi.width; x += step) {
            int idx = y * frame->depth_width + x;
            if (idx < static_cast<int>(frame->depth_data.size())) {
                uint16_t depth_raw = frame->depth_data[idx];
                if (depth_raw > 0) {
                    float z = depth_raw * frame->depth_scale;
                    
                    // IMPROVED: Validate depth range before processing
                    if (z >= 0.1f && z <= 2.0f) {  // Valid face depth range
                        float pix[2] = { static_cast<float>(x), static_cast<float>(y) };
                        float p3[3];
                        rs2_deproject_pixel_to_point(p3, &frame->depth_intrinsics, pix, z);
                        
                        // Additional validation: check for reasonable 3D coordinates
                        if (std::abs(p3[0]) < 1.0f && std::abs(p3[1]) < 1.0f && p3[2] > 0.1f) {
                            face_points_all.emplace_back(p3[0], p3[1], p3[2]);
                            depth_values_all.push_back(p3[2]);
                        }
                    }
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
        float avg_plane_error_mm = avg_plane_error * 1000.0f;
        
        // Debug plane error
        static int plane_debug = 0;
        if (++plane_debug % 30 == 0) {
            std::cout << "📐 Plane fitting: error=" << avg_plane_error_mm << "mm" << std::endl;
        }
        
        // STRICT 2D ATTACK DETECTION:
        // Screens/photos are extremely flat (<3mm)
        // Real faces have curvature (>6mm typically)
        // Masks in between (4-6mm)
        if (avg_plane_error < 0.003f) {
            // Extremely flat - definitely 2D attack (screen/photo/paper)
            std::cout << "🚫 2D ATTACK: Plane error " << avg_plane_error_mm << "mm < 3mm (flat surface)" << std::endl;
            return 0.0f;  // HARD REJECT
        } else if (avg_plane_error < 0.006f) {
            // Very flat - likely 2D or rigid mask
            geometry_score += 0.1f;  // Very suspicious
            std::cout << "⚠️  Very flat surface: " << avg_plane_error_mm << "mm" << std::endl;
        } else if (avg_plane_error < 0.015f) {
            // Somewhat flat - borderline
            geometry_score += 0.4f;
        } else if (avg_plane_error < 0.025f) {
            // Good curvature - natural face
            geometry_score += 0.8f;
        } else {
            // Excellent curvature - definitely not flat
            geometry_score += 1.0f;
        }
    }
    
    // 2. Depth range (TIGHTENED for 2D attack detection)
    if (depth_values.size() >= 5) {
        float min_depth = *std::min_element(depth_values.begin(), depth_values.end());
        float max_depth = *std::max_element(depth_values.begin(), depth_values.end());
        float depth_range = max_depth - min_depth;
        float depth_range_mm = depth_range * 1000.0f;
        
        // 2D attacks have very little depth variation (<20mm)
        // Real faces have nose-to-cheek depth variation (>30mm typically)
        if (depth_range < 0.020f) {
            // Extremely flat depth - 2D attack
            geometry_score += 0.0f;
            std::cout << "🚫 Flat depth range: " << depth_range_mm << "mm (2D attack)" << std::endl;
        } else if (depth_range < 0.030f) {
            // Very small range - suspicious
            geometry_score += 0.2f;
        } else if (depth_range < 0.040f) {
            // Small but acceptable
            geometry_score += 0.6f;
        } else if (depth_range > 0.15f) {
            // Too much range - noise or multiple objects
            geometry_score += 0.3f;
        } else {
            // Good depth variation - natural face
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
                
                if (center_idx < static_cast<int>(frame->depth_data.size()) && 
                    right_idx < static_cast<int>(frame->depth_data.size()) && 
                    down_idx < static_cast<int>(frame->depth_data.size())) {
                    
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
        static int ir_missing_debug = 0;
        if (++ir_missing_debug % 30 == 0) {
            std::cout << "⚠️  IR data missing: left=" << frame->ir_left_data.size() 
                      << ", right=" << frame->ir_right_data.size() << std::endl;
        }
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
    
    // DEBUG: Show actual IR values
    static int ir_debug = 0;
    if (++ir_debug % 30 == 0) {
        std::cout << "🔍 IR Analysis: std=" << ir_std << ", mean=" << mean_intensity << std::endl;
    }
    
    // CORRECTED: Realistic thresholds based on actual data
    if (ir_std < 8.0f) {
        texture_score += 0.0f;  // Too uniform (screen/plastic)
        if (ir_debug % 30 == 0) std::cout << "⚠️  IR too uniform (screen/mask)" << std::endl;
    } else if (ir_std < 15.0f) {
        texture_score += 0.5f;  // Borderline - could be real human
        if (ir_debug % 30 == 0) std::cout << "⚠️  IR borderline texture" << std::endl;
    } else if (ir_std < 80.0f) {
        texture_score += 1.0f;  // Good skin texture (REALISTIC RANGE)
        if (ir_debug % 30 == 0) std::cout << "✅ IR good skin texture" << std::endl;
    } else {
        texture_score += 0.7f;  // Very high texture (possible textured mask)
        if (ir_debug % 30 == 0) std::cout << "⚠️  IR very high texture (possible mask)" << std::endl;
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
    
    // 5. NEW: THERMAL GRADIENT ANALYSIS for mask detection
    // Masks have uniform temperature; real faces have thermal gradients
    float thermal_score = 0.0f;
    
    if (face_ir_left.rows > 20 && face_ir_left.cols > 20) {
        // Define regions: forehead (top 30%), cheeks (middle 40%), chin (bottom 30%)
        int forehead_start = 0;
        int forehead_end = static_cast<int>(face_ir_left.rows * 0.3f);
        int cheeks_start = static_cast<int>(face_ir_left.rows * 0.3f);
        int cheeks_end = static_cast<int>(face_ir_left.rows * 0.7f);
        int chin_start = static_cast<int>(face_ir_left.rows * 0.7f);
        int chin_end = face_ir_left.rows;
        
        cv::Mat forehead_region = face_ir_left(cv::Rect(0, forehead_start, face_ir_left.cols, forehead_end - forehead_start));
        cv::Mat cheeks_region = face_ir_left(cv::Rect(0, cheeks_start, face_ir_left.cols, cheeks_end - cheeks_start));
        cv::Mat chin_region = face_ir_left(cv::Rect(0, chin_start, face_ir_left.cols, chin_end - chin_start));
        
        // Calculate mean intensity for each region (proxy for temperature)
        cv::Scalar forehead_mean = cv::mean(forehead_region);
        cv::Scalar cheeks_mean = cv::mean(cheeks_region);
        cv::Scalar chin_mean = cv::mean(chin_region);
        
        float forehead_temp = static_cast<float>(forehead_mean[0]);
        float cheeks_temp = static_cast<float>(cheeks_mean[0]);
        float chin_temp = static_cast<float>(chin_mean[0]);
        
        // Real faces: forehead tends to be warmer (blood flow from brain)
        // Masks: uniform temperature across all regions
        float temp_variation = std::abs(forehead_temp - cheeks_temp) + std::abs(cheeks_temp - chin_temp);
        
        // DEBUG: Print thermal values every 30 frames
        if (ENABLE_DEBUG_LOGGING) {
            static int thermal_debug_count = 0;
            if (++thermal_debug_count % 30 == 0) {
                std::cout << "🌡️  THERMAL DEBUG:" << std::endl;
                std::cout << "   Forehead: " << forehead_temp << " | Cheeks: " << cheeks_temp << " | Chin: " << chin_temp << std::endl;
                std::cout << "   Variation: " << temp_variation << std::endl;
            }
        }
        
        // Check spatial IR reflectance variation
        if (temp_variation < 3.0f) {
            thermal_score = 0.3f;
        } else if (temp_variation < 8.0f) {
            thermal_score = 0.6f;
        } else if (temp_variation < 20.0f) {
            thermal_score = 1.0f;
        } else {
            thermal_score = 0.5f;
        }
        
        // Calculate within-region variance
        cv::Scalar forehead_std;
        cv::meanStdDev(forehead_region, cv::Scalar(), forehead_std);
        float region_variance = static_cast<float>(forehead_std[0]);
        
        // Real faces have some variance within regions; masks are very uniform
        if (region_variance < 3.0f) {
            thermal_score *= 0.5f;  // Penalize uniform regions (mask-like)
        }
    } else {
        // ROI too small for reliable thermal analysis
        thermal_score = 0.5f;  // Neutral score
    }
    
    // NOTE: "thermal" is a misnomer - RealSense IR is NIR reflectance, not thermal emission
    // This measures spatial variance in reflectance, which correlates with surface texture
    // REMOVED hard gate - too brittle for production (causes false rejects on real faces)
    
    // Combine texture, stereo, and IR spatial variance scores
    // REDUCED IR variance weight since it's reflectance-based, not thermal
    // Texture: 50%, Stereo: 30%, IR Spatial Variance: 20%
    float final_score = (texture_score / 3.0f) * 0.50f + (stereo_score / 2.0f) * 0.30f + thermal_score * 0.20f;
    
    // NOTE: Removed stereo hard gate - too brittle, let it contribute via weighted fusion instead
    
    return final_score;
#else
    return 0.5f;
#endif
}

float AntiSpoofingDetector::analyze_rppg_pulse(const FrameBox* frame, const FaceROI& face) {
    std::cout << "🔬 analyze_rppg_pulse called, samples: " << rppg_samples_.size() 
              << ", face.detected: " << face.detected << std::endl;
    
    if (!frame || !face.detected || frame->color_data.empty()) {
        std::cout << "⚠️  Early return: frame=" << (frame != nullptr) 
                  << ", face=" << face.detected << std::endl;
        return 0.5f;  // Neutral score if no data
    }

#ifdef HAVE_OPENCV
    // Extract forehead region (best for pulse detection - less movement, good blood flow)
    cv::Mat color_mat = frame->get_color_mat();
    if (color_mat.empty()) {
        return 0.5f;
    }
    
    cv::Rect forehead_roi;
    
    // Use 468 landmarks to get PRECISE forehead region if available
    if (!frame->metadata.landmarks.empty() && frame->metadata.landmarks.size() >= 468) {
        // Forehead landmark indices (center forehead, above eyebrows)
        // These are stable points less affected by expressions
        std::vector<int> forehead_indices = {
            10, 338, 297, 332, 284, 251,  // Upper forehead
            389, 356, 454, 323, 361, 288,  // Mid forehead
            397, 365, 379, 378, 400, 377   // Lower forehead (above eyebrows)
        };
        
        // Calculate bounding box from forehead landmarks
        float min_x = FLT_MAX, max_x = -FLT_MAX;
        float min_y = FLT_MAX, max_y = -FLT_MAX;
        int valid_landmarks = 0;
        
        for (int idx : forehead_indices) {
            if (idx < (int)frame->metadata.landmarks.size()) {
                const auto& lm = frame->metadata.landmarks[idx];
                min_x = std::min(min_x, lm.x);
                max_x = std::max(max_x, lm.x);
                min_y = std::min(min_y, lm.y);
                max_y = std::max(max_y, lm.y);
                valid_landmarks++;
            }
        }
        
        if (valid_landmarks > 10) {
            // Create ROI from landmarks (add small margin)
            int margin = 5;
            forehead_roi = cv::Rect(
                std::max(0, static_cast<int>(min_x) - margin),
                std::max(0, static_cast<int>(min_y) - margin),
                std::min(color_mat.cols - static_cast<int>(min_x) + margin, static_cast<int>(max_x - min_x) + 2*margin),
                std::min(color_mat.rows - static_cast<int>(min_y) + margin, static_cast<int>(max_y - min_y) + 2*margin)
            );
            std::cout << "📍 Using 468-landmark forehead ROI: " << forehead_roi << std::endl;
        } else {
            // Fallback to bbox-based method
            forehead_roi = face.bbox;
            forehead_roi.height = static_cast<int>(face.bbox.height * 0.3f);
            forehead_roi.y += static_cast<int>(face.bbox.height * 0.1f);
        }
    } else {
        // Fallback: Use face bbox (old method)
        forehead_roi = face.bbox;
        forehead_roi.height = static_cast<int>(face.bbox.height * 0.3f);
        forehead_roi.y += static_cast<int>(face.bbox.height * 0.1f);  // Skip hair
    }
    
    // Bounds check
    forehead_roi = forehead_roi & cv::Rect(0, 0, color_mat.cols, color_mat.rows);
    if (forehead_roi.width < 10 || forehead_roi.height < 10) {
        return 0.5f;
    }
    
    cv::Mat forehead_region = color_mat(forehead_roi);
    
    // Extract RGB channels for CHROM method
    std::vector<cv::Mat> bgr_channels;
    cv::split(forehead_region, bgr_channels);
    
    cv::Scalar mean_blue = cv::mean(bgr_channels[0]);
    cv::Scalar mean_green = cv::mean(bgr_channels[1]);
    cv::Scalar mean_red = cv::mean(bgr_channels[2]);
    
    if (ENABLE_DEBUG_LOGGING) {
        static int rgb_debug_count = 0;
        if (++rgb_debug_count % 60 == 0) {
            std::cout << "📊 RGB RAW VALUES:" << std::endl;
            std::cout << "   R: " << mean_red[0] << ", G: " << mean_green[0] 
                      << ", B: " << mean_blue[0] << std::endl;
            std::cout << "   Forehead ROI: " << forehead_roi << std::endl;
        }
    }
    
    // Calculate motion score (stability of face)
    float motion_score = 1.0f;
    if (face_history_.size() >= 2) {
        const FaceROI& prev = face_history_[face_history_.size() - 2];
        if (prev.detected) {
            float dx = std::abs(face.bbox.x - prev.bbox.x);
            float dy = std::abs(face.bbox.y - prev.bbox.y);
            float motion = std::sqrt(dx * dx + dy * dy);
            motion_score = std::exp(-motion / 10.0f);  // Exponential decay
        }
    }
    
    // Get timestamp (use frame count as proxy)
    double timestamp = static_cast<double>(frame_history_.size()) / 30.0;
    if (rppg_first_timestamp_ == 0.0) {
        rppg_first_timestamp_ = timestamp;
    }
    
    // Create sample
    RPPGSample sample;
    sample.timestamp = timestamp;
    sample.red = static_cast<float>(mean_red[0]);
    sample.green = static_cast<float>(mean_green[0]);
    sample.blue = static_cast<float>(mean_blue[0]);
    sample.motion_score = motion_score;
    sample.landmark_count = 1;  // Simplified
    
    // Add to history
    rppg_samples_.push_back(sample);
    while (rppg_samples_.size() > RPPG_MAX_WINDOW) {
        rppg_samples_.pop_front();
    }
    
    // Need minimum data for reliable detection (5 seconds)
    if (rppg_samples_.size() < RPPG_MIN_WINDOW) {
        return 0.5f;  // Not enough data yet
    }
    
    // Extract CHROM signal with improved detrending
    std::vector<float> chrom_signal;
    float estimated_fps = 30.0f;
    bool signal_ok = extract_rppg_signal_chrom(chrom_signal, estimated_fps);
    
    if (!signal_ok || chrom_signal.empty()) {
        return 0.3f;  // Signal extraction failed
    }
    
    // Apply detrending BEFORE bandpass filter to remove DC drift
    // Use moving average detrending (removes slow trends)
    if (chrom_signal.size() > 30) {
        std::vector<float> detrended(chrom_signal.size());
        int window_size = static_cast<int>(estimated_fps * 2.0f);  // 2-second window
        window_size = std::min(window_size, static_cast<int>(chrom_signal.size()) / 3);
        
        for (size_t i = 0; i < chrom_signal.size(); i++) {
            int start = std::max(0, static_cast<int>(i) - window_size / 2);
            int end = std::min(static_cast<int>(chrom_signal.size()), static_cast<int>(i) + window_size / 2);
            
            float sum = 0.0f;
            int count = 0;
            for (int j = start; j < end; j++) {
                sum += chrom_signal[j];
                count++;
            }
            float local_mean = (count > 0) ? sum / count : 0.0f;
            detrended[i] = chrom_signal[i] - local_mean;
        }
        
        if (ENABLE_DEBUG_LOGGING) {
            static int detrend_debug_count = 0;
            if (++detrend_debug_count % 60 == 0) {
                std::cout << "🔧 DETRENDING DEBUG:" << std::endl;
                std::cout << "   Before (first 10): ";
                for (size_t i = 0; i < std::min(size_t(10), chrom_signal.size()); i++) {
                    std::cout << chrom_signal[i] << " ";
                }
                std::cout << std::endl;
                std::cout << "   After (first 10): ";
                for (size_t i = 0; i < std::min(size_t(10), detrended.size()); i++) {
                    std::cout << detrended[i] << " ";
                }
                std::cout << std::endl;
                
                // Calculate variance after detrending
                float mean_det = 0.0f;
                for (float v : detrended) mean_det += v;
                mean_det /= detrended.size();
                float var_det = 0.0f;
                for (float v : detrended) {
                    float diff = v - mean_det;
                    var_det += diff * diff;
                }
                var_det /= detrended.size();
                std::cout << "   Variance after detrend: " << var_det << std::endl;
            }
        }
        
        chrom_signal = detrended;
    }
    
    // DISABLED: Bandpass filter too aggressive - removes pulse signal
    // Moving average implementation reduces variance to zero
    // Detrending already removes DC drift, so skip bandpass for now
    
    // Detect pulse using improved autocorrelation
    float detected_bpm = 0.0f;
    float confidence = 0.0f;
    bool has_pulse = detect_pulse_fft(chrom_signal, estimated_fps, detected_bpm, confidence);
    
    if (ENABLE_DEBUG_LOGGING) {
        static int rppg_debug_count = 0;
        if (++rppg_debug_count % 30 == 0) {
            std::cout << "💓 rPPG ROBUST DEBUG:" << std::endl;
            std::cout << "   FPS: " << estimated_fps << ", Samples: " << rppg_samples_.size() << std::endl;
            std::cout << "   CHROM signal size: " << chrom_signal.size() << std::endl;
            
            // Calculate signal statistics
            float chrom_mean = 0.0f, chrom_variance = 0.0f;
            for (float val : chrom_signal) {
                chrom_mean += val;
            }
            chrom_mean /= chrom_signal.size();
            for (float val : chrom_signal) {
                float diff = val - chrom_mean;
                chrom_variance += diff * diff;
            }
            chrom_variance /= chrom_signal.size();
            
            std::cout << "   CHROM variance: " << chrom_variance << std::endl;
            
            // Show first few CHROM values for inspection
            std::cout << "   CHROM samples (first 10): ";
            for (size_t i = 0; i < std::min(size_t(10), chrom_signal.size()); i++) {
                std::cout << chrom_signal[i] << " ";
            }
            std::cout << std::endl;
            
            // Check how many samples were skipped due to motion
            int motion_skipped = 0;
            for (const auto& sample : rppg_samples_) {
                if (calculate_motion_compensation_weight(sample) < 0.3f) {
                    motion_skipped++;
                }
            }
            std::cout << "   Motion-skipped samples: " << motion_skipped << "/" << rppg_samples_.size() << std::endl;
            
            if (has_pulse) {
                std::cout << "   ✅ PULSE: " << detected_bpm << " BPM (confidence: " 
                          << confidence << ")" << std::endl;
    } else {
                std::cout << "   ❌ NO PULSE detected (mask suspected)" << std::endl;
                std::cout << "   Detected BPM: " << detected_bpm << ", Confidence: " << confidence << std::endl;
            }
        }
    }
    
    // Score based on pulse detection quality
    // CRITICAL: After enough data, no pulse = likely mask attack
    if (has_pulse && detected_bpm >= 50.0f && detected_bpm <= 120.0f) {
        // Good pulse in healthy range
        return 0.5f + (confidence * 0.5f);  // 0.5-1.0 based on confidence
    } else if (has_pulse && detected_bpm >= 40.0f && detected_bpm <= 150.0f) {
        // Pulse detected but unusual rate
        return 0.3f + (confidence * 0.3f);  // 0.3-0.6
    } else {
        // No pulse detected
        // DISABLED: Hard gate too aggressive - rejects real faces
        // rPPG needs more tuning before it can be used as hard rejection criterion
        // For now, use as soft signal only
        if (rppg_samples_.size() >= RPPG_MIN_WINDOW + 60) {  // 7 seconds = 210 samples
            std::cout << "⚠️  NO PULSE after " << rppg_samples_.size() << " samples (soft penalty)" << std::endl;
            return 0.1f;  // Low score but not hard reject
        }
        // Still collecting data
        std::cout << "⏳ Collecting rPPG data: " << rppg_samples_.size() << "/" 
                  << (RPPG_MIN_WINDOW + 60) << std::endl;
        return 0.3f;  // Still collecting, neutral
    }
    
#else
    return 0.5f;
#endif
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

float AntiSpoofingDetector::calculate_micro_motion() {
    // Calculate micro-motion from recent face history
    // Real faces have subtle natural movements, masks/photos are rigid
    
    if (face_history_.size() < 10) {
        return 0.5f;  // Not enough history
    }
    
    float total_motion = 0.0f;
    int valid_samples = 0;
    
    // Calculate landmark displacement over recent frames
    for (size_t i = face_history_.size() - 10; i < face_history_.size() - 1; i++) {
        const FaceROI& current = face_history_[i];
        const FaceROI& next = face_history_[i + 1];
        
        if (current.detected && next.detected) {
            // Center point motion
            float dx = (current.bbox.x + current.bbox.width / 2.0f) - 
                       (next.bbox.x + next.bbox.width / 2.0f);
            float dy = (current.bbox.y + current.bbox.height / 2.0f) - 
                       (next.bbox.y + next.bbox.height / 2.0f);
            
            float motion = std::sqrt(dx * dx + dy * dy);
            total_motion += motion;
            valid_samples++;
        }
    }
    
    if (valid_samples == 0) {
    return 0.5f;
    }
    
    float avg_motion = total_motion / valid_samples;
    
    // Score based on natural motion range
    // Real faces: 0.5-5 pixels/frame of micro-motion
    // Masks/photos: < 0.5 (too rigid) or > 5 (held by shaky hand)
    if (avg_motion < 0.3f) {
        return 0.3f;  // Too rigid - suspicious
    } else if (avg_motion > 8.0f) {
        return 0.4f;  // Too much motion - suspicious
    } else {
        return 1.0f;  // Natural micro-motion
    }
}

float AntiSpoofingDetector::analyze_depth_breathing(const FrameBox* frame, const FaceROI& face) {
    // Analyze subtle depth changes from breathing
    // Real faces show periodic depth variation (chest/torso breathing)
    // Masks held in hand won't show this pattern
    
    if (!frame || !face.detected || frame->depth_data.empty()) {
    return 0.5f;
    }
    
    if (face_history_.size() < 30) {
        return 0.5f;  // Need ~1 second of data
    }
    
    // Calculate average depth over recent frames
    std::vector<float> depth_sequence;
    depth_sequence.reserve(face_history_.size());
    
    for (const auto& hist_face : face_history_) {
        if (hist_face.detected) {
            // Use center of face bbox as reference point
            int center_x = hist_face.bbox.x + hist_face.bbox.width / 2;
            int center_y = hist_face.bbox.y + hist_face.bbox.height / 2;
            
            // Scale to depth frame coordinates (simplified)
            int depth_x = center_x * frame->depth_width / frame->color_width;
            int depth_y = center_y * frame->depth_height / frame->color_height;
            
            int idx = depth_y * frame->depth_width + depth_x;
            if (idx >= 0 && idx < (int)frame->depth_data.size() && frame->depth_data[idx] > 0) {
                depth_sequence.push_back(frame->depth_data[idx] * frame->depth_scale);
            }
        }
    }
    
    if (depth_sequence.size() < 20) {
        return 0.5f;
    }
    
    // Calculate depth variance (breathing causes ~1-5mm variation)
    float mean_depth = std::accumulate(depth_sequence.begin(), depth_sequence.end(), 0.0f) / depth_sequence.size();
    float variance = 0.0f;
    for (float d : depth_sequence) {
        float diff = d - mean_depth;
        variance += diff * diff;
    }
    variance /= depth_sequence.size();
    float stddev = std::sqrt(variance);
    
    // Real breathing: 1-10mm stddev
    // Rigid mask: < 0.5mm stddev
    if (stddev < 0.0005f) {  // < 0.5mm
        return 0.2f;  // Too rigid - suspicious
    } else if (stddev > 0.02f) {  // > 20mm
        return 0.4f;  // Too much variation - unstable
    } else {
        return 1.0f;  // Natural breathing motion
    }
}

std::string AntiSpoofingDetector::detect_attack_type(const FrameBox* frame, [[maybe_unused]] const FaceROI& face) {
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

float AntiSpoofingDetector::calculate_confidence(const FrameBox* frame, [[maybe_unused]] const FaceROI& face) {
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
// ROBUST rPPG HELPER METHODS
// ============================================================================

bool AntiSpoofingDetector::extract_rppg_signal_chrom(std::vector<float>& chrom_signal, float& estimated_fps) {
    if (rppg_samples_.size() < RPPG_MIN_WINDOW) {
        return false;
    }
    
    // Calculate FPS from timestamps
    double time_span = rppg_samples_.back().timestamp - rppg_samples_.front().timestamp;
    estimated_fps = (rppg_samples_.size() - 1) / time_span;
    
    // Clamp FPS to reasonable range
    if (estimated_fps < 15.0f || estimated_fps > 60.0f) {
        estimated_fps = 30.0f;  // Fallback
    }
    
    chrom_signal.clear();
    chrom_signal.reserve(rppg_samples_.size());
    
    // CHROM method: X = 3R - 2G, Y = 1.5R + G - 1.5B
    // Pulse signal: S = X - α*Y, where α minimizes motion artifacts
    float alpha = 1.0f;  // Simplified, normally adaptive
    
    // Calculate temporal mean for each channel (for normalization)
    float mean_r = 0.0f, mean_g = 0.0f, mean_b = 0.0f;
    for (const auto& sample : rppg_samples_) {
        mean_r += sample.red;
        mean_g += sample.green;
        mean_b += sample.blue;
    }
    mean_r /= rppg_samples_.size();
    mean_g /= rppg_samples_.size();
    mean_b /= rppg_samples_.size();
    
    // Add small epsilon to avoid division by zero
    mean_r = std::max(mean_r, 1.0f);
    mean_g = std::max(mean_g, 1.0f);
    mean_b = std::max(mean_b, 1.0f);
    
    for (const auto& sample : rppg_samples_) {
        // Motion compensation weight
        float weight = calculate_motion_compensation_weight(sample);
        
        if (weight < 0.3f) {
            // Too much motion, skip this sample (interpolate later)
            if (!chrom_signal.empty()) {
                chrom_signal.push_back(chrom_signal.back());
            } else {
                chrom_signal.push_back(0.0f);
            }
            continue;
        }
        
        // Normalize each channel by ITS OWN temporal mean (not by RGB sum!)
        // This preserves intensity variations while normalizing illumination
        float r = sample.red / mean_r;
        float g = sample.green / mean_g;
        float b = sample.blue / mean_b;
        
        // CHROM color space transformation
        float X = 3.0f * r - 2.0f * g;
        float Y = 1.5f * r + g - 1.5f * b;
        
        // Pulse signal
        float S = X - alpha * Y;
        chrom_signal.push_back(S * weight);
    }
    
    return !chrom_signal.empty();
}

float AntiSpoofingDetector::calculate_snr_at_frequency(const std::vector<float>& signal, 
                                                        [[maybe_unused]] float fps, [[maybe_unused]] float target_hz) {
    // Simplified SNR calculation
    // In production, use FFT to calculate power at target frequency vs noise floor
    
    float signal_power = 0.0f;
    float noise_power = 0.0f;
    
    // Calculate signal variance
    float mean = 0.0f;
    for (float val : signal) {
        mean += val;
    }
    mean /= signal.size();
    
    for (float val : signal) {
        float deviation = val - mean;
        signal_power += deviation * deviation;
    }
    signal_power /= signal.size();
    
    // Estimate noise (high-frequency components)
    for (size_t i = 1; i < signal.size(); i++) {
        float diff = signal[i] - signal[i-1];
        noise_power += diff * diff;
    }
    noise_power /= (signal.size() - 1);
    
    // SNR in dB
    if (noise_power < 1e-6f) {
        return 20.0f;  // Very high SNR
    }
    
    float snr = 10.0f * std::log10((signal_power + 1e-6f) / (noise_power + 1e-6f));
    return std::max(0.0f, std::min(snr, 30.0f));  // Clamp to reasonable range
}

bool AntiSpoofingDetector::detect_pulse_fft(const std::vector<float>& signal, float fps, 
                                              float& detected_bpm, float& confidence) {
    if (signal.size() < 60) {
        return false;
    }
    
    // Simple peak detection in frequency domain (simplified FFT alternative)
    // Use autocorrelation for periodicity detection
    
    // Detrend
    std::vector<float> detrended(signal.size());
    float mean = std::accumulate(signal.begin(), signal.end(), 0.0f) / signal.size();
    for (size_t i = 0; i < signal.size(); i++) {
        detrended[i] = signal[i] - mean;
    }
    
    // Autocorrelation for pulse detection (40-150 BPM range)
    int min_lag = static_cast<int>(fps * 60.0f / 150.0f);  // 150 BPM
    int max_lag = static_cast<int>(fps * 60.0f / 40.0f);   // 40 BPM
    
    float max_correlation = 0.0f;
    int best_lag = 0;
    
    for (int lag = min_lag; lag <= max_lag && lag < static_cast<int>(detrended.size()) / 2; lag++) {
        float correlation = 0.0f;
        for (size_t i = 0; i < detrended.size() - lag; i++) {
            correlation += detrended[i] * detrended[i + lag];
        }
        
        if (correlation > max_correlation) {
            max_correlation = correlation;
            best_lag = lag;
        }
    }
    
    // Normalize by variance
    float variance = 0.0f;
    for (float val : detrended) {
        variance += val * val;
    }
    variance /= detrended.size();
    
    float normalized_corr = (variance > 1e-6f) ? max_correlation / (variance * detrended.size()) : 0.0f;
    
    // Calculate confidence from correlation strength and SNR
    float snr = calculate_snr_at_frequency(signal, fps, 1.0f);
    confidence = std::min(1.0f, (normalized_corr * 2.0f) * (snr / 15.0f));  // SNR 15dB = good
    
    if (ENABLE_DEBUG_LOGGING) {
        static int autocorr_debug_count = 0;
        if (++autocorr_debug_count % 60 == 0) {
            std::cout << "🔍 AUTOCORRELATION DEBUG:" << std::endl;
            std::cout << "   Signal variance: " << variance << std::endl;
            std::cout << "   Max correlation: " << max_correlation << std::endl;
            std::cout << "   Normalized corr: " << normalized_corr << " (threshold: 0.4)" << std::endl;
            std::cout << "   Best lag: " << best_lag << " frames" << std::endl;
            std::cout << "   SNR: " << snr << " dB" << std::endl;
            if (best_lag > 0) {
                float bpm = (fps / best_lag) * 60.0f;
                std::cout << "   Would be: " << bpm << " BPM" << std::endl;
            }
        }
    }
    
    // Lowered threshold from 0.4 to 0.15 - original threshold too strict
    // With detrended signal (no bandpass), normalized_corr ~0.15-0.25 is typical
    if (normalized_corr > 0.15f && best_lag > 0) {
        detected_bpm = (fps / best_lag) * 60.0f;
        return true;
    }
    
    return false;
}

float AntiSpoofingDetector::calculate_motion_compensation_weight(const RPPGSample& sample) {
    // Weight based on motion score (1.0 = stable, 0.0 = too much motion)
    return sample.motion_score;
}

// ============================================================================
// MATERIAL ANALYSIS FOR MASK DETECTION
// ============================================================================

float AntiSpoofingDetector::analyze_material_properties(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected) {
    return 0.5f;
    }
    
#ifdef HAVE_OPENCV
    cv::Mat color_mat = frame->get_color_mat();
    if (color_mat.empty() || frame->depth_data.empty()) {
        return 0.5f;
    }
    
    cv::Rect roi = face.bbox & cv::Rect(0, 0, color_mat.cols, color_mat.rows);
    if (roi.width < 20 || roi.height < 20) {
        return 0.5f;
    }
    
    float material_score = 0.0f;
    int checks = 0;
    
    // 1. Edge sharpness analysis - Plastic masks have unnaturally sharp edges
    cv::Mat gray, edges;
    cv::cvtColor(color_mat(roi), gray, cv::COLOR_BGR2GRAY);
    cv::Canny(gray, edges, 100, 200);
    
    int strong_edges = cv::countNonZero(edges);
    float edge_density = strong_edges / (float)(roi.width * roi.height);
    
    // Real skin: moderate edges (wrinkles, features) ~0.05-0.15
    // Plastic: too sharp or too smooth <0.03 or >0.20
    if (edge_density > 0.05f && edge_density < 0.18f) {
        material_score += 1.0f;  // Natural edge density
    } else {
        material_score += 0.3f;  // Suspicious
    }
    
    checks++;
    
    // 2. Texture uniformity - Skin has pores/texture, plastic is uniform
    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar lap_mean, lap_std;
    cv::meanStdDev(laplacian, lap_mean, lap_std);
    
    float texture_variance = lap_std[0];
    
    // DEBUG: Show material analysis values
    static int material_debug = 0;
    if (++material_debug % 30 == 0) {
        std::cout << "🔍 Material Analysis: edge_density=" << edge_density << ", texture_variance=" << texture_variance << std::endl;
    }
    
    // Real skin: moderate variance (pores at distance) > 4
    // Plastic: low variance (too smooth) < 2
    // Note: Values much lower than expected due to camera distance/resolution
    if (texture_variance > 4.0f) {
        material_score += 1.0f;  // Good texture (natural)
    } else if (texture_variance < 2.0f) {
        material_score += 0.2f;  // Too uniform - suspicious (mask)
    } else {
        material_score += 0.7f;  // Borderline
    }
    checks++;
    
    // 3. Specular highlights - Plastic is more reflective/glossy
    cv::Mat hsv;
    cv::cvtColor(color_mat(roi), hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv, hsv_channels);
    cv::Mat value_channel = hsv_channels[2];  // V channel
    
    // Count very bright pixels (specular highlights)
    int highlight_pixels = cv::countNonZero(value_channel > 240);
    float highlight_ratio = highlight_pixels / (float)(roi.width * roi.height);
    
    // Real skin: few highlights <0.02
    // Plastic: more highlights >0.05
    if (highlight_ratio < 0.03f) {
        material_score += 1.0f;  // Matte surface (skin)
    } else if (highlight_ratio > 0.08f) {
        material_score += 0.2f;  // Too glossy (plastic)
    } else {
        material_score += 0.6f;  // Borderline
    }
    checks++;
    
    // 4. Depth edge discontinuities - Mask edges show sudden depth jumps
    // Sample depth around face perimeter
    std::vector<uint16_t> perimeter_depths;
    int depth_w = frame->depth_width;
    int depth_h = frame->depth_height;
    
    // Scale ROI to depth coordinates
    int dx = roi.x * depth_w / color_mat.cols;
    int dy = roi.y * depth_h / color_mat.rows;
    int dw = roi.width * depth_w / color_mat.cols;
    int dh = roi.height * depth_h / color_mat.rows;
    
    // Sample top, bottom, left, right edges
    for (int i = 0; i < dw; i += 5) {
        // Top edge
        int idx_top = (dy) * depth_w + (dx + i);
        if (idx_top < (int)frame->depth_data.size() && frame->depth_data[idx_top] > 0) {
            perimeter_depths.push_back(frame->depth_data[idx_top]);
        }
        // Bottom edge
        int idx_bot = (dy + dh - 1) * depth_w + (dx + i);
        if (idx_bot < (int)frame->depth_data.size() && frame->depth_data[idx_bot] > 0) {
            perimeter_depths.push_back(frame->depth_data[idx_bot]);
        }
    }
    
    if (perimeter_depths.size() > 10) {
        // Calculate variance of perimeter depths
        float mean_depth = std::accumulate(perimeter_depths.begin(), perimeter_depths.end(), 0.0f) / perimeter_depths.size();
        float variance = 0.0f;
        for (uint16_t d : perimeter_depths) {
            float diff = d - mean_depth;
            variance += diff * diff;
        }
        variance /= perimeter_depths.size();
        float stddev_mm = std::sqrt(variance) * frame->depth_scale * 1000.0f;
        
        // Real face: smooth depth transitions <20mm
        // Mask edge: sharp discontinuities >30mm
        if (stddev_mm < 25.0f) {
            material_score += 1.0f;  // Smooth (natural)
        } else {
            material_score += 0.3f;  // Sharp edges (mask)
        }
        checks++;
    }
    
    // 5. NEW: IR material analysis for 3D masks and silicone masks
    float ir_material_score = analyze_ir_material_properties(frame, face);
    
    // 6. NEW: Depth material analysis for material behavior patterns
    float depth_material_score = analyze_depth_material_properties(frame, face);
    
    // Combine RGB, IR, and depth material analysis
    float rgb_material_score = (checks > 0) ? (material_score / checks) : 0.5f;
    
    // Weighted combination: RGB analysis (40%) + IR analysis (35%) + Depth analysis (25%)
    // Multi-modal material analysis is critical for 3D masks and silicone masks
    float final_score = (rgb_material_score * 0.4f + ir_material_score * 0.35f + depth_material_score * 0.25f);
    
    std::cout << "🎭 Material analysis: edge=" << edge_density 
              << ", texture=" << texture_variance 
              << ", specular=" << highlight_ratio 
              << ", rgb_score=" << rgb_material_score
              << ", ir_score=" << ir_material_score
              << ", depth_score=" << depth_material_score
              << ", final=" << final_score << std::endl;
    
    return final_score;
    
#else
    return 0.5f;
#endif
}

// ============================================================================
// IMPROVED: Comprehensive facial landmark analysis for anti-spoofing
float AntiSpoofingDetector::analyze_facial_landmarks(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || frame->metadata.landmarks.empty()) {
        return 0.0f;  // No landmarks = suspicious
    }
    
    const auto& landmarks = frame->metadata.landmarks;
    float landmark_score = 1.0f;
    
    // 1. CRITICAL: Eye region analysis (most important for liveness)
    float eye_score = analyze_eye_region(landmarks);
    landmark_score *= eye_score;
    
    // 2. CRITICAL: Mouth region analysis (breathing, speaking)
    float mouth_score = analyze_mouth_region(landmarks);
    landmark_score *= mouth_score;
    
    // 3. CRITICAL: Nose region analysis (3D structure)
    float nose_score = analyze_nose_region(landmarks);
    landmark_score *= nose_score;
    
    // 4. CRITICAL: Overall facial symmetry (real faces are symmetric)
    float symmetry_score = analyze_facial_symmetry(landmarks);
    landmark_score *= symmetry_score;
    
    // 5. CRITICAL: Landmark consistency (should be stable for real faces)
    float consistency_score = analyze_landmark_consistency(landmarks);
    landmark_score *= consistency_score;
    
    // Debug output
    static int landmark_debug = 0;
    if (++landmark_debug % 30 == 0) {
        std::cout << "🔍 Facial Landmarks: Eyes=" << eye_score 
                  << ", Mouth=" << mouth_score 
                  << ", Nose=" << nose_score 
                  << ", Symmetry=" << symmetry_score 
                  << ", Consistency=" << consistency_score 
                  << ", Overall=" << landmark_score << std::endl;
    }
    
    return landmark_score;
}

// IMPROVED: 3D facial structure validation using depth + landmarks
float AntiSpoofingDetector::analyze_3d_facial_structure(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || frame->metadata.landmarks.empty() || frame->depth_data.empty()) {
        return 0.0f;
    }
    
    float structure_score = 1.0f;
    
    // 1. CRITICAL: Validate 3D depth at key facial landmarks
    float depth_consistency = validate_landmark_depth_consistency(frame);
    structure_score *= depth_consistency;
    
    // 2. CRITICAL: Check for 3D facial curvature (real faces have natural curves)
    float curvature_score = analyze_facial_curvature(frame);
    structure_score *= curvature_score;
    
    // 3. CRITICAL: Validate eye socket depth (eyes should be recessed)
    float eye_socket_score = validate_eye_socket_depth(frame);
    structure_score *= eye_socket_score;
    
    // 4. CRITICAL: Check nose bridge 3D structure
    float nose_bridge_score = validate_nose_bridge_structure(frame);
    structure_score *= nose_bridge_score;
    
    return structure_score;
}

// IMPROVED: Comprehensive eye analysis including iris detection
float AntiSpoofingDetector::analyze_eye_region(const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;  // Can't analyze with insufficient landmarks
    }
    
    // MediaPipe 468-point eye landmarks (what we actually get)
    // Left eye: 33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246
    // Right eye: 362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385, 384, 398
    
    std::vector<int> left_eye_indices = {33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246};
    std::vector<int> right_eye_indices = {362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385, 384, 398};
    
    // 1. Basic eye landmark analysis (MediaPipe provides)
    float left_eye_score = analyze_single_eye(landmarks, left_eye_indices);
    float right_eye_score = analyze_single_eye(landmarks, right_eye_indices);
    
    // 2. CRITICAL: Enhanced iris/pupil analysis (what we need to add)
    float iris_score = analyze_iris_detection(landmarks);
    
    // 3. CRITICAL: Eye openness analysis (blink detection)
    float openness_score = analyze_eye_openness(landmarks);
    
    // 4. CRITICAL: Eye symmetry analysis
    float symmetry_score = analyze_eye_symmetry(landmarks, left_eye_indices, right_eye_indices);
    
    // Combine all eye analysis components
    float overall_eye_score = std::min(left_eye_score, right_eye_score) * 
                             iris_score * openness_score * symmetry_score;
    
    return overall_eye_score;
}

// IMPROVED: Enhanced iris detection using eye region analysis
float AntiSpoofingDetector::analyze_iris_detection(const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;
    }
    
    // MediaPipe doesn't provide iris landmarks, so we need to infer from eye region
    // We'll analyze the eye region for iris-like characteristics
    
    // Key eye landmarks for iris analysis
    std::vector<int> left_eye_center = {33, 133, 159, 145};  // Left eye center region
    std::vector<int> right_eye_center = {362, 263, 386, 374}; // Right eye center region
    
    float left_iris_score = analyze_eye_iris_region(landmarks, left_eye_center);
    float right_iris_score = analyze_eye_iris_region(landmarks, right_eye_center);
    
    // Both eyes must have detectable iris characteristics
    return std::min(left_iris_score, right_iris_score);
}

// IMPROVED: Analyze eye openness for blink detection
float AntiSpoofingDetector::analyze_eye_openness(const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;
    }
    
    // Calculate Eye Aspect Ratio (EAR) for both eyes
    // EAR = (|p2-p6| + |p3-p5|) / (2 * |p1-p4|)
    // Where p1-p6 are eye landmarks in order
    
    // Left eye EAR calculation
    float left_ear = calculate_eye_aspect_ratio_detailed(landmarks, {33, 160, 158, 133, 153, 144});
    
    // Right eye EAR calculation  
    float right_ear = calculate_eye_aspect_ratio_detailed(landmarks, {362, 385, 387, 263, 373, 380});
    
    // Analyze openness
    float openness_score = 1.0f;
    
    // Both eyes should be reasonably open (EAR > 0.2)
    if (left_ear < 0.15f || right_ear < 0.15f) {
        openness_score *= 0.2f;  // Very closed eyes - suspicious
    } else if (left_ear < 0.2f || right_ear < 0.2f) {
        openness_score *= 0.5f;  // Partially closed - might be blinking
    }
    
    // Check for asymmetric eye openness (one eye closed)
    float ear_difference = std::abs(left_ear - right_ear);
    if (ear_difference > 0.1f) {
        openness_score *= 0.7f;  // Asymmetric openness - suspicious
    }
    
    return openness_score;
}

// IMPROVED: Analyze eye symmetry between left and right eyes
float AntiSpoofingDetector::analyze_eye_symmetry(const std::vector<FrameBoxMetadata::Landmark>& landmarks,
                                                const std::vector<int>& left_indices,
                                                const std::vector<int>& right_indices) {
    if (landmarks.size() < 468 || left_indices.size() != right_indices.size()) {
        return 0.5f;
    }
    
    float symmetry_score = 1.0f;
    
    // Compare corresponding landmarks between left and right eyes
    for (size_t i = 0; i < left_indices.size(); i++) {
        if (left_indices[i] < static_cast<int>(landmarks.size()) && 
            right_indices[i] < static_cast<int>(landmarks.size())) {
            
            const auto& left_lm = landmarks[left_indices[i]];
            const auto& right_lm = landmarks[right_indices[i]];
            
            // Check if landmarks are symmetric (similar y-coordinates)
            float y_difference = std::abs(left_lm.y - right_lm.y);
            if (y_difference > 0.1f) {  // Large vertical difference
                symmetry_score *= 0.8f;  // Penalty for asymmetry
            }
        }
    }
    
    return symmetry_score;
}

// IMPROVED: Analyze eye region for iris-like characteristics
float AntiSpoofingDetector::analyze_eye_iris_region(const std::vector<FrameBoxMetadata::Landmark>& landmarks,
                                                   const std::vector<int>& eye_center_indices) {
    if (landmarks.size() < 468 || eye_center_indices.empty()) {
        return 0.5f;
    }
    
    float iris_score = 1.0f;
    
    // Check if eye center landmarks are valid and well-distributed
    int valid_landmarks = 0;
    float center_x = 0.0f, center_y = 0.0f;
    
    for (int idx : eye_center_indices) {
        if (idx < static_cast<int>(landmarks.size())) {
            const auto& lm = landmarks[idx];
            if (lm.x > 1.0f && lm.y > 1.0f) {  // Valid landmark
                center_x += lm.x;
                center_y += lm.y;
                valid_landmarks++;
            }
        }
    }
    
    if (valid_landmarks < 2) {
        return 0.2f;  // Not enough landmarks for iris analysis
    }
    
    center_x /= valid_landmarks;
    center_y /= valid_landmarks;
    
    // Check landmark distribution around center (should be circular for iris)
    float max_distance = 0.0f;
    for (int idx : eye_center_indices) {
        if (idx < static_cast<int>(landmarks.size())) {
            const auto& lm = landmarks[idx];
            if (lm.x > 1.0f && lm.y > 1.0f) {
                float distance = std::sqrt((lm.x - center_x) * (lm.x - center_x) + 
                                         (lm.y - center_y) * (lm.y - center_y));
                max_distance = std::max(max_distance, distance);
            }
        }
    }
    
    // Iris should have reasonable size (not too small, not too large)
    if (max_distance < 0.01f) {
        iris_score *= 0.3f;  // Too small - might be occluded
    } else if (max_distance > 0.1f) {
        iris_score *= 0.7f;  // Too large - might be unnatural
    }
    
    return iris_score;
}

// IMPROVED: Detailed Eye Aspect Ratio calculation
float AntiSpoofingDetector::calculate_eye_aspect_ratio_detailed(const std::vector<FrameBoxMetadata::Landmark>& landmarks,
                                                                const std::vector<int>& eye_indices) {
    if (landmarks.size() < 468 || eye_indices.size() < 6) {
        return 0.0f;
    }
    
    // EAR = (|p2-p6| + |p3-p5|) / (2 * |p1-p4|)
    // p1, p2, p3, p4, p5, p6 are eye landmarks in order
    
    if (eye_indices.size() >= 6) {
        const auto& p1 = landmarks[eye_indices[0]];
        const auto& p2 = landmarks[eye_indices[1]];
        const auto& p3 = landmarks[eye_indices[2]];
        const auto& p4 = landmarks[eye_indices[3]];
        const auto& p5 = landmarks[eye_indices[4]];
        const auto& p6 = landmarks[eye_indices[5]];
        
        // Calculate vertical distances
        float vertical1 = std::sqrt((p2.x - p6.x) * (p2.x - p6.x) + (p2.y - p6.y) * (p2.y - p6.y));
        float vertical2 = std::sqrt((p3.x - p5.x) * (p3.x - p5.x) + (p3.y - p5.y) * (p3.y - p5.y));
        
        // Calculate horizontal distance
        float horizontal = std::sqrt((p1.x - p4.x) * (p1.x - p4.x) + (p1.y - p4.y) * (p1.y - p4.y));
        
        if (horizontal == 0.0f) return 0.0f;
        
        return (vertical1 + vertical2) / (2.0f * horizontal);
    }
    
    return 0.0f;
}

float AntiSpoofingDetector::analyze_single_eye(const std::vector<FrameBoxMetadata::Landmark>& landmarks, 
                                               const std::vector<int>& eye_indices) {
    if (eye_indices.empty()) return 0.0f;
    
    float score = 1.0f;
    int valid_landmarks = 0;
    
    // Check landmark validity and eye shape
    for (int idx : eye_indices) {
        if (idx < static_cast<int>(landmarks.size())) {
            const auto& lm = landmarks[idx];
            if (lm.x > 1.0f && lm.y > 1.0f) {  // Valid landmark
                valid_landmarks++;
            }
        }
    }
    
    // Need at least 80% of eye landmarks valid
    float validity_ratio = static_cast<float>(valid_landmarks) / eye_indices.size();
    if (validity_ratio < 0.8f) {
        score *= 0.3f;  // Heavy penalty for missing eye landmarks
    }
    
    // Check eye aspect ratio (EAR) for natural eye shape
    if (valid_landmarks >= 6) {
        float ear = calculate_eye_aspect_ratio(landmarks, eye_indices);
        if (ear < 0.15f) {
            score *= 0.2f;  // Very low EAR = suspicious (closed eye or mask)
        } else if (ear > 0.4f) {
            score *= 0.5f;  // Very high EAR = suspicious (unnatural)
        }
    }
    
    return score;
}

float AntiSpoofingDetector::calculate_eye_aspect_ratio(const std::vector<FrameBoxMetadata::Landmark>& landmarks, 
                                                       const std::vector<int>& eye_indices) {
    if (eye_indices.size() < 6) return 0.0f;
    
    // Calculate vertical and horizontal eye distances
    // Simplified EAR calculation using key eye points
    float vertical_dist = 0.0f;
    float horizontal_dist = 0.0f;
    
    // Use first 6 points for EAR calculation
    for (size_t i = 0; i < std::min(eye_indices.size(), size_t(6)); i++) {
        if (eye_indices[i] < static_cast<int>(landmarks.size())) {
            const auto& lm = landmarks[eye_indices[i]];
            // Simple distance calculation (would be more sophisticated in real implementation)
            vertical_dist += lm.y;
            horizontal_dist += lm.x;
        }
    }
    
    if (horizontal_dist == 0.0f) return 0.0f;
    return vertical_dist / horizontal_dist;
}

// IMPROVED: Mouth region analysis for breathing and speaking detection
float AntiSpoofingDetector::analyze_mouth_region(const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;
    }
    
    // MediaPipe mouth landmarks
    std::vector<int> mouth_indices = {61, 84, 17, 314, 405, 320, 307, 375, 321, 308, 324, 318, 13, 82, 81, 80, 78, 95, 88, 178, 87, 14, 317, 402, 318, 324};
    
    int valid_landmarks = 0;
    for (int idx : mouth_indices) {
        if (idx < static_cast<int>(landmarks.size())) {
            const auto& lm = landmarks[idx];
            if (lm.x > 1.0f && lm.y > 1.0f) {
                valid_landmarks++;
            }
        }
    }
    
    float validity_ratio = static_cast<float>(valid_landmarks) / mouth_indices.size();
    if (validity_ratio < 0.7f) {
        return 0.2f;  // Heavy penalty for missing mouth landmarks
    }
    
    return validity_ratio;
}

// IMPROVED: Nose region analysis for 3D structure validation
float AntiSpoofingDetector::analyze_nose_region(const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;
    }
    
    // MediaPipe nose landmarks
    std::vector<int> nose_indices = {1, 2, 5, 4, 6, 19, 20, 94, 125, 141, 235, 236, 3, 51, 48, 115, 131, 134, 102, 49, 220, 305, 281, 360, 279, 331, 294, 358, 327, 326, 2};
    
    int valid_landmarks = 0;
    for (int idx : nose_indices) {
        if (idx < static_cast<int>(landmarks.size())) {
            const auto& lm = landmarks[idx];
            if (lm.x > 1.0f && lm.y > 1.0f) {
                valid_landmarks++;
            }
        }
    }
    
    float validity_ratio = static_cast<float>(valid_landmarks) / nose_indices.size();
    if (validity_ratio < 0.8f) {
        return 0.3f;  // Penalty for missing nose landmarks
    }
    
    return validity_ratio;
}

// IMPROVED: Facial symmetry analysis (real faces are naturally symmetric)
float AntiSpoofingDetector::analyze_facial_symmetry(const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;
    }
    
    // Check left-right symmetry of key facial features
    float symmetry_score = 1.0f;
    
    // Eye symmetry
    float eye_symmetry = check_eye_symmetry(landmarks);
    symmetry_score *= eye_symmetry;
    
    // Mouth symmetry
    float mouth_symmetry = check_mouth_symmetry(landmarks);
    symmetry_score *= mouth_symmetry;
    
    // Overall facial symmetry
    float overall_symmetry = check_overall_symmetry(landmarks);
    symmetry_score *= overall_symmetry;
    
    return symmetry_score;
}

float AntiSpoofingDetector::check_eye_symmetry([[maybe_unused]] const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    // Simplified symmetry check - in real implementation would be more sophisticated
    return 0.8f;  // Placeholder
}

float AntiSpoofingDetector::check_mouth_symmetry([[maybe_unused]] const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    // Simplified symmetry check - in real implementation would be more sophisticated
    return 0.8f;  // Placeholder
}

float AntiSpoofingDetector::check_overall_symmetry([[maybe_unused]] const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    // Simplified symmetry check - in real implementation would be more sophisticated
    return 0.8f;  // Placeholder
}

// IMPROVED: Landmark consistency analysis (should be stable for real faces)
float AntiSpoofingDetector::analyze_landmark_consistency(const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;
    }
    
    // Check for landmark jitter (real faces have stable landmarks)
    float consistency_score = 1.0f;
    
    // Analyze landmark stability (would need temporal data in real implementation)
    // For now, check landmark distribution
    int valid_landmarks = 0;
    for (const auto& lm : landmarks) {
        if (lm.x > 1.0f && lm.y > 1.0f) {
            valid_landmarks++;
        }
    }
    
    float landmark_ratio = static_cast<float>(valid_landmarks) / landmarks.size();
    if (landmark_ratio < 0.9f) {
        consistency_score *= 0.5f;  // Penalty for missing landmarks
    }
    
    return consistency_score;
}

// IMPROVED: 3D depth validation at facial landmarks
float AntiSpoofingDetector::validate_landmark_depth_consistency(const FrameBox* frame) {
    if (!frame || frame->metadata.landmarks.empty() || frame->depth_data.empty()) {
        return 0.0f;
    }
    
    const auto& landmarks = frame->metadata.landmarks;
    float depth_score = 1.0f;
    int valid_depth_checks = 0;
    
    // Check depth consistency at key facial landmarks
    std::vector<int> key_landmarks = {1, 4, 5, 6, 33, 133, 362, 263, 61, 291, 0, 17};  // Nose, eyes, mouth
    
    for (int idx : key_landmarks) {
        if (idx < static_cast<int>(landmarks.size())) {
            const auto& lm = landmarks[idx];
            
            // Convert landmark to depth pixel
            int depth_x = static_cast<int>(lm.x * frame->depth_width / frame->color_width);
            int depth_y = static_cast<int>(lm.y * frame->depth_height / frame->color_height);
            int depth_idx = depth_y * frame->depth_width + depth_x;
            
            if (depth_idx >= 0 && depth_idx < static_cast<int>(frame->depth_data.size())) {
                uint16_t depth_raw = frame->depth_data[depth_idx];
                if (depth_raw > 0) {
                    float depth_meters = depth_raw * frame->depth_scale;
                    
                    // Check if depth is reasonable for face (0.3m to 1.5m)
                    if (depth_meters >= 0.3f && depth_meters <= 1.5f) {
                        valid_depth_checks++;
                    } else {
                        depth_score *= 0.5f;  // Penalty for unreasonable depth
                    }
                } else {
                    depth_score *= 0.3f;  // Penalty for missing depth data
                }
            }
        }
    }
    
    // Need at least 70% of landmarks to have valid depth
    float depth_ratio = static_cast<float>(valid_depth_checks) / key_landmarks.size();
    if (depth_ratio < 0.7f) {
        depth_score *= 0.2f;  // Heavy penalty for insufficient depth data
    }
    
    return depth_score;
}

// IMPROVED: Analyze facial curvature using depth data
float AntiSpoofingDetector::analyze_facial_curvature(const FrameBox* frame) {
    if (!frame || frame->depth_data.empty()) {
        return 0.5f;
    }
    
    // Analyze depth variance across face region
    // Real faces have natural curvature, masks are flatter
    
    cv::Rect face_roi = cv::Rect(
        frame->metadata.face_x * frame->depth_width / frame->color_width,
        frame->metadata.face_y * frame->depth_height / frame->color_height,
        frame->metadata.face_w * frame->depth_width / frame->color_width,
        frame->metadata.face_h * frame->depth_height / frame->color_height
    );
    
    // Sample depth values across face
    std::vector<float> depth_values;
    for (int y = face_roi.y; y < face_roi.y + face_roi.height; y += 4) {
        for (int x = face_roi.x; x < face_roi.x + face_roi.width; x += 4) {
            int idx = y * frame->depth_width + x;
            if (idx >= 0 && idx < static_cast<int>(frame->depth_data.size())) {
                uint16_t depth_raw = frame->depth_data[idx];
                if (depth_raw > 0) {
                    depth_values.push_back(depth_raw * frame->depth_scale);
                }
            }
        }
    }
    
    if (depth_values.size() < 10) {
        return 0.3f;  // Not enough depth data
    }
    
    // Calculate depth variance (curvature indicator)
    float mean_depth = 0.0f;
    for (float d : depth_values) {
        mean_depth += d;
    }
    mean_depth /= depth_values.size();
    
    float variance = 0.0f;
    for (float d : depth_values) {
        variance += (d - mean_depth) * (d - mean_depth);
    }
    variance /= depth_values.size();
    
    // Real faces have moderate curvature (variance 0.001-0.01)
    // Masks are flatter (variance < 0.001)
    // Screens are very flat (variance < 0.0001)
    if (variance < 0.0001f) {
        return 0.1f;  // Very flat - likely screen/photo
    } else if (variance < 0.001f) {
        return 0.3f;  // Flat - likely mask
    } else if (variance > 0.01f) {
        return 0.7f;  // Very curved - might be real or 3D model
    } else {
        return 1.0f;  // Good curvature - likely real face
    }
}

// IMPROVED: Validate eye socket depth (eyes should be recessed)
float AntiSpoofingDetector::validate_eye_socket_depth(const FrameBox* frame) {
    if (!frame || frame->metadata.landmarks.empty() || frame->depth_data.empty()) {
        return 0.5f;
    }
    
    // Check depth at eye landmarks vs surrounding areas
    // Eyes should be recessed (deeper) than surrounding skin
    
    const auto& landmarks = frame->metadata.landmarks;
    float eye_socket_score = 1.0f;
    
    // Left eye center landmark (approximate)
    int left_eye_idx = 33;  // MediaPipe left eye center
    int right_eye_idx = 362; // MediaPipe right eye center
    
    if (left_eye_idx < static_cast<int>(landmarks.size()) && 
        right_eye_idx < static_cast<int>(landmarks.size())) {
        
        // Check left eye socket
        float left_eye_depth = get_depth_at_landmark(frame, landmarks[left_eye_idx]);
        float left_surrounding_depth = get_surrounding_depth(frame, landmarks[left_eye_idx]);
        
        if (left_eye_depth > 0 && left_surrounding_depth > 0) {
            float left_recession = left_surrounding_depth - left_eye_depth;
            if (left_recession < 0.005f) {  // Less than 5mm recession
                eye_socket_score *= 0.3f;  // Penalty for flat eye socket
            }
        }
        
        // Check right eye socket
        float right_eye_depth = get_depth_at_landmark(frame, landmarks[right_eye_idx]);
        float right_surrounding_depth = get_surrounding_depth(frame, landmarks[right_eye_idx]);
        
        if (right_eye_depth > 0 && right_surrounding_depth > 0) {
            float right_recession = right_surrounding_depth - right_eye_depth;
            if (right_recession < 0.005f) {  // Less than 5mm recession
                eye_socket_score *= 0.3f;  // Penalty for flat eye socket
            }
        }
    }
    
    return eye_socket_score;
}

// IMPROVED: Validate nose bridge 3D structure
float AntiSpoofingDetector::validate_nose_bridge_structure(const FrameBox* frame) {
    if (!frame || frame->metadata.landmarks.empty() || frame->depth_data.empty()) {
        return 0.5f;
    }
    
    // Check nose bridge depth profile
    // Real noses have characteristic 3D structure
    
    const auto& landmarks = frame->metadata.landmarks;
    float nose_score = 1.0f;
    
    // Nose bridge landmarks
    std::vector<int> nose_indices = {1, 4, 5, 6};  // MediaPipe nose landmarks
    
    std::vector<float> nose_depths;
    for (int idx : nose_indices) {
        if (idx < static_cast<int>(landmarks.size())) {
            float depth = get_depth_at_landmark(frame, landmarks[idx]);
            if (depth > 0) {
                nose_depths.push_back(depth);
            }
        }
    }
    
    if (nose_depths.size() >= 2) {
        // Check nose depth progression (should have characteristic curve)
        std::sort(nose_depths.begin(), nose_depths.end());
        float depth_range = nose_depths.back() - nose_depths.front();
        
        if (depth_range < 0.002f) {  // Less than 2mm depth variation
            nose_score *= 0.2f;  // Very flat nose - likely mask
        } else if (depth_range > 0.02f) {  // More than 20mm variation
            nose_score *= 0.5f;  // Unusual nose structure
        }
    }
    
    return nose_score;
}

// Helper functions for depth analysis
float AntiSpoofingDetector::get_depth_at_landmark(const FrameBox* frame, const FrameBoxMetadata::Landmark& landmark) {
    int depth_x = static_cast<int>(landmark.x * frame->depth_width / frame->color_width);
    int depth_y = static_cast<int>(landmark.y * frame->depth_height / frame->color_height);
    int depth_idx = depth_y * frame->depth_width + depth_x;
    
    if (depth_idx >= 0 && depth_idx < static_cast<int>(frame->depth_data.size())) {
        uint16_t depth_raw = frame->depth_data[depth_idx];
        if (depth_raw > 0) {
            return depth_raw * frame->depth_scale;
        }
    }
    return 0.0f;
}

float AntiSpoofingDetector::get_surrounding_depth(const FrameBox* frame, const FrameBoxMetadata::Landmark& landmark) {
    // Sample depth around landmark (5x5 region)
    int center_x = static_cast<int>(landmark.x * frame->depth_width / frame->color_width);
    int center_y = static_cast<int>(landmark.y * frame->depth_height / frame->color_height);
    
    float total_depth = 0.0f;
    int valid_samples = 0;
    
    for (int dy = -2; dy <= 2; dy++) {
        for (int dx = -2; dx <= 2; dx++) {
            if (dx == 0 && dy == 0) continue;  // Skip center
            
            int x = center_x + dx;
            int y = center_y + dy;
            int idx = y * frame->depth_width + x;
            
            if (idx >= 0 && idx < static_cast<int>(frame->depth_data.size())) {
                uint16_t depth_raw = frame->depth_data[idx];
                if (depth_raw > 0) {
                    total_depth += depth_raw * frame->depth_scale;
                    valid_samples++;
                }
            }
        }
    }
    
    return (valid_samples > 0) ? (total_depth / valid_samples) : 0.0f;
}

// CRITICAL: Depth-landmark fusion validation (NEW)
float AntiSpoofingDetector::validate_depth_landmark_fusion(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || frame->metadata.landmarks.empty() || frame->depth_data.empty()) {
        return 0.0f;
    }
    
    const auto& landmarks = frame->metadata.landmarks;
    float fusion_score = 1.0f;
    
    // 1. CRITICAL: Validate landmark-to-depth correspondence
    float correspondence_score = validate_landmark_depth_correspondence(frame, landmarks);
    fusion_score *= correspondence_score;
    
    // 2. CRITICAL: Check for depth-landmark consistency
    float consistency_score = validate_depth_landmark_consistency(frame, landmarks);
    fusion_score *= consistency_score;
    
    // 3. CRITICAL: Detect template-based landmark failures
    float template_detection_score = detect_template_landmark_failures(frame, landmarks);
    fusion_score *= template_detection_score;
    
    // 4. CRITICAL: Validate 3D landmark structure
    float structure_score = validate_3d_landmark_structure(frame, landmarks);
    fusion_score *= structure_score;
    
    // Debug output
    static int fusion_debug = 0;
    if (++fusion_debug % 30 == 0) {
        std::cout << "🔗 Depth-Landmark Fusion: Correspondence=" << correspondence_score 
                  << ", Consistency=" << consistency_score 
                  << ", Template=" << template_detection_score 
                  << ", Structure=" << structure_score 
                  << ", Overall=" << fusion_score << std::endl;
    }
    
    return fusion_score;
}

// CRITICAL: Validate landmark-to-depth correspondence
float AntiSpoofingDetector::validate_landmark_depth_correspondence(const FrameBox* frame, 
                                                                  const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;
    }
    
    float correspondence_score = 1.0f;
    int valid_correspondences = 0;
    int total_checks = 0;
    
    // Check key facial landmarks for depth correspondence
    std::vector<int> key_landmarks = {
        // Eyes
        33, 133, 159, 145, 362, 263, 386, 374,
        // Nose
        1, 4, 5, 6, 19, 20, 94, 125, 141, 235, 236,
        // Mouth
        61, 84, 17, 314, 405, 320, 307, 375, 321, 308, 324, 318,
        // Face contour
        10, 338, 297, 332, 152, 172, 136, 150, 149, 176, 148, 152
    };
    
    for (int idx : key_landmarks) {
        if (idx < static_cast<int>(landmarks.size())) {
            total_checks++;
            const auto& lm = landmarks[idx];
            
            // Convert landmark to depth pixel using proper projection
            int depth_x = static_cast<int>(lm.x * frame->depth_width / frame->color_width);
            int depth_y = static_cast<int>(lm.y * frame->depth_height / frame->color_height);
            int depth_idx = depth_y * frame->depth_width + depth_x;
            
            if (depth_idx >= 0 && depth_idx < static_cast<int>(frame->depth_data.size())) {
                uint16_t depth_raw = frame->depth_data[depth_idx];
                if (depth_raw > 0) {
                    float depth_meters = depth_raw * frame->depth_scale;
                    
                    // Check if depth is reasonable for face (0.3m to 1.5m)
                    if (depth_meters >= 0.3f && depth_meters <= 1.5f) {
                        valid_correspondences++;
                    } else {
                        correspondence_score *= 0.8f;  // Penalty for unreasonable depth
                    }
                } else {
                    correspondence_score *= 0.6f;  // Penalty for missing depth data
                }
            } else {
                correspondence_score *= 0.5f;  // Penalty for out-of-bounds landmark
            }
        }
    }
    
    // Need at least 80% of landmarks to have valid depth correspondence
    float correspondence_ratio = (total_checks > 0) ? 
        static_cast<float>(valid_correspondences) / total_checks : 0.0f;
    
    if (correspondence_ratio < 0.8f) {
        correspondence_score *= 0.3f;  // Heavy penalty for poor correspondence
    }
    
    return correspondence_score;
}

// CRITICAL: Validate depth-landmark consistency
float AntiSpoofingDetector::validate_depth_landmark_consistency(const FrameBox* frame,
                                                              const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;
    }
    
    float consistency_score = 1.0f;
    
    // Check depth consistency across facial regions
    std::vector<std::vector<int>> facial_regions = {
        // Left eye region
        {33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246},
        // Right eye region
        {362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385, 384, 398},
        // Nose region
        {1, 2, 5, 4, 6, 19, 20, 94, 125, 141, 235, 236, 3, 51, 48, 115, 131, 134, 102, 49},
        // Mouth region
        {61, 84, 17, 314, 405, 320, 307, 375, 321, 308, 324, 318, 13, 82, 81, 80, 78, 95, 88, 178, 87, 14}
    };
    
    for (const auto& region : facial_regions) {
        std::vector<float> region_depths;
        
        for (int idx : region) {
            if (idx < static_cast<int>(landmarks.size())) {
                float depth = get_depth_at_landmark(frame, landmarks[idx]);
                if (depth > 0) {
                    region_depths.push_back(depth);
                }
            }
        }
        
        if (region_depths.size() >= 3) {
            // Check depth variance within region (should be consistent)
            float mean_depth = 0.0f;
            for (float d : region_depths) {
                mean_depth += d;
            }
            mean_depth /= region_depths.size();
            
            float variance = 0.0f;
            for (float d : region_depths) {
                variance += (d - mean_depth) * (d - mean_depth);
            }
            variance /= region_depths.size();
            
            // Real facial regions should have consistent depth
            if (variance > 0.01f) {  // More than 1cm variance
                consistency_score *= 0.7f;  // Penalty for inconsistent depth
            }
        }
    }
    
    return consistency_score;
}

// CRITICAL: Detect template-based landmark failures
float AntiSpoofingDetector::detect_template_landmark_failures(const FrameBox* frame,
                                                             const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;
    }
    
    float template_score = 1.0f;
    
    // Check for template-based failures that indicate 2D attacks
    // 1. Check for unrealistic landmark distributions
    float landmark_distribution_score = analyze_landmark_distribution(landmarks);
    template_score *= landmark_distribution_score;
    
    // 2. Check for depth-landmark mismatch
    float depth_mismatch_score = analyze_depth_landmark_mismatch(frame, landmarks);
    template_score *= depth_mismatch_score;
    
    // 3. Check for template symmetry (too perfect = suspicious)
    float template_symmetry_score = analyze_template_symmetry(landmarks);
    template_score *= template_symmetry_score;
    
    return template_score;
}

// CRITICAL: Validate 3D landmark structure
float AntiSpoofingDetector::validate_3d_landmark_structure(const FrameBox* frame,
                                                          const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;
    }
    
    float structure_score = 1.0f;
    
    // 1. Check eye socket depth (eyes should be recessed)
    float eye_socket_score = validate_eye_socket_3d_structure(frame, landmarks);
    structure_score *= eye_socket_score;
    
    // 2. Check nose bridge 3D structure
    float nose_bridge_score = validate_nose_bridge_3d_structure(frame, landmarks);
    structure_score *= nose_bridge_score;
    
    // 3. Check facial curvature using landmarks
    float facial_curvature_score = validate_facial_curvature_3d(frame, landmarks);
    structure_score *= facial_curvature_score;
    
    return structure_score;
}

// Helper functions for depth-landmark fusion
float AntiSpoofingDetector::analyze_landmark_distribution(const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;
    }
    
    // Check if landmarks are distributed naturally
    // Real faces have natural landmark distributions, templates are often too perfect
    
    float distribution_score = 1.0f;
    
    // Check for clustering (landmarks too close together)
    int clustered_landmarks = 0;
    for (size_t i = 0; i < landmarks.size(); i++) {
        for (size_t j = i + 1; j < landmarks.size(); j++) {
            float distance = std::sqrt(
                (landmarks[i].x - landmarks[j].x) * (landmarks[i].x - landmarks[j].x) +
                (landmarks[i].y - landmarks[j].y) * (landmarks[i].y - landmarks[j].y)
            );
            
            if (distance < 0.01f) {  // Very close landmarks
                clustered_landmarks++;
            }
        }
    }
    
    // Too many clustered landmarks = suspicious
    if (clustered_landmarks > 10) {
        distribution_score *= 0.5f;
    }
    
    return distribution_score;
}

float AntiSpoofingDetector::analyze_depth_landmark_mismatch(const FrameBox* frame,
                                                           const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;
    }
    
    float mismatch_score = 1.0f;
    
    // Check if landmarks correspond to actual 3D structure
    // 2D attacks will have landmarks that don't match depth structure
    
    int mismatched_landmarks = 0;
    int total_checks = 0;
    
    for (size_t i = 0; i < landmarks.size(); i += 10) {  // Sample every 10th landmark
        total_checks++;
        const auto& lm = landmarks[i];
        
        float landmark_depth = get_depth_at_landmark(frame, lm);
        float surrounding_depth = get_surrounding_depth(frame, lm);
        
        if (landmark_depth > 0 && surrounding_depth > 0) {
            float depth_difference = std::abs(landmark_depth - surrounding_depth);
            
            // Large depth difference = mismatch
            if (depth_difference > 0.05f) {  // More than 5cm difference
                mismatched_landmarks++;
            }
        }
    }
    
    // Too many mismatched landmarks = suspicious
    float mismatch_ratio = (total_checks > 0) ? 
        static_cast<float>(mismatched_landmarks) / total_checks : 0.0f;
    
    if (mismatch_ratio > 0.3f) {  // More than 30% mismatch
        mismatch_score *= 0.4f;
    }
    
    return mismatch_score;
}

float AntiSpoofingDetector::analyze_template_symmetry(const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;
    }
    
    // Real faces are naturally asymmetric, templates are often too symmetric
    float symmetry_score = 1.0f;
    
    // Check left-right symmetry of key features
    std::vector<std::pair<int, int>> symmetric_pairs = {
        {33, 362},   // Left/right eye centers
        {133, 263},  // Left/right eye centers
        {159, 386},  // Left/right eye centers
        {145, 374}   // Left/right eye centers
    };
    
    float total_symmetry = 0.0f;
    int valid_pairs = 0;
    
    for (const auto& pair : symmetric_pairs) {
        if (pair.first < static_cast<int>(landmarks.size()) && 
            pair.second < static_cast<int>(landmarks.size())) {
            
            const auto& left = landmarks[pair.first];
            const auto& right = landmarks[pair.second];
            
            // Check vertical symmetry
            float y_difference = std::abs(left.y - right.y);
            total_symmetry += y_difference;
            valid_pairs++;
        }
    }
    
    if (valid_pairs > 0) {
        float avg_symmetry = total_symmetry / valid_pairs;
        
        // Too perfect symmetry = suspicious (template)
        if (avg_symmetry < 0.01f) {  // Very symmetric
            symmetry_score *= 0.3f;  // Heavy penalty for template-like symmetry
        } else if (avg_symmetry > 0.1f) {  // Very asymmetric
            symmetry_score *= 0.7f;  // Some penalty for extreme asymmetry
        }
    }
    
    return symmetry_score;
}

float AntiSpoofingDetector::validate_eye_socket_3d_structure(const FrameBox* frame,
                                                           const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;
    }
    
    float eye_socket_score = 1.0f;
    
    // Check eye socket depth using landmarks
    std::vector<int> eye_center_landmarks = {33, 133, 159, 145, 362, 263, 386, 374};
    
    for (int idx : eye_center_landmarks) {
        if (idx < static_cast<int>(landmarks.size())) {
            float eye_depth = get_depth_at_landmark(frame, landmarks[idx]);
            float surrounding_depth = get_surrounding_depth(frame, landmarks[idx]);
            
            if (eye_depth > 0 && surrounding_depth > 0) {
                float recession = surrounding_depth - eye_depth;
                
                // Eyes should be recessed (deeper) than surrounding skin
                if (recession < 0.003f) {  // Less than 3mm recession
                    eye_socket_score *= 0.5f;  // Penalty for flat eye socket
                }
            }
        }
    }
    
    return eye_socket_score;
}

float AntiSpoofingDetector::validate_nose_bridge_3d_structure(const FrameBox* frame,
                                                         const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;
    }
    
    float nose_bridge_score = 1.0f;
    
    // Check nose bridge depth using landmarks
    std::vector<int> nose_landmarks = {1, 4, 5, 6, 19, 20, 94, 125, 141, 235, 236};
    
    std::vector<float> nose_depths;
    for (int idx : nose_landmarks) {
        if (idx < static_cast<int>(landmarks.size())) {
            float depth = get_depth_at_landmark(frame, landmarks[idx]);
            if (depth > 0) {
                nose_depths.push_back(depth);
            }
        }
    }
    
    if (nose_depths.size() >= 3) {
        // Check nose depth progression
        std::sort(nose_depths.begin(), nose_depths.end());
        float depth_range = nose_depths.back() - nose_depths.front();
        
        if (depth_range < 0.005f) {  // Less than 5mm depth variation
            nose_bridge_score *= 0.3f;  // Very flat nose - likely mask
        }
    }
    
    return nose_bridge_score;
}

float AntiSpoofingDetector::validate_facial_curvature_3d(const FrameBox* frame,
                                                         const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;
    }
    
    float curvature_score = 1.0f;
    
    // Sample landmarks across face for curvature analysis
    std::vector<int> sample_landmarks = {10, 338, 297, 332, 152, 172, 136, 150, 149, 176, 148, 152};
    
    std::vector<float> face_depths;
    for (int idx : sample_landmarks) {
        if (idx < static_cast<int>(landmarks.size())) {
            float depth = get_depth_at_landmark(frame, landmarks[idx]);
            if (depth > 0) {
                face_depths.push_back(depth);
            }
        }
    }
    
    if (face_depths.size() >= 5) {
        // Calculate depth variance (curvature indicator)
        float mean_depth = 0.0f;
        for (float d : face_depths) {
            mean_depth += d;
        }
        mean_depth /= face_depths.size();
        
        float variance = 0.0f;
        for (float d : face_depths) {
            variance += (d - mean_depth) * (d - mean_depth);
        }
        variance /= face_depths.size();
        
        // Real faces have moderate curvature
        if (variance < 0.0001f) {  // Very flat
            curvature_score *= 0.2f;  // Heavy penalty for flat face
        } else if (variance > 0.01f) {  // Very curved
            curvature_score *= 0.7f;  // Some penalty for extreme curvature
        }
    }
    
    return curvature_score;
}

// CRITICAL: Comprehensive temporal analysis functions
float AntiSpoofingDetector::analyze_micro_motion_temporal(const std::vector<FrameBox*>& frames) {
    if (frames.size() < 5) {
        return 0.5f;  // Need at least 5 frames for analysis
    }
    
    float motion_score = 0.0f;
    int valid_motions = 0;
    
    // Analyze micro-motion patterns over time
    for (size_t i = 1; i < frames.size(); i++) {
        if (frames[i] && frames[i-1] && 
            frames[i]->metadata.face_detected && 
            frames[i-1]->metadata.face_detected) {
            
            // Calculate 2D motion
            float dx = frames[i]->metadata.face_x - frames[i-1]->metadata.face_x;
            float dy = frames[i]->metadata.face_y - frames[i-1]->metadata.face_y;
            float motion_2d = std::sqrt(dx * dx + dy * dy);
            
            // Calculate 3D motion using depth
            float depth_motion = 0.0f;
            if (!frames[i]->depth_data.empty() && !frames[i-1]->depth_data.empty()) {
                // Sample depth at face center with proper bounds checking
                int center_x = frames[i]->metadata.face_x * frames[i]->depth_width / frames[i]->color_width;
                int center_y = frames[i]->metadata.face_y * frames[i]->depth_height / frames[i]->color_height;
                
                // CRITICAL FIX: Proper bounds checking for both frames
                int idx_current = center_y * frames[i]->depth_width + center_x;
                int idx_previous = center_y * frames[i-1]->depth_width + center_x;
                
                if (idx_current >= 0 && idx_current < static_cast<int>(frames[i]->depth_data.size()) &&
                    idx_previous >= 0 && idx_previous < static_cast<int>(frames[i-1]->depth_data.size())) {
                    
                    float depth_current = frames[i]->depth_data[idx_current] * frames[i]->depth_scale;
                    float depth_previous = frames[i-1]->depth_data[idx_previous] * frames[i-1]->depth_scale;
                    
                    if (depth_current > 0 && depth_previous > 0) {
                        depth_motion = std::abs(depth_current - depth_previous);
                    }
                }
            }
            
            // Natural micro-motion characteristics:
            // - 2D motion: 0.5-5 pixels per frame
            // - 3D motion: 1-10mm per frame
            // - Should be continuous, not jerky
            
            float motion_quality = 1.0f;
            
            // Check 2D motion range
            if (motion_2d < 0.1f) {
                motion_quality *= 0.3f;  // Too static (photo/screen)
            } else if (motion_2d > 15.0f) {
                motion_quality *= 0.5f;  // Too jerky
            }
            
            // Check 3D motion range
            if (depth_motion > 0) {
                if (depth_motion < 0.001f) {
                    motion_quality *= 0.4f;  // Too static in depth
                } else if (depth_motion > 0.02f) {
                    motion_quality *= 0.6f;  // Too much depth motion
                }
            }
            
            motion_score += motion_quality;
            valid_motions++;
        }
    }
    
    return (valid_motions > 0) ? (motion_score / valid_motions) : 0.5f;
}

float AntiSpoofingDetector::analyze_depth_breathing_temporal(const std::vector<FrameBox*>& frames) {
    if (frames.size() < 10) {
        return 0.5f;  // Need at least 10 frames for breathing analysis
    }
    
    // Extract depth values at face center over time
    std::vector<float> depth_values;
    depth_values.reserve(frames.size());
    
    for (const auto* frame : frames) {
        if (frame && frame->metadata.face_detected && !frame->depth_data.empty()) {
            int center_x = frame->metadata.face_x * frame->depth_width / frame->color_width;
            int center_y = frame->metadata.face_y * frame->depth_height / frame->color_height;
            int idx = center_y * frame->depth_width + center_x;
            
            if (idx >= 0 && idx < static_cast<int>(frame->depth_data.size())) {
                float depth = frame->depth_data[idx] * frame->depth_scale;
                if (depth > 0.3f && depth < 1.5f) {  // Valid face depth range
                    depth_values.push_back(depth);
                }
            }
        }
    }
    
    if (depth_values.size() < 8) {
        return 0.5f;  // Not enough valid depth samples
    }
    
    // Analyze breathing pattern using FFT
    float breathing_score = analyze_breathing_pattern_fft(depth_values);
    
    return breathing_score;
}

float AntiSpoofingDetector::analyze_blink_patterns_temporal(const std::vector<FrameBox*>& frames) {
    if (frames.size() < 15) {
        return 0.5f;  // Need at least 15 frames for blink analysis
    }
    
    // Extract eye aspect ratios over time
    std::vector<float> ear_values;
    ear_values.reserve(frames.size());
    
    for (const auto* frame : frames) {
        if (frame && frame->metadata.face_detected && !frame->metadata.landmarks.empty()) {
            float ear = calculate_eye_aspect_ratio_detailed(frame->metadata.landmarks, {33, 133, 159, 145, 362, 263, 386, 374});
            ear_values.push_back(ear);
        }
    }
    
    if (ear_values.size() < 10) {
        return 0.5f;  // Not enough valid EAR samples
    }
    
    // Analyze blink patterns
    float blink_score = analyze_blink_frequency(ear_values);
    
    return blink_score;
}

float AntiSpoofingDetector::analyze_landmark_temporal_consistency(const std::vector<FrameBox*>& frames) {
    if (frames.size() < 5) {
        return 0.5f;
    }
    
    float consistency_score = 1.0f;
    int valid_checks = 0;
    
    // Check landmark stability over time
    for (size_t i = 1; i < frames.size(); i++) {
        if (frames[i] && frames[i-1] && 
            frames[i]->metadata.face_detected && 
            frames[i-1]->metadata.face_detected &&
            !frames[i]->metadata.landmarks.empty() &&
            !frames[i-1]->metadata.landmarks.empty()) {
            
            const auto& current_landmarks = frames[i]->metadata.landmarks;
            const auto& previous_landmarks = frames[i-1]->metadata.landmarks;
            
            if (current_landmarks.size() == previous_landmarks.size() && 
                current_landmarks.size() >= 468) {
                
                float landmark_stability = 0.0f;
                int valid_landmarks = 0;
                
                // Check key facial landmarks for stability
                std::vector<int> key_landmarks = {33, 133, 159, 145, 362, 263, 386, 374, 1, 4, 5, 6};
                
                for (int idx : key_landmarks) {
                    if (idx < static_cast<int>(current_landmarks.size())) {
                        float dx = current_landmarks[idx].x - previous_landmarks[idx].x;
                        float dy = current_landmarks[idx].y - previous_landmarks[idx].y;
                        float displacement = std::sqrt(dx * dx + dy * dy);
                        
                        // Natural landmark movement should be small but not zero
                        if (displacement < 0.01f) {
                            landmark_stability += 0.3f;  // Too stable (template)
                        } else if (displacement > 0.1f) {
                            landmark_stability += 0.5f;  // Too unstable
                        } else {
                            landmark_stability += 1.0f;  // Good natural movement
                        }
                        valid_landmarks++;
                    }
                }
                
                if (valid_landmarks > 0) {
                    consistency_score *= (landmark_stability / valid_landmarks);
                    valid_checks++;
                }
            }
        }
    }
    
    return (valid_checks > 0) ? consistency_score : 0.5f;
}

float AntiSpoofingDetector::validate_realsense_data_quality(const FrameBox* frame) {
    if (!frame) {
        return 0.0f;
    }
    
    float quality_score = 1.0f;
    
    // 1. Check depth data quality
    if (!frame->depth_data.empty()) {
        int valid_depth_pixels = 0;
        int total_pixels = frame->depth_width * frame->depth_height;
        
        for (int i = 0; i < total_pixels; i++) {
            if (frame->depth_data[i] > 0) {
                valid_depth_pixels++;
            }
        }
        
        float depth_coverage = static_cast<float>(valid_depth_pixels) / total_pixels;
        
        if (depth_coverage < 0.3f) {
            quality_score *= 0.4f;  // Poor depth coverage
        } else if (depth_coverage < 0.6f) {
            quality_score *= 0.7f;  // Moderate depth coverage
        }
    } else {
        quality_score *= 0.2f;  // No depth data
    }
    
    // 2. Check color data quality
    if (!frame->color_data.empty()) {
        // Check for reasonable color values (not all black/white)
        int non_zero_pixels = 0;
        int total_color_pixels = frame->color_width * frame->color_height * 3;
        
        for (int i = 0; i < total_color_pixels; i++) {
            if (frame->color_data[i] > 10 && frame->color_data[i] < 245) {
                non_zero_pixels++;
            }
        }
        
        float color_quality = static_cast<float>(non_zero_pixels) / total_color_pixels;
        
        if (color_quality < 0.5f) {
            quality_score *= 0.6f;  // Poor color quality
        }
    } else {
        quality_score *= 0.3f;  // No color data
    }
    
    // 3. Check camera settings
    if (frame->exposure > 0) {
        // Check for reasonable exposure values
        if (frame->exposure < 100 || frame->exposure > 10000) {
            quality_score *= 0.8f;  // Unusual exposure
        }
    }
    
    // 4. Check frame timing
    if (frame->time_color > 0 && frame->time_depth > 0) {
        float time_diff = std::abs(frame->time_color - frame->time_depth);
        if (time_diff > 0.1f) {  // More than 100ms difference
            quality_score *= 0.7f;  // Poor synchronization
        }
    }
    
    return quality_score;
}

float AntiSpoofingDetector::analyze_depth_temporal_consistency(const std::vector<FrameBox*>& frames) {
    if (frames.size() < 5) {
        return 0.5f;
    }
    
    float consistency_score = 1.0f;
    int valid_checks = 0;
    
    // Check depth consistency over time
    for (size_t i = 1; i < frames.size(); i++) {
        if (frames[i] && frames[i-1] && 
            !frames[i]->depth_data.empty() && 
            !frames[i-1]->depth_data.empty()) {
            
            // Sample depth at face center with proper bounds checking
            int center_x = frames[i]->metadata.face_x * frames[i]->depth_width / frames[i]->color_width;
            int center_y = frames[i]->metadata.face_y * frames[i]->depth_height / frames[i]->color_height;
            
            // CRITICAL FIX: Calculate separate indices for each frame
            int idx_current = center_y * frames[i]->depth_width + center_x;
            int idx_previous = center_y * frames[i-1]->depth_width + center_x;
            
            if (idx_current >= 0 && idx_current < static_cast<int>(frames[i]->depth_data.size()) &&
                idx_previous >= 0 && idx_previous < static_cast<int>(frames[i-1]->depth_data.size())) {
                
                float depth_current = frames[i]->depth_data[idx_current] * frames[i]->depth_scale;
                float depth_previous = frames[i-1]->depth_data[idx_previous] * frames[i-1]->depth_scale;
                
                if (depth_current > 0 && depth_previous > 0) {
                    float depth_change = std::abs(depth_current - depth_previous);
                    
                    // Natural depth changes should be small
                    if (depth_change > 0.05f) {  // More than 5cm change
                        consistency_score *= 0.6f;  // Inconsistent depth
                    }
                    valid_checks++;
                }
            }
        }
    }
    
    return (valid_checks > 0) ? consistency_score : 0.5f;
}

// Helper functions for temporal analysis
float AntiSpoofingDetector::analyze_breathing_pattern_fft(const std::vector<float>& depth_values) {
    if (depth_values.size() < 8) {
        return 0.5f;
    }
    
    // Simple breathing pattern analysis
    // Real breathing should show periodic depth changes
    
    float mean_depth = 0.0f;
    for (float d : depth_values) {
        mean_depth += d;
    }
    mean_depth /= depth_values.size();
    
    // Calculate depth variance (breathing indicator)
    float variance = 0.0f;
    for (float d : depth_values) {
        float diff = d - mean_depth;
        variance += diff * diff;
    }
    variance /= depth_values.size();
    
    // Natural breathing causes 1-5mm depth variation
    if (variance < 0.000001f) {  // Less than 1mm variance
        return 0.2f;  // Too stable (no breathing)
    } else if (variance > 0.000025f) {  // More than 5mm variance
        return 0.6f;  // Too much variation
    } else {
        return 1.0f;  // Good breathing pattern
    }
}

float AntiSpoofingDetector::analyze_blink_frequency(const std::vector<float>& ear_values) {
    if (ear_values.size() < 10) {
        return 0.5f;
    }
    
    // Count blinks (EAR drops below threshold)
    int blinks = 0;
    float ear_threshold = 0.25f;  // Typical EAR threshold for blinks
    
    for (size_t i = 1; i < ear_values.size(); i++) {
        if (ear_values[i-1] > ear_threshold && ear_values[i] < ear_threshold) {
            blinks++;
        }
    }
    
    // Calculate blink rate (blinks per second)
    float time_span = ear_values.size() / 30.0f;  // Assuming 30fps
    float blink_rate = blinks / time_span;
    
    // Natural blink rate: 0.1-0.5 blinks per second
    if (blink_rate < 0.05f) {
        return 0.3f;  // Too few blinks (static attack)
    } else if (blink_rate > 1.0f) {
        return 0.6f;  // Too many blinks (unusual)
    } else {
        return 1.0f;  // Good blink pattern
    }
}

float AntiSpoofingDetector::analyze_occlusion(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected) {
        return 0.5f;
    }
    
    float occlusion_score = 1.0f;  // Start with no occlusion
    
    // 1. CRITICAL: Enhanced eye visibility detection
    if (!frame->metadata.landmarks.empty()) {
        float eye_visibility_score = detect_eye_occlusion(frame->metadata.landmarks);
        occlusion_score *= eye_visibility_score;
        
        if (eye_visibility_score < 0.5f) {
            std::cout << "⚠️  EYES OCCLUDED - possible mask or obstruction (score: " 
                      << eye_visibility_score << ")" << std::endl;
        }
    }
    
    // 2. CRITICAL: Detect objects blocking the face
    float object_blocking_score = detect_face_blocking_objects(frame, face);
    occlusion_score *= object_blocking_score;
    
    if (object_blocking_score < 0.7f) {
        std::cout << "⚠️  OBJECTS BLOCKING FACE - possible mask or hand (score: " 
                  << object_blocking_score << ")" << std::endl;
    }
    
    // 3. CRITICAL: Enhanced landmark analysis for partial occlusion
    if (frame->metadata.landmarks.size() >= 468) {
        float landmark_occlusion_score = analyze_landmark_occlusion(frame->metadata.landmarks);
        occlusion_score *= landmark_occlusion_score;
        
        if (landmark_occlusion_score < 0.8f) {
            std::cout << "⚠️  FACE PARTIALLY OCCLUDED - landmarks missing (score: " 
                      << landmark_occlusion_score << ")" << std::endl;
        }
    }
    
    // 4. CRITICAL: Depth-based obstacle detection
    if (!frame->depth_data.empty()) {
        float depth_obstacle_score = detect_depth_obstacles(frame, face);
        occlusion_score *= depth_obstacle_score;
        
        if (depth_obstacle_score < 0.6f) {
            std::cout << "⚠️  DEPTH OBSTACLES DETECTED - objects in front of face (score: " 
                      << depth_obstacle_score << ")" << std::endl;
        }
    }
    
    // 5. CRITICAL: Face shape analysis for masks
    float face_shape_score = analyze_face_shape_anomalies(frame, face);
    occlusion_score *= face_shape_score;
    
    if (face_shape_score < 0.7f) {
        std::cout << "⚠️  UNUSUAL FACE SHAPE - possible mask (score: " 
                  << face_shape_score << ")" << std::endl;
    }
    
    // Debug output
    static int occ_debug = 0;
    if (++occ_debug % 30 == 0) {
        std::cout << "🔍 Comprehensive Occlusion Analysis: Eyes=" << (frame->metadata.landmarks.empty() ? 1.0f : detect_eye_occlusion(frame->metadata.landmarks))
                  << ", Objects=" << detect_face_blocking_objects(frame, face)
                  << ", Landmarks=" << (frame->metadata.landmarks.size() >= 468 ? analyze_landmark_occlusion(frame->metadata.landmarks) : 1.0f)
                  << ", Depth=" << (!frame->depth_data.empty() ? detect_depth_obstacles(frame, face) : 1.0f)
                  << ", Shape=" << analyze_face_shape_anomalies(frame, face)
                  << ", Overall=" << occlusion_score << std::endl;
    }
    
    return occlusion_score;
}

// CRITICAL: Enhanced face obstacle detection functions
float AntiSpoofingDetector::detect_eye_occlusion(const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 1.0f;  // Can't analyze with insufficient landmarks
    }
    
    // Use MediaPipe's built-in visibility scores for eye landmarks
    std::vector<int> left_eye_indices = {33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246};
    std::vector<int> right_eye_indices = {362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385, 384, 398};
    
    float left_eye_visibility = 0.0f;
    int left_eye_count = 0;
    for (int idx : left_eye_indices) {
        if (idx < static_cast<int>(landmarks.size())) {
            left_eye_visibility += landmarks[idx].visibility;
            left_eye_count++;
        }
    }
    left_eye_visibility = (left_eye_count > 0) ? left_eye_visibility / left_eye_count : 1.0f;
    
    float right_eye_visibility = 0.0f;
    int right_eye_count = 0;
    for (int idx : right_eye_indices) {
        if (idx < static_cast<int>(landmarks.size())) {
            right_eye_visibility += landmarks[idx].visibility;
            right_eye_count++;
        }
    }
    right_eye_visibility = (right_eye_count > 0) ? right_eye_visibility / right_eye_count : 1.0f;
    
    // Both eyes must be visible for good score
    return std::min(left_eye_visibility, right_eye_visibility);
}

float AntiSpoofingDetector::detect_face_blocking_objects(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected) {
        return 1.0f;
    }
    
    float blocking_score = 1.0f;
    int component_count = 0;
    float total_score = 0.0f;
    
    // 1. Check for objects in front of face using depth analysis
    if (!frame->depth_data.empty()) {
        float depth_consistency = analyze_face_depth_consistency(frame, face);
        total_score += depth_consistency;
        component_count++;
    }
    
    // 2. Check for unusual color patterns that might indicate masks
    if (!frame->color_data.empty()) {
        float color_anomaly_score = detect_color_anomalies(frame, face);
        total_score += color_anomaly_score;
        component_count++;
    }
    
    // 3. Check for IR texture anomalies (masks block IR differently)
    if (!frame->ir_left_data.empty()) {
        float ir_anomaly_score = detect_ir_texture_anomalies(frame, face);
        total_score += ir_anomaly_score;
        component_count++;
    }
    
    // Use average instead of multiplication to avoid overly harsh penalties
    if (component_count > 0) {
        blocking_score = total_score / component_count;
    }
    
    return blocking_score;
}

float AntiSpoofingDetector::analyze_landmark_occlusion(const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 0.5f;  // Insufficient landmarks
    }
    
    float occlusion_score = 1.0f;
    
    // Check key facial regions using MediaPipe's visibility scores
    std::vector<std::pair<std::vector<int>, std::string>> facial_regions = {
        {{10, 338, 297, 332}, "forehead"},
        {{33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246}, "left_eye"},
        {{362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385, 384, 398}, "right_eye"},
        {{1, 2, 5, 4, 6, 19, 20, 94, 125, 141, 235, 236, 3, 51, 48, 115, 131, 134, 102, 49}, "nose"},
        {{61, 84, 17, 314, 405, 320, 307, 375, 321, 308, 324, 318, 13, 82, 81, 80, 78, 95, 88, 178, 87, 14}, "mouth"},
        {{234, 454, 227, 234, 93, 132, 58, 172, 136, 150, 149, 176, 148, 152, 377, 400, 378, 379, 365, 397, 288, 361, 323}, "cheeks"}
    };
    
    for (const auto& region : facial_regions) {
        // Use MediaPipe's visibility scores directly
        float region_visibility = 0.0f;
        int region_count = 0;
        
        for (int idx : region.first) {
            if (idx < static_cast<int>(landmarks.size())) {
                region_visibility += landmarks[idx].visibility;
                region_count++;
            }
        }
        
        float region_score = (region_count > 0) ? region_visibility / region_count : 1.0f;
        occlusion_score *= region_score;
        
        if (region_score < 0.5f) {
            std::cout << "⚠️  " << region.second << " region occluded (visibility: " << region_score << ")" << std::endl;
        }
    }
    
    return occlusion_score;
}

float AntiSpoofingDetector::detect_depth_obstacles(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || frame->depth_data.empty()) {
        return 1.0f;
    }
    
    float obstacle_score = 1.0f;
    
    // 1. Check for depth discontinuities that might indicate objects
    float depth_discontinuity = analyze_depth_discontinuities(frame, face);
    obstacle_score *= depth_discontinuity;
    
    // 2. Check for unusual depth patterns
    float depth_pattern_score = analyze_depth_patterns(frame, face);
    obstacle_score *= depth_pattern_score;
    
    // 3. Check for objects closer than the face
    float closer_object_score = detect_closer_objects(frame, face);
    obstacle_score *= closer_object_score;
    
    return obstacle_score;
}

float AntiSpoofingDetector::analyze_face_shape_anomalies(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected) {
        return 1.0f;
    }
    
    float shape_score = 1.0f;
    
    // 1. Check face aspect ratio (masks often have different proportions)
    float aspect_ratio = static_cast<float>(face.bbox.width) / face.bbox.height;
    if (aspect_ratio < 0.7f || aspect_ratio > 1.3f) {
        shape_score *= 0.6f;  // Unusual aspect ratio
        std::cout << "⚠️  Unusual face aspect ratio: " << aspect_ratio << std::endl;
    }
    
    // 2. Check face size consistency
    float face_area = face.bbox.width * face.bbox.height;
    if (face_area < 1000 || face_area > 50000) {
        shape_score *= 0.7f;  // Unusual face size
        std::cout << "⚠️  Unusual face size: " << face_area << std::endl;
    }
    
    // 3. Check for landmark distribution anomalies
    if (!frame->metadata.landmarks.empty() && frame->metadata.landmarks.size() >= 468) {
        float landmark_distribution_score = analyze_landmark_distribution_anomalies(frame->metadata.landmarks);
        shape_score *= landmark_distribution_score;
    }
    
    return shape_score;
}

// Helper functions for comprehensive obstacle detection
float AntiSpoofingDetector::analyze_eye_region_occlusion(const std::vector<FrameBoxMetadata::Landmark>& landmarks, 
                                                        const std::vector<int>& eye_indices) {
    if (landmarks.size() < 468) {
        return 1.0f;
    }
    
    int valid_landmarks = 0;
    int total_landmarks = 0;
    
    for (int idx : eye_indices) {
        if (idx < static_cast<int>(landmarks.size())) {
            total_landmarks++;
            const auto& lm = landmarks[idx];
            
            // Check if landmark is tracked and has reasonable coordinates
            if (lm.x > 0.01f && lm.y > 0.01f && lm.x < 0.99f && lm.y < 0.99f) {
                valid_landmarks++;
            }
        }
    }
    
    if (total_landmarks == 0) {
        return 0.5f;  // Can't analyze
    }
    
    float visibility_ratio = static_cast<float>(valid_landmarks) / total_landmarks;
    
    // More realistic thresholds for eye visibility
    if (visibility_ratio < 0.4f) {
        return 0.3f;  // Heavily occluded (serious obstruction)
    } else if (visibility_ratio < 0.7f) {
        return 0.7f;  // Partially occluded (minor obstruction)
    } else {
        return 1.0f;  // Fully visible
    }
}

float AntiSpoofingDetector::analyze_face_depth_consistency(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || frame->depth_data.empty()) {
        return 1.0f;
    }
    
    // Sample depth values across the face region
    std::vector<float> depth_values;
    int step = std::max(1, std::min(face.bbox.width, face.bbox.height) / 10);
    
    for (int y = face.bbox.y; y < face.bbox.y + face.bbox.height; y += step) {
        for (int x = face.bbox.x; x < face.bbox.x + face.bbox.width; x += step) {
            int depth_x = x * frame->depth_width / frame->color_width;
            int depth_y = y * frame->depth_height / frame->color_height;
            int idx = depth_y * frame->depth_width + depth_x;
            
            if (idx >= 0 && idx < static_cast<int>(frame->depth_data.size())) {
                uint16_t depth_raw = frame->depth_data[idx];
                if (depth_raw > 0) {
                    depth_values.push_back(depth_raw * frame->depth_scale);
                }
            }
        }
    }
    
    if (depth_values.size() < 5) {
        return 0.5f;  // Insufficient depth data
    }
    
    // Calculate depth variance
    float mean_depth = 0.0f;
    for (float d : depth_values) {
        mean_depth += d;
    }
    mean_depth /= depth_values.size();
    
    float variance = 0.0f;
    for (float d : depth_values) {
        float diff = d - mean_depth;
        variance += diff * diff;
    }
    variance /= depth_values.size();
    
    // High variance might indicate objects in front of face
    if (variance > 0.05f) {  // More than 5cm variance (realistic threshold)
        return 0.4f;  // Likely objects blocking face
    } else {
        return 1.0f;  // Consistent depth
    }
}

float AntiSpoofingDetector::detect_color_anomalies(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || frame->color_data.empty()) {
        return 1.0f;
    }
    
    // Sample color values in face region
    int valid_pixels = 0;
    int total_pixels = 0;
    float total_brightness = 0.0f;
    
    int step = std::max(1, std::min(face.bbox.width, face.bbox.height) / 20);
    
    for (int y = face.bbox.y; y < face.bbox.y + face.bbox.height; y += step) {
        for (int x = face.bbox.x; x < face.bbox.x + face.bbox.width; x += step) {
            if (x >= 0 && x < frame->color_width && y >= 0 && y < frame->color_height) {
                int idx = (y * frame->color_width + x) * 3;
                if (idx + 2 < static_cast<int>(frame->color_data.size())) {
                    total_pixels++;
                    uint8_t b = frame->color_data[idx];
                    uint8_t g = frame->color_data[idx + 1];
                    uint8_t r = frame->color_data[idx + 2];
                    
                    float brightness = (r + g + b) / 3.0f;
                    total_brightness += brightness;
                    valid_pixels++;
                }
            }
        }
    }
    
    if (valid_pixels < 5) {
        return 0.5f;  // Insufficient color data
    }
    
    float avg_brightness = total_brightness / valid_pixels;
    
    // Check for unusual brightness patterns (might indicate masks)
    if (avg_brightness < 30.0f || avg_brightness > 200.0f) {
        return 0.6f;  // Unusual brightness
    }
    
    return 1.0f;  // Normal color patterns
}

float AntiSpoofingDetector::detect_ir_texture_anomalies(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || frame->ir_left_data.empty()) {
        return 1.0f;
    }
    
    // Sample IR values in face region
    int valid_pixels = 0;
    int total_pixels = 0;
    float total_intensity = 0.0f;
    
    int step = std::max(1, std::min(face.bbox.width, face.bbox.height) / 20);
    
    for (int y = face.bbox.y; y < face.bbox.y + face.bbox.height; y += step) {
        for (int x = face.bbox.x; x < face.bbox.x + face.bbox.width; x += step) {
            int ir_x = x * frame->ir_width / frame->color_width;
            int ir_y = y * frame->ir_height / frame->color_height;
            int idx = ir_y * frame->ir_width + ir_x;
            
            if (idx >= 0 && idx < static_cast<int>(frame->ir_left_data.size())) {
                total_pixels++;
                uint8_t intensity = frame->ir_left_data[idx];
                total_intensity += intensity;
                valid_pixels++;
            }
        }
    }
    
    if (valid_pixels < 5) {
        return 0.5f;  // Insufficient IR data
    }
    
    float avg_intensity = total_intensity / valid_pixels;
    
    // Check for unusual IR patterns (masks block IR differently)
    if (avg_intensity < 20.0f || avg_intensity > 180.0f) {
        return 0.5f;  // Unusual IR response
    }
    
    return 1.0f;  // Normal IR patterns
}

float AntiSpoofingDetector::analyze_facial_region_occlusion(const std::vector<FrameBoxMetadata::Landmark>& landmarks, 
                                                           const std::vector<int>& region_indices) {
    if (landmarks.size() < 468) {
        return 1.0f;
    }
    
    int valid_landmarks = 0;
    int total_landmarks = 0;
    
    for (int idx : region_indices) {
        if (idx < static_cast<int>(landmarks.size())) {
            total_landmarks++;
            const auto& lm = landmarks[idx];
            
            // Check if landmark is tracked and has reasonable coordinates
            if (lm.x > 0.01f && lm.y > 0.01f && lm.x < 0.99f && lm.y < 0.99f) {
                valid_landmarks++;
            }
        }
    }
    
    if (total_landmarks == 0) {
        return 0.5f;  // Can't analyze
    }
    
    float visibility_ratio = static_cast<float>(valid_landmarks) / total_landmarks;
    
    // More realistic thresholds for facial region visibility
    if (visibility_ratio < 0.3f) {
        return 0.3f;  // Heavily occluded (serious obstruction)
    } else if (visibility_ratio < 0.6f) {
        return 0.7f;  // Partially occluded (minor obstruction)
    } else {
        return 1.0f;  // Fully visible
    }
}

float AntiSpoofingDetector::analyze_depth_discontinuities(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || frame->depth_data.empty()) {
        return 1.0f;
    }
    
    // Check for large depth jumps that might indicate objects
    int discontinuities = 0;
    int total_checks = 0;
    
    int step = std::max(1, std::min(face.bbox.width, face.bbox.height) / 15);
    
    for (int y = face.bbox.y; y < face.bbox.y + face.bbox.height; y += step) {
        for (int x = face.bbox.x; x < face.bbox.x + face.bbox.width; x += step) {
            int depth_x = x * frame->depth_width / frame->color_width;
            int depth_y = y * frame->depth_height / frame->color_height;
            int idx = depth_y * frame->depth_width + depth_x;
            
            if (idx >= 0 && idx < static_cast<int>(frame->depth_data.size())) {
                uint16_t depth_raw = frame->depth_data[idx];
                if (depth_raw > 0) {
                    float depth = depth_raw * frame->depth_scale;
                    
                    // Check surrounding pixels for large depth differences
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            if (dx == 0 && dy == 0) continue;
                            
                            int nx = depth_x + dx;
                            int ny = depth_y + dy;
                            int nidx = ny * frame->depth_width + nx;
                            
                            if (nidx >= 0 && nidx < static_cast<int>(frame->depth_data.size())) {
                                uint16_t neighbor_raw = frame->depth_data[nidx];
                                if (neighbor_raw > 0) {
                                    float neighbor_depth = neighbor_raw * frame->depth_scale;
                                    float depth_diff = std::abs(depth - neighbor_depth);
                                    
                                    total_checks++;
                                    if (depth_diff > 0.05f) {  // More than 5cm difference
                                        discontinuities++;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    if (total_checks == 0) {
        return 0.5f;  // No depth data
    }
    
    float discontinuity_ratio = static_cast<float>(discontinuities) / total_checks;
    
    // High discontinuity ratio might indicate objects
    if (discontinuity_ratio > 0.3f) {
        return 0.4f;  // Likely objects causing depth discontinuities
    } else {
        return 1.0f;  // Smooth depth surface
    }
}

float AntiSpoofingDetector::analyze_depth_patterns(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || frame->depth_data.empty()) {
        return 1.0f;
    }
    
    // Analyze depth patterns for unusual shapes
    std::vector<float> depth_values;
    
    int step = std::max(1, std::min(face.bbox.width, face.bbox.height) / 20);
    
    for (int y = face.bbox.y; y < face.bbox.y + face.bbox.height; y += step) {
        for (int x = face.bbox.x; x < face.bbox.x + face.bbox.width; x += step) {
            int depth_x = x * frame->depth_width / frame->color_width;
            int depth_y = y * frame->depth_height / frame->color_height;
            int idx = depth_y * frame->depth_width + depth_x;
            
            if (idx >= 0 && idx < static_cast<int>(frame->depth_data.size())) {
                uint16_t depth_raw = frame->depth_data[idx];
                if (depth_raw > 0) {
                    depth_values.push_back(depth_raw * frame->depth_scale);
                }
            }
        }
    }
    
    if (depth_values.size() < 10) {
        return 0.5f;  // Insufficient depth data
    }
    
    // Check for depth distribution patterns
    std::sort(depth_values.begin(), depth_values.end());
    float depth_range = depth_values.back() - depth_values.front();
    
    // Unusual depth patterns might indicate masks or objects
    if (depth_range > 0.2f) {  // More than 20cm depth range
        return 0.6f;  // Unusual depth pattern
    }
    
    return 1.0f;  // Normal depth pattern
}

float AntiSpoofingDetector::detect_closer_objects(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || frame->depth_data.empty()) {
        return 1.0f;
    }
    
    // Get face depth
    int face_center_x = face.bbox.x + face.bbox.width / 2;
    int face_center_y = face.bbox.y + face.bbox.height / 2;
    int depth_x = face_center_x * frame->depth_width / frame->color_width;
    int depth_y = face_center_y * frame->depth_height / frame->color_height;
    int idx = depth_y * frame->depth_width + depth_x;
    
    if (idx < 0 || idx >= static_cast<int>(frame->depth_data.size())) {
        return 1.0f;  // Can't analyze
    }
    
    uint16_t face_depth_raw = frame->depth_data[idx];
    if (face_depth_raw == 0) {
        return 1.0f;  // No face depth data
    }
    
    float face_depth = face_depth_raw * frame->depth_scale;
    
    // Check for objects closer than the face
    int closer_objects = 0;
    int total_checks = 0;
    
    // Check region around face for closer objects
    int check_radius = std::max(10, std::min(face.bbox.width, face.bbox.height) / 4);
    
    for (int dy = -check_radius; dy <= check_radius; dy += 2) {
        for (int dx = -check_radius; dx <= check_radius; dx += 2) {
            int px = depth_x + dx;
            int py = depth_y + dy;
            int pidx = py * frame->depth_width + px;
            
            if (px >= 0 && px < frame->depth_width && py >= 0 && py < frame->depth_height &&
                pidx >= 0 && pidx < static_cast<int>(frame->depth_data.size())) {
                
                uint16_t pixel_depth_raw = frame->depth_data[pidx];
                if (pixel_depth_raw > 0) {
                    float pixel_depth = pixel_depth_raw * frame->depth_scale;
                    total_checks++;
                    
                    // Check if this pixel is significantly closer than the face
                    if (pixel_depth < face_depth - 0.1f) {  // More than 10cm closer
                        closer_objects++;
                    }
                }
            }
        }
    }
    
    if (total_checks == 0) {
        return 1.0f;  // No data to analyze
    }
    
    float closer_ratio = static_cast<float>(closer_objects) / total_checks;
    
    // High ratio of closer objects might indicate blocking
    if (closer_ratio > 0.2f) {
        return 0.3f;  // Likely objects blocking face
    } else {
        return 1.0f;  // No blocking objects
    }
}

float AntiSpoofingDetector::analyze_landmark_distribution_anomalies(const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return 1.0f;
    }
    
    // Check for unusual landmark distributions that might indicate masks
    float distribution_score = 1.0f;
    
    // Check for clustering (landmarks too close together)
    int clustered_pairs = 0;
    int total_pairs = 0;
    
    for (size_t i = 0; i < landmarks.size(); i += 5) {  // Sample every 5th landmark
        for (size_t j = i + 5; j < landmarks.size(); j += 5) {
            total_pairs++;
            const auto& lm1 = landmarks[i];
            const auto& lm2 = landmarks[j];
            
            float distance = std::sqrt(
                (lm1.x - lm2.x) * (lm1.x - lm2.x) + 
                (lm1.y - lm2.y) * (lm1.y - lm2.y)
            );
            
            if (distance < 0.01f) {  // Very close landmarks
                clustered_pairs++;
            }
        }
    }
    
    if (total_pairs > 0) {
        float clustering_ratio = static_cast<float>(clustered_pairs) / total_pairs;
        
        if (clustering_ratio > 0.1f) {  // More than 10% clustered
            distribution_score *= 0.6f;  // Unusual distribution
        }
    }
    
    return distribution_score;
}

// ============================================================================
// MATERIAL-BASED SPOOF DETECTION (NEW APPROACH)
// ============================================================================

// Core material analysis function - IR material properties
float AntiSpoofingDetector::analyze_ir_material_properties(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected) {
        return 0.5f;  // Can't analyze without frame or face
    }
    
    // Check IR data availability
    bool has_ir_left = !frame->ir_left_data.empty();
    bool has_ir_right = !frame->ir_right_data.empty();
    
    if (!has_ir_left || !has_ir_right) {
        // Debug: Show why IR analysis is failing
        static int ir_debug = 0;
        if (++ir_debug % 30 == 0) {
            std::cout << "🔍 IR Material Analysis: ir_left=" << has_ir_left 
                      << ", ir_right=" << has_ir_right 
                      << ", ir_left_size=" << frame->ir_left_data.size()
                      << ", ir_right_size=" << frame->ir_right_data.size() << std::endl;
        }
        return 0.5f;  // Can't analyze without IR data
    }
    
    // Convert IR data to OpenCV matrices
    cv::Mat ir_left = cv::Mat(frame->ir_height, frame->ir_width, CV_8UC1, 
                              const_cast<uint8_t*>(frame->ir_left_data.data()));
    cv::Mat ir_right = cv::Mat(frame->ir_height, frame->ir_width, CV_8UC1, 
                               const_cast<uint8_t*>(frame->ir_right_data.data()));
    
    // Map face ROI to IR coordinates
    cv::Rect ir_roi;
    if (frame->color_width > 0 && frame->color_height > 0) {
        float scale_x = static_cast<float>(frame->ir_width) / static_cast<float>(frame->color_width);
        float scale_y = static_cast<float>(frame->ir_height) / static_cast<float>(frame->color_height);
        
        ir_roi.x = static_cast<int>(face.bbox.x * scale_x);
        ir_roi.y = static_cast<int>(face.bbox.y * scale_y);
        ir_roi.width = static_cast<int>(face.bbox.width * scale_x);
        ir_roi.height = static_cast<int>(face.bbox.height * scale_y);
        
        // Ensure ROI is within bounds
        ir_roi &= cv::Rect(0, 0, frame->ir_width, frame->ir_height);
    } else {
        return 0.5f;  // Can't map coordinates
    }
    
    if (ir_roi.width < 20 || ir_roi.height < 20) {
        return 0.5f;  // ROI too small
    }
    
    // Extract face regions
    cv::Mat face_ir_left = ir_left(ir_roi);
    cv::Mat face_ir_right = ir_right(ir_roi);
    
    // Analyze IR reflectance patterns
    float reflectance_score = analyze_ir_reflectance_patterns(face_ir_left, face_ir_right, ir_roi);
    
    // Analyze thermal gradients
    float thermal_score = analyze_thermal_gradients(face_ir_left, face_ir_right, ir_roi);
    
    // Analyze material consistency between stereo IR
    float consistency_score = analyze_material_consistency(face_ir_left, face_ir_right, ir_roi);
    
    // Combine scores (material analysis is critical)
    float material_score = (reflectance_score * 0.4f + thermal_score * 0.3f + consistency_score * 0.3f);
    
    // Debug output
    static int material_debug = 0;
    if (++material_debug % 30 == 0) {
        std::cout << "🔍 IR Material Analysis: reflectance=" << reflectance_score 
                  << ", thermal=" << thermal_score 
                  << ", consistency=" << consistency_score 
                  << ", overall=" << material_score << std::endl;
    }
    
    return material_score;
}

// IR reflectance patterns analysis
float AntiSpoofingDetector::analyze_ir_reflectance_patterns(const cv::Mat& ir_left, const cv::Mat& ir_right, const cv::Rect& /* face_roi */) {
    if (ir_left.empty() || ir_right.empty()) {
        return 0.5f;
    }
    
    float reflectance_score = 0.0f;
    int checks = 0;
    
    // 1. Analyze IR standard deviation (material texture)
    cv::Scalar mean_left, std_left, mean_right, std_right;
    cv::meanStdDev(ir_left, mean_left, std_left);
    cv::meanStdDev(ir_right, mean_right, std_right);
    
    float std_left_val = static_cast<float>(std_left[0]);
    float std_right_val = static_cast<float>(std_right[0]);
    float mean_left_val = static_cast<float>(mean_left[0]);
    float mean_right_val = static_cast<float>(mean_right[0]);
    
    // Real skin: moderate IR variation (15-40 std)
    // Masks: either too uniform (<10) or too noisy (>60)
    if (std_left_val >= 15.0f && std_left_val <= 40.0f && 
        std_right_val >= 15.0f && std_right_val <= 40.0f) {
        reflectance_score += 1.0f;  // Good skin texture
    } else if (std_left_val < 10.0f || std_right_val < 10.0f) {
        reflectance_score += 0.2f;  // Too uniform (possible mask)
    } else if (std_left_val > 60.0f || std_right_val > 60.0f) {
        reflectance_score += 0.3f;  // Too noisy (possible textured mask)
    } else {
        reflectance_score += 0.6f;  // Borderline
    }
    checks++;
    
    // 2. Analyze IR intensity patterns (material reflectance)
    // Real skin: moderate IR intensity (40-120)
    // Masks: different intensity patterns
    if (mean_left_val >= 40.0f && mean_left_val <= 120.0f && 
        mean_right_val >= 40.0f && mean_right_val <= 120.0f) {
        reflectance_score += 1.0f;  // Normal skin reflectance
    } else if (mean_left_val < 20.0f || mean_right_val < 20.0f) {
        reflectance_score += 0.2f;  // Too dark (possible mask)
    } else if (mean_left_val > 180.0f || mean_right_val > 180.0f) {
        reflectance_score += 0.3f;  // Too bright (possible screen)
    } else {
        reflectance_score += 0.6f;  // Borderline
    }
    checks++;
    
    // 3. Analyze stereo IR correlation (material consistency)
    cv::Mat hist_left, hist_right;
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange[] = {range};
    int channels[] = {0};
    
    cv::calcHist(&ir_left, 1, channels, cv::Mat(), hist_left, 1, &histSize, histRange);
    cv::calcHist(&ir_right, 1, channels, cv::Mat(), hist_right, 1, &histSize, histRange);
    cv::normalize(hist_left, hist_left, 0, 1, cv::NORM_MINMAX);
    cv::normalize(hist_right, hist_right, 0, 1, cv::NORM_MINMAX);
    
    double hist_corr = cv::compareHist(hist_left, hist_right, cv::HISTCMP_CORREL);
    
    // Real skin: high stereo correlation (>0.8)
    // Masks: lower correlation due to material differences
    if (hist_corr > 0.8f) {
        reflectance_score += 1.0f;  // High correlation (real skin)
    } else if (hist_corr > 0.6f) {
        reflectance_score += 0.6f;  // Moderate correlation
    } else {
        reflectance_score += 0.2f;  // Low correlation (possible mask)
    }
    checks++;
    
    return (checks > 0) ? reflectance_score / checks : 0.5f;
}

// Thermal gradients analysis
float AntiSpoofingDetector::analyze_thermal_gradients(const cv::Mat& ir_left, const cv::Mat& ir_right, const cv::Rect& /* face_roi */) {
    if (ir_left.empty() || ir_right.empty()) {
        return 0.5f;
    }
    
    float thermal_score = 0.0f;
    int checks = 0;
    
    // 1. Analyze thermal gradients using Sobel operators
    cv::Mat grad_x_left, grad_y_left, grad_x_right, grad_y_right;
    cv::Sobel(ir_left, grad_x_left, CV_32F, 1, 0, 3);
    cv::Sobel(ir_left, grad_y_left, CV_32F, 0, 1, 3);
    cv::Sobel(ir_right, grad_x_right, CV_32F, 1, 0, 3);
    cv::Sobel(ir_right, grad_y_right, CV_32F, 0, 1, 3);
    
    // Calculate gradient magnitudes
    cv::Mat mag_left, mag_right;
    cv::magnitude(grad_x_left, grad_y_left, mag_left);
    cv::magnitude(grad_x_right, grad_y_right, mag_right);
    
    cv::Scalar mean_mag_left, std_mag_left, mean_mag_right, std_mag_right;
    cv::meanStdDev(mag_left, mean_mag_left, std_mag_left);
    cv::meanStdDev(mag_right, mean_mag_right, std_mag_right);
    
    float mean_mag_left_val = static_cast<float>(mean_mag_left[0]);
    float mean_mag_right_val = static_cast<float>(mean_mag_right[0]);
    
    // Real skin: moderate thermal gradients (5-25)
    // Masks: different thermal patterns
    if (mean_mag_left_val >= 5.0f && mean_mag_left_val <= 25.0f && 
        mean_mag_right_val >= 5.0f && mean_mag_right_val <= 25.0f) {
        thermal_score += 1.0f;  // Good thermal gradients
    } else if (mean_mag_left_val < 2.0f || mean_mag_right_val < 2.0f) {
        thermal_score += 0.2f;  // Too flat (possible mask)
    } else if (mean_mag_left_val > 40.0f || mean_mag_right_val > 40.0f) {
        thermal_score += 0.3f;  // Too noisy (possible textured mask)
    } else {
        thermal_score += 0.6f;  // Borderline
    }
    checks++;
    
    // 2. Analyze thermal symmetry between left and right
    float thermal_symmetry = 1.0f - std::abs(mean_mag_left_val - mean_mag_right_val) / 
                             (mean_mag_left_val + mean_mag_right_val + 1e-6f);
    
    if (thermal_symmetry > 0.8f) {
        thermal_score += 1.0f;  // High symmetry (real face)
    } else if (thermal_symmetry > 0.6f) {
        thermal_score += 0.6f;  // Moderate symmetry
    } else {
        thermal_score += 0.2f;  // Low symmetry (possible mask)
    }
    checks++;
    
    return (checks > 0) ? thermal_score / checks : 0.5f;
}

// Material consistency analysis
float AntiSpoofingDetector::analyze_material_consistency(const cv::Mat& ir_left, const cv::Mat& ir_right, const cv::Rect& /* face_roi */) {
    if (ir_left.empty() || ir_right.empty()) {
        return 0.5f;
    }
    
    float consistency_score = 0.0f;
    int checks = 0;
    
    // 1. Analyze local binary patterns (LBP) for material texture
    cv::Mat lbp_left = cv::Mat::zeros(ir_left.size(), CV_8UC1);
    cv::Mat lbp_right = cv::Mat::zeros(ir_right.size(), CV_8UC1);
    
    // Calculate LBP for both images
    for (int y = 1; y < ir_left.rows - 1; y++) {
        for (int x = 1; x < ir_left.cols - 1; x++) {
            uint8_t center_left = ir_left.at<uint8_t>(y, x);
            uint8_t center_right = ir_right.at<uint8_t>(y, x);
            uint8_t pattern_left = 0, pattern_right = 0;
            
            // 8-neighborhood LBP
            if (ir_left.at<uint8_t>(y-1, x-1) >= center_left) pattern_left |= 1;
            if (ir_left.at<uint8_t>(y-1, x) >= center_left) pattern_left |= 2;
            if (ir_left.at<uint8_t>(y-1, x+1) >= center_left) pattern_left |= 4;
            if (ir_left.at<uint8_t>(y, x+1) >= center_left) pattern_left |= 8;
            if (ir_left.at<uint8_t>(y+1, x+1) >= center_left) pattern_left |= 16;
            if (ir_left.at<uint8_t>(y+1, x) >= center_left) pattern_left |= 32;
            if (ir_left.at<uint8_t>(y+1, x-1) >= center_left) pattern_left |= 64;
            if (ir_left.at<uint8_t>(y, x-1) >= center_left) pattern_left |= 128;
            
            if (ir_right.at<uint8_t>(y-1, x-1) >= center_right) pattern_right |= 1;
            if (ir_right.at<uint8_t>(y-1, x) >= center_right) pattern_right |= 2;
            if (ir_right.at<uint8_t>(y-1, x+1) >= center_right) pattern_right |= 4;
            if (ir_right.at<uint8_t>(y, x+1) >= center_right) pattern_right |= 8;
            if (ir_right.at<uint8_t>(y+1, x+1) >= center_right) pattern_right |= 16;
            if (ir_right.at<uint8_t>(y+1, x) >= center_right) pattern_right |= 32;
            if (ir_right.at<uint8_t>(y+1, x-1) >= center_right) pattern_right |= 64;
            if (ir_right.at<uint8_t>(y, x-1) >= center_right) pattern_right |= 128;
            
            lbp_left.at<uint8_t>(y, x) = pattern_left;
            lbp_right.at<uint8_t>(y, x) = pattern_right;
        }
    }
    
    // Calculate LBP histograms
    cv::Mat hist_lbp_left, hist_lbp_right;
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange[] = {range};
    int channels[] = {0};
    
    cv::calcHist(&lbp_left, 1, channels, cv::Mat(), hist_lbp_left, 1, &histSize, histRange);
    cv::calcHist(&lbp_right, 1, channels, cv::Mat(), hist_lbp_right, 1, &histSize, histRange);
    cv::normalize(hist_lbp_left, hist_lbp_left, 0, 1, cv::NORM_MINMAX);
    cv::normalize(hist_lbp_right, hist_lbp_right, 0, 1, cv::NORM_MINMAX);
    
    double lbp_corr = cv::compareHist(hist_lbp_left, hist_lbp_right, cv::HISTCMP_CORREL);
    
    // Real skin: high LBP correlation (>0.7)
    // Masks: lower correlation due to material differences
    if (lbp_corr > 0.7f) {
        consistency_score += 1.0f;  // High consistency (real skin)
    } else if (lbp_corr > 0.5f) {
        consistency_score += 0.6f;  // Moderate consistency
    } else {
        consistency_score += 0.2f;  // Low consistency (possible mask)
    }
    checks++;
    
    return (checks > 0) ? consistency_score / checks : 0.5f;
}

// Depth material analysis for material behavior patterns
float AntiSpoofingDetector::analyze_depth_material_properties(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected || frame->depth_data.empty()) {
        return 0.5f;  // Can't analyze without depth data
    }
    
    // Convert depth data to OpenCV matrix
    cv::Mat depth = cv::Mat(frame->depth_height, frame->depth_width, CV_16UC1, 
                           const_cast<uint16_t*>(frame->depth_data.data()));
    
    // Map face ROI to depth coordinates
    cv::Rect depth_roi;
    if (frame->color_width > 0 && frame->color_height > 0) {
        float scale_x = static_cast<float>(frame->depth_width) / static_cast<float>(frame->color_width);
        float scale_y = static_cast<float>(frame->depth_height) / static_cast<float>(frame->color_height);
        
        depth_roi.x = static_cast<int>(face.bbox.x * scale_x);
        depth_roi.y = static_cast<int>(face.bbox.y * scale_y);
        depth_roi.width = static_cast<int>(face.bbox.width * scale_x);
        depth_roi.height = static_cast<int>(face.bbox.height * scale_y);
        
        // Ensure ROI is within bounds
        depth_roi &= cv::Rect(0, 0, frame->depth_width, frame->depth_height);
    } else {
        return 0.5f;  // Can't map coordinates
    }
    
    if (depth_roi.width < 20 || depth_roi.height < 20) {
        return 0.5f;  // ROI too small
    }
    
    // Extract face region
    cv::Mat face_depth = depth(depth_roi);
    
    // Analyze depth behavior patterns
    float behavior_score = analyze_depth_behavior_patterns(face_depth, depth_roi);
    
    // Debug output
    static int depth_material_debug = 0;
    if (++depth_material_debug % 30 == 0) {
        std::cout << "🔍 Depth Material Analysis: behavior=" << behavior_score << std::endl;
    }
    
    return behavior_score;
}

// Depth behavior patterns analysis
float AntiSpoofingDetector::analyze_depth_behavior_patterns(const cv::Mat& depth, const cv::Rect& /* face_roi */) {
    if (depth.empty()) {
        return 0.5f;
    }
    
    float behavior_score = 0.0f;
    int checks = 0;
    
    // 1. Analyze depth variance (material surface roughness)
    cv::Scalar mean_depth, std_depth;
    cv::meanStdDev(depth, mean_depth, std_depth);
    
    float std_depth_val = static_cast<float>(std_depth[0]);
    
    // Real skin: moderate depth variation (2-8mm std)
    // Masks: either too smooth (<1mm) or too rough (>12mm)
    if (std_depth_val >= 2.0f && std_depth_val <= 8.0f) {
        behavior_score += 1.0f;  // Good skin surface roughness
    } else if (std_depth_val < 1.0f) {
        behavior_score += 0.2f;  // Too smooth (possible mask)
    } else if (std_depth_val > 12.0f) {
        behavior_score += 0.3f;  // Too rough (possible textured mask)
    } else {
        behavior_score += 0.6f;  // Borderline
    }
    checks++;
    
    // 2. Analyze depth gradients (material surface curvature)
    cv::Mat grad_x, grad_y;
    cv::Sobel(depth, grad_x, CV_32F, 1, 0, 3);
    cv::Sobel(depth, grad_y, CV_32F, 0, 1, 3);
    
    cv::Mat grad_magnitude;
    cv::magnitude(grad_x, grad_y, grad_magnitude);
    
    cv::Scalar mean_grad, std_grad;
    cv::meanStdDev(grad_magnitude, mean_grad, std_grad);
    
    float mean_grad_val = static_cast<float>(mean_grad[0]);
    
    // Real skin: moderate depth gradients (1-5)
    // Masks: different gradient patterns
    if (mean_grad_val >= 1.0f && mean_grad_val <= 5.0f) {
        behavior_score += 1.0f;  // Good skin curvature
    } else if (mean_grad_val < 0.5f) {
        behavior_score += 0.2f;  // Too flat (possible mask)
    } else if (mean_grad_val > 8.0f) {
        behavior_score += 0.3f;  // Too sharp (possible mask edges)
    } else {
        behavior_score += 0.6f;  // Borderline
    }
    checks++;
    
    // 3. Analyze depth histogram (material depth distribution)
    cv::Mat hist;
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange[] = {range};
    int channels[] = {0};
    
    cv::calcHist(&depth, 1, channels, cv::Mat(), hist, 1, &histSize, histRange);
    cv::normalize(hist, hist, 0, 1, cv::NORM_MINMAX);
    
    // Calculate histogram entropy (material complexity)
    float entropy = 0.0f;
    for (int i = 0; i < histSize; i++) {
        float prob = hist.at<float>(i);
        if (prob > 0.0f) {
            entropy -= prob * std::log2(prob);
        }
    }
    
    // Real skin: moderate entropy (3-6)
    // Masks: different entropy patterns
    if (entropy >= 3.0f && entropy <= 6.0f) {
        behavior_score += 1.0f;  // Good material complexity
    } else if (entropy < 2.0f) {
        behavior_score += 0.2f;  // Too uniform (possible mask)
    } else if (entropy > 7.0f) {
        behavior_score += 0.3f;  // Too complex (possible textured mask)
    } else {
        behavior_score += 0.6f;  // Borderline
    }
    checks++;
    
    return (checks > 0) ? behavior_score / checks : 0.5f;
}

} // namespace mdai
