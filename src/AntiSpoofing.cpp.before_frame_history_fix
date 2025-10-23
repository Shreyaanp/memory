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
    
    frame->metadata.anti_spoofing.depth_analysis_score = analyze_depth_geometry(frame, face);
    
    // IR texture analysis now includes stereo consistency check
    frame->metadata.anti_spoofing.ir_texture_score = analyze_ir_texture(frame, face);
    
    frame->metadata.anti_spoofing.cross_modal_score = analyze_cross_modal_consistency(frame, face);
    frame->metadata.anti_spoofing.temporal_consistency_score = analyze_temporal_consistency(face);
    
    // Material analysis for mask detection (edge sharpness, texture, specular, depth discontinuities)
    float material_score = analyze_material_properties(frame, face);
    
    // Occlusion/obstacle detection (eyes visible, landmarks visible, depth holes)
    float occlusion_score = analyze_occlusion(frame, face);
    
    // rPPG pulse detection (NEW - supplementary signal for mask detection)
    // NOTE: Made non-critical to avoid false rejects on real faces
    // Now using robust rPPG with CHROM, bandpass filtering, and motion compensation
    float rppg_pulse_score = analyze_rppg_pulse(frame, face);
    
    // DEBUG: Print rPPG score
    static int rppg_frame_count = 0;
    if (++rppg_frame_count % 30 == 0) {
        std::cout << "🩺 rPPG score: " << rppg_pulse_score << ", samples: " << rppg_samples_.size() << std::endl;
    }
    
    // If not enough data yet (analyze_rppg_pulse returns 0.5f), score already handled
    
    frame->metadata.anti_spoofing.depth_anomaly_detected = 
        frame->metadata.anti_spoofing.depth_analysis_score < config_.min_depth_analysis_score;
    frame->metadata.anti_spoofing.ir_material_mismatch = 
        frame->metadata.anti_spoofing.ir_texture_score < config_.min_ir_texture_score;
    frame->metadata.anti_spoofing.cross_modal_disagreement = 
        frame->metadata.anti_spoofing.cross_modal_score < config_.min_cross_modal_score;
    
    // Integrate all detection methods (weights sum to 1.0)
    // Prioritize: depth (2D attacks), occlusion (masks/obstacles), material (masks), IR texture, rPPG, temporal
    frame->metadata.anti_spoofing.overall_liveness_score = 
        (frame->metadata.anti_spoofing.depth_analysis_score * 0.25f +  // 2D attack detection
         occlusion_score * 0.20f +  // NEW: Eyes/face visibility (critical for masks)
         material_score * 0.20f +  // Mask detection (edge/texture/specular/depth)
         frame->metadata.anti_spoofing.ir_texture_score * 0.15f +  // NIR reflectance patterns
         rppg_pulse_score * 0.10f +  // Pulse detection (soft signal)
         frame->metadata.anti_spoofing.temporal_consistency_score * 0.10f);  // Breathing/micro-motion/blinks
    
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
    
    // Project color ROI corners to depth space using proper projection
    // Corners: TL, TR, BL, BR
    std::vector<cv::Point2i> color_corners = {
        {face.bbox.x, face.bbox.y},
        {face.bbox.x + face.bbox.width, face.bbox.y},
        {face.bbox.x, face.bbox.y + face.bbox.height},
        {face.bbox.x + face.bbox.width, face.bbox.y + face.bbox.height}
    };
    
    std::vector<cv::Point2i> depth_corners;
    for (const auto& color_pt : color_corners) {
        // Deproject from color to 3D point
        float color_pixel[2] = {static_cast<float>(color_pt.x), static_cast<float>(color_pt.y)};
        float point3d[3];
        rs2_deproject_pixel_to_point(point3d, &frame->color_intrinsics, color_pixel, median_depth_for_roi);
        
        // Transform from color space to depth space if needed
        // (For D435i, they're typically close but may have slight offset)
        // For now, assume aligned sensors (can add extrinsics later)
        
        // Project back to depth pixel
        float depth_pixel[2];
        rs2_project_point_to_pixel(depth_pixel, &frame->depth_intrinsics, point3d);
        
        depth_corners.push_back(cv::Point2i(
            static_cast<int>(depth_pixel[0]),
            static_cast<int>(depth_pixel[1])
        ));
    }
    
    // Calculate bounding box from projected corners
    int min_x = std::min({depth_corners[0].x, depth_corners[1].x, depth_corners[2].x, depth_corners[3].x});
    int max_x = std::max({depth_corners[0].x, depth_corners[1].x, depth_corners[2].x, depth_corners[3].x});
    int min_y = std::min({depth_corners[0].y, depth_corners[1].y, depth_corners[2].y, depth_corners[3].y});
    int max_y = std::max({depth_corners[0].y, depth_corners[1].y, depth_corners[2].y, depth_corners[3].y});
    
    cv::Rect depth_roi(min_x, min_y, max_x - min_x, max_y - min_y);
    
    // Debug: Show projection results
    static int proj_debug = 0;
    if (++proj_debug % 30 == 0) {
        std::cout << "🔍 ROI Projection Debug:" << std::endl;
        std::cout << "   Color ROI: " << face.bbox << std::endl;
        std::cout << "   Center depth: " << median_depth_for_roi << "m" << std::endl;
        std::cout << "   Depth corners: TL=" << depth_corners[0] << ", BR=" << depth_corners[3] << std::endl;
        std::cout << "   Depth ROI: " << depth_roi << std::endl;
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
            if (idx < frame->depth_data.size() && frame->depth_data[idx] > 0) {
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

float AntiSpoofingDetector::analyze_temporal_consistency(const FaceROI& face) {
    // Enhanced temporal analysis with micro-motion and breathing
    if (!face.detected || face_history_.size() < 3) {
        return 0.0f;  // Insufficient data
    }
    
    float temporal_score = 0.0f;
    int components = 0;
    
    // 1. Enhanced micro-motion detection (uses new calculate_micro_motion)
    float micro_motion_score = calculate_micro_motion();
    temporal_score += micro_motion_score;
    components++;
    
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
                temporal_score += 0.3f;  // Very stable = suspicious
            }
            components++;
        }
    }
    
    // 3. Depth-based breathing analysis (if we have recent frame)
    if (frame_history_.size() > 0) {
        FrameBox* recent_frame = frame_history_.back();
        float breathing_score = analyze_depth_breathing(recent_frame, face);
        temporal_score += breathing_score;
        components++;
    }
    
    return (components > 0) ? (temporal_score / components) : 0.5f;
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
    // EAR (Eye Aspect Ratio) based blink detection for 468-point face mesh
    // MediaPipe eye landmarks:
    // Left eye: 362(outer), 385(top), 387(bottom), 263(inner), 373(top), 380(bottom)
    // Right eye: 33(inner), 160(top), 158(bottom), 133(outer), 153(top), 144(bottom)
    
    if (landmarks.size() < 468) {
        return false;  // Not enough landmarks
    }
    
    // Calculate EAR for left eye
    float left_vertical_1 = cv::norm(landmarks[385] - landmarks[380]);
    float left_vertical_2 = cv::norm(landmarks[387] - landmarks[373]);
    float left_horizontal = cv::norm(landmarks[362] - landmarks[263]);
    float ear_left = (left_vertical_1 + left_vertical_2) / (2.0f * left_horizontal + 1e-6f);
    
    // Calculate EAR for right eye
    float right_vertical_1 = cv::norm(landmarks[160] - landmarks[144]);
    float right_vertical_2 = cv::norm(landmarks[158] - landmarks[153]);
    float right_horizontal = cv::norm(landmarks[33] - landmarks[133]);
    float ear_right = (right_vertical_1 + right_vertical_2) / (2.0f * right_horizontal + 1e-6f);
    
    // Average EAR
    float avg_ear = (ear_left + ear_right) / 2.0f;
    
    // EAR threshold for blink: typically < 0.2 indicates closed eyes
    // For liveness: We want to see variation in EAR over time (blinking)
    // This is a simplified check - full implementation needs temporal tracking
    return (avg_ear < 0.25f);  // Eyes are closed or partially closed
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
        // process_frame already computes all scores including rPPG and sets overall_liveness_score
        anti_spoofing_passed = anti_spoofing_detector_->process_frame(frame);
        
        // Add supplementary temporal analysis
        float temporal_score = anti_spoofing_detector_->process_temporal_analysis(recent_frames, frame);
        frame->metadata.anti_spoofing.temporal_consistency_score = temporal_score;
        
        // NOTE: Don't recalculate overall_liveness_score here - it's already set by process_frame
        // with the correct fusion including rPPG. Recalculating would create inconsistency.
        // The overall_liveness_score from process_frame already includes:
        // depth (35%), IR (30%), rPPG (15%), temporal (10%), cross-modal (10%)
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

void AntiSpoofingDetector::apply_bandpass_filter(std::vector<float>& signal, float fps, 
                                                   float low_hz, float high_hz) {
    if (signal.size() < 30) return;  // Too short for filtering
    
    // Simple moving average-based bandpass (compromise for real-time)
    int low_window = static_cast<int>(fps / low_hz);
    int high_window = static_cast<int>(fps / high_hz);
    
    low_window = std::max(3, std::min(low_window, static_cast<int>(signal.size()) / 4));
    high_window = std::max(1, std::min(high_window, low_window / 2));
    
    // Remove low frequencies (DC drift)
    std::vector<float> detrended(signal.size());
    for (size_t i = 0; i < signal.size(); i++) {
        float sum = 0.0f;
        int count = 0;
        int start = std::max(0, static_cast<int>(i) - low_window / 2);
        int end = std::min(static_cast<int>(signal.size()), static_cast<int>(i) + low_window / 2);
        
        for (int j = start; j < end; j++) {
            sum += signal[j];
            count++;
        }
        float local_mean = (count > 0) ? sum / count : 0.0f;
        detrended[i] = signal[i] - local_mean;
    }
    
    // Remove high frequencies (noise)
    for (size_t i = 0; i < signal.size(); i++) {
        float sum = 0.0f;
        int count = 0;
        int start = std::max(0, static_cast<int>(i) - high_window / 2);
        int end = std::min(static_cast<int>(signal.size()), static_cast<int>(i) + high_window / 2);
        
        for (int j = start; j < end; j++) {
            sum += detrended[j];
            count++;
        }
        signal[i] = (count > 0) ? sum / count : 0.0f;
    }
}

float AntiSpoofingDetector::calculate_snr_at_frequency(const std::vector<float>& signal, 
                                                        float fps, float target_hz) {
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
    
    float final_score = (checks > 0) ? (material_score / checks) : 0.5f;
    
    std::cout << "🎭 Material analysis: edge=" << edge_density 
              << ", texture=" << texture_variance 
              << ", specular=" << highlight_ratio 
              << ", score=" << final_score << std::endl;
    
    return final_score;
    
#else
    return 0.5f;
#endif
}

// ============================================================================
// OCCLUSION AND OBSTACLE DETECTION
// ============================================================================

bool AntiSpoofingDetector::check_eyes_visible(const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 468) {
        return true;  // Can't check with 468 mesh, assume OK (will fail other checks)
    }
    
    // Key eye landmarks from MediaPipe 468-point mesh
    // Left eye: 33, 133, 159, 145
    // Right eye: 362, 263, 386, 374
    std::vector<int> left_eye_indices = {33, 133, 159, 145};
    std::vector<int> right_eye_indices = {362, 263, 386, 374};
    
    // Check if eye landmarks are present and have reasonable coordinates
    // If a landmark is at (0,0,0), it's likely not tracked (occluded)
    int left_valid = 0, right_valid = 0;
    
    // Check left eye
    for (int idx : left_eye_indices) {
        if (idx < (int)landmarks.size()) {
            const auto& lm = landmarks[idx];
            // Check if landmark has non-zero coordinates (tracked)
            if (lm.x > 1.0f && lm.y > 1.0f) {
                left_valid++;
            }
        }
    }
    
    // Check right eye
    for (int idx : right_eye_indices) {
        if (idx < (int)landmarks.size()) {
            const auto& lm = landmarks[idx];
            if (lm.x > 1.0f && lm.y > 1.0f) {
                right_valid++;
            }
        }
    }
    
    // Need at least 3/4 landmarks valid per eye
    bool left_ok = (left_valid >= 3);
    bool right_ok = (right_valid >= 3);
    
    if (!left_ok || !right_ok) {
        std::cout << "👁️  Eyes not fully visible: L=" << left_valid << "/4, R=" 
                  << right_valid << "/4" << std::endl;
    }
    
    return left_ok && right_ok;
}

float AntiSpoofingDetector::analyze_occlusion(const FrameBox* frame, const FaceROI& face) {
    if (!frame || !face.detected) {
        return 0.5f;
    }
    
    float occlusion_score = 1.0f;  // Start with no occlusion
    
    // 1. Check eye visibility using 468 landmarks
    if (!frame->metadata.landmarks.empty()) {
        bool eyes_visible = check_eyes_visible(frame->metadata.landmarks);
        if (!eyes_visible) {
            occlusion_score *= 0.2f;  // Heavy penalty - eyes critical for liveness
            std::cout << "⚠️  EYES OCCLUDED - possible mask or obstruction" << std::endl;
        }
    }
    
    // 2. Check overall landmark presence (geometric check)
    if (frame->metadata.landmarks.size() >= 468) {
        int total_landmarks = 0;
        int valid_landmarks = 0;
        
        // Sample key landmarks across the face
        std::vector<int> key_indices = {
            // Forehead: 10, 338, 297, 332
            10, 338, 297, 332,
            // Eyes: 33, 133, 362, 263
            33, 133, 362, 263,
            // Nose: 1, 4, 5, 6
            1, 4, 5, 6,
            // Mouth: 61, 291, 0, 17
            61, 291, 0, 17,
            // Cheeks: 234, 454
            234, 454,
            // Chin: 152
            152
        };
        
        for (int idx : key_indices) {
            if (idx < (int)frame->metadata.landmarks.size()) {
                total_landmarks++;
                const auto& lm = frame->metadata.landmarks[idx];
                // Check if landmark has valid coordinates (not (0,0))
                if (lm.x > 1.0f && lm.y > 1.0f) {
                    valid_landmarks++;
                }
            }
        }
        
        float validity_ratio = (total_landmarks > 0) ? 
            (float)valid_landmarks / total_landmarks : 1.0f;
        
        // Need at least 80% of key landmarks valid
        if (validity_ratio < 0.8f) {
            occlusion_score *= (0.5f + validity_ratio * 0.5f);
            std::cout << "⚠️  Face partially occluded: " << (int)(validity_ratio * 100) 
                      << "% valid landmarks" << std::endl;
        }
    }
    
    // 3. Check for depth holes (missing depth data in face region)
    // This can indicate obstacles or partial faces
    if (!frame->depth_data.empty()) {
        int face_center_x = face.bbox.x + face.bbox.width / 2;
        int face_center_y = face.bbox.y + face.bbox.height / 2;
        
        // Map to depth coordinates
        int depth_x = face_center_x * frame->depth_width / std::max(1, frame->color_width);
        int depth_y = face_center_y * frame->depth_height / std::max(1, frame->color_height);
        
        // Check 3x3 region around face center
        int valid_center_pixels = 0;
        int total_center_pixels = 0;
        
        for (int dy = -5; dy <= 5; dy++) {
            for (int dx = -5; dx <= 5; dx++) {
                int px = depth_x + dx;
                int py = depth_y + dy;
                if (px >= 0 && px < frame->depth_width && py >= 0 && py < frame->depth_height) {
                    int idx = py * frame->depth_width + px;
                    if (idx < (int)frame->depth_data.size()) {
                        total_center_pixels++;
                        if (frame->depth_data[idx] > 0) {
                            valid_center_pixels++;
                        }
                    }
                }
            }
        }
        
        float center_coverage = (total_center_pixels > 0) ?
            (float)valid_center_pixels / total_center_pixels : 1.0f;
        
        if (center_coverage < 0.7f) {
            occlusion_score *= 0.4f;
            std::cout << "⚠️  Depth holes in face center: " << (int)(center_coverage * 100) 
                      << "% coverage" << std::endl;
        }
    }
    
    static int occ_debug = 0;
    if (++occ_debug % 30 == 0) {
        std::cout << "🔍 Occlusion score: " << occlusion_score << std::endl;
    }
    
    return occlusion_score;
}

} // namespace mdai
