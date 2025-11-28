#pragma once

#include "FrameBox.hpp"
#include <memory>
#include <vector>
#include <string>
#include <deque>

#ifdef HAVE_OPENCV
#include <opencv2/opencv.hpp>
#endif

// MediaPipe GPU face detection via Python bridge
// Uses the working Python MediaPipe installation
#include <stdio.h>
#include <memory>

namespace mdai {

// Face detection result
struct FaceROI {
    bool detected = false;
    cv::Rect bbox;
    std::vector<cv::Point2f> landmarks;
    float confidence = 0.0f;
};

// Helper function for color conversion
void rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b, float& h, float& s, float& v);

/**
 * @brief Configuration for anti-spoofing algorithms
 */
struct AntiSpoofingConfig {
    // Quality Gate Thresholds (will be adaptive in future)
    float min_lighting_score = 0.3f;
    float min_motion_score = 0.5f;
    float min_positioning_score = 0.7f;
    float min_synchronization_score = 0.8f;
    float min_stability_score = 0.6f;
    float min_overall_quality = 0.6f;
    
    // Anti-Spoofing Thresholds - STRICT to prevent spoofs
    float min_depth_analysis_score = 0.65f;  // Require strong 3D structure
    float min_ir_texture_score = 0.5f;       // Require skin-like IR patterns
    float min_temporal_consistency_score = 0.5f;
    float min_cross_modal_score = 0.55f;
    float min_overall_liveness = 0.7f;       // Combined signal
    float min_confidence = 0.75f;            // Agreement
    
    // Temporal Analysis
    int temporal_window_size = 10;  // Number of frames for temporal analysis
    float max_temporal_variance = 0.3f;
    
    // Depth Analysis
    float min_face_depth_variance = 0.01f;  // Minimum depth variation for real face
    float max_flat_surface_threshold = 0.005f;  // Maximum flatness for real face
    
    // IR Analysis
    float min_ir_texture_variance = 0.1f;  // Minimum IR texture variation
    float max_ir_reflection_threshold = 0.8f;  // Maximum IR reflection for real skin
    
    // Cross-Modal Validation
    float min_sensor_agreement = 0.85f;  // Minimum agreement between sensors
    float max_cross_modal_variance = 0.2f;  // Maximum variance between modalities
};

/**
 * @brief Quality Gate System - First line of defense
 * 
 * Validates input quality before anti-spoofing analysis
 */
class QualityGate {
public:
    QualityGate(const AntiSpoofingConfig& config);
    
    /**
     * @brief Process quality gates for a single frame
     * @param frame Input frame to analyze
     * @return true if quality gates passed, false otherwise
     */
    bool process_frame(FrameBox* frame);
    
    /**
     * @brief Get current configuration
     */
    const AntiSpoofingConfig& get_config() const { return config_; }
    
    /**
     * @brief Update configuration (for adaptive thresholds)
     */
    void update_config(const AntiSpoofingConfig& config) { config_ = config; }

    /**
     * @brief Detect face in frame using modern face detector
     * @return FaceROI with detection results
     */
    FaceROI detect_face(const FrameBox* frame);

private:
    AntiSpoofingConfig config_;
    
#ifdef HAVE_OPENCV
    // Face detection handled externally - no internal detector
    
    // Motion analysis state
    cv::Mat previous_gray_;
    std::deque<cv::Mat> motion_frame_history_;
    static constexpr int MOTION_HISTORY_SIZE = 5;
#endif
    
    // Temporal buffer for motion analysis
    std::deque<FaceROI> face_history_;
    static constexpr int HISTORY_SIZE = 30;  // 1 second at 30fps
    
    // Quality gate implementations (now ROI-driven) - FIXED NAMES
    float analyze_lighting_quality(const FrameBox* frame, const FaceROI& face);
    float analyze_sharpness(const FrameBox* frame, const FaceROI& face);  // RENAMED from motion_stability
    float analyze_motion_stability(const FrameBox* frame, const FaceROI& face);  // NEW: real motion via optical flow
    float analyze_face_positioning(const FrameBox* frame, const FaceROI& face);
    float analyze_sensor_synchronization(const FrameBox* frame);
    float analyze_camera_stability(const FrameBox* frame);
    
    // Helper methods
    std::string generate_quality_issues(const FrameBox* frame);
};

/**
 * @brief Anti-Spoofing Algorithm - Core detection system
 * 
 * Multi-modal approach using depth, IR, temporal, and cross-modal analysis
 */
class AntiSpoofingDetector {
public:
    AntiSpoofingDetector(const AntiSpoofingConfig& config);
    
    /**
     * @brief Process anti-spoofing analysis for a single frame
     * @param frame Input frame to analyze
     * @return true if frame is determined to be live, false if spoofed
     */
    bool process_frame(FrameBox* frame);
    
    /**
     * @brief Process temporal analysis across multiple frames
     * @param frames Vector of recent frames for temporal analysis
     * @param current_frame Current frame to analyze
     * @return temporal consistency score
     */
    float process_temporal_analysis(const std::vector<FrameBox*>& frames, FrameBox* current_frame);
    
    /**
     * @brief Get current configuration
     */
    const AntiSpoofingConfig& get_config() const { return config_; }
    
    /**
     * @brief Update configuration (for adaptive thresholds)
     */
    void update_config(const AntiSpoofingConfig& config) { config_ = config; }

private:
    AntiSpoofingConfig config_;
    
    // Temporal buffer
    std::deque<FrameBox*> frame_history_;
    std::deque<FaceROI> face_history_;
    static constexpr int TEMPORAL_WINDOW = 30;  // 1 second at 30fps
    
    // rPPG (remote photoplethysmography) for pulse detection - ROBUST VERSION
    struct RPPGSample {
        double timestamp;      // Frame timestamp in seconds
        float red;            // Mean red channel
        float green;          // Mean green channel  
        float blue;           // Mean blue channel
        float motion_score;   // Motion stability (0-1, 1=stable)
        int landmark_count;   // Number of valid landmarks in ROI
    };
    std::deque<RPPGSample> rppg_samples_;  // RGB values over time with metadata
    static constexpr int RPPG_MIN_WINDOW = 150;   // Minimum 5 seconds at 30fps
    static constexpr int RPPG_MAX_WINDOW = 300;   // Maximum 10 seconds at 30fps
    double rppg_first_timestamp_ = 0.0;            // For FPS calculation
    
    // Core detection algorithms (now ROI-driven)
    float analyze_depth_geometry(const FrameBox* frame, const FaceROI& face);
    float analyze_ir_texture(const FrameBox* frame, const FaceROI& face);  // Now includes stereo consistency
    float analyze_cross_modal_consistency(const FrameBox* frame, const FaceROI& face);
    
    // rPPG pulse detection (mask detection) - ROBUST VERSION
    float analyze_rppg_pulse(const FrameBox* frame, const FaceROI& face);
    
    // Robust rPPG helper methods
    bool extract_rppg_signal_chrom(std::vector<float>& chrom_signal, float& estimated_fps);
    float calculate_snr_at_frequency(const std::vector<float>& signal, float fps, float target_hz);
    bool detect_pulse_fft(const std::vector<float>& signal, float fps, float& detected_bpm, float& confidence);
    float calculate_motion_compensation_weight(const RPPGSample& sample);
    
    // Temporal liveness checks
    float calculate_micro_motion();
    float analyze_depth_breathing(const FrameBox* frame, const FaceROI& face);
    
    // Material analysis for mask detection
    float analyze_material_properties(const FrameBox* frame, const FaceROI& face);
    
    // Occlusion and obstacle detection
    float analyze_occlusion(const FrameBox* frame, const FaceROI& face);
    
    // Attack type detection
    std::string detect_attack_type(const FrameBox* frame, const FaceROI& face);
    
    // Helper methods
    std::string generate_rejection_reason(const FrameBox* frame);
    float calculate_confidence(const FrameBox* frame, const FaceROI& face);
    
    // IMPROVED: Comprehensive facial landmark analysis
    float analyze_facial_landmarks(const FrameBox* frame, const FaceROI& face);
    float analyze_3d_facial_structure(const FrameBox* frame, const FaceROI& face);
    
    // Detailed landmark analysis functions
    float analyze_eye_region(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float analyze_single_eye(const std::vector<FrameBoxMetadata::Landmark>& landmarks, const std::vector<int>& eye_indices);
    float calculate_eye_aspect_ratio(const std::vector<FrameBoxMetadata::Landmark>& landmarks, const std::vector<int>& eye_indices);
    float analyze_mouth_region(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float analyze_nose_region(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float analyze_facial_symmetry(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float analyze_landmark_consistency(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    
    // 3D structure validation functions
    float validate_landmark_depth_consistency(const FrameBox* frame);
    float analyze_facial_curvature(const FrameBox* frame);
    float validate_eye_socket_depth(const FrameBox* frame);
    float validate_nose_bridge_structure(const FrameBox* frame);
    
    // Helper functions
    float get_depth_at_landmark(const FrameBox* frame, const FrameBoxMetadata::Landmark& landmark);
    float get_surrounding_depth(const FrameBox* frame, const FrameBoxMetadata::Landmark& landmark);
    
    // Symmetry analysis functions
    float check_eye_symmetry(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float check_mouth_symmetry(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float check_overall_symmetry(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    
    // IMPROVED: Enhanced eye analysis functions
    float analyze_iris_detection(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float analyze_eye_openness(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float analyze_eye_symmetry(const std::vector<FrameBoxMetadata::Landmark>& landmarks,
                              const std::vector<int>& left_indices,
                              const std::vector<int>& right_indices);
    float analyze_eye_iris_region(const std::vector<FrameBoxMetadata::Landmark>& landmarks,
                                 const std::vector<int>& eye_center_indices);
    float calculate_eye_aspect_ratio_detailed(const std::vector<FrameBoxMetadata::Landmark>& landmarks,
                                             const std::vector<int>& eye_indices);
    
    // CRITICAL: Depth-landmark fusion validation functions
    float validate_depth_landmark_fusion(const FrameBox* frame, const FaceROI& face);
    float validate_landmark_depth_correspondence(const FrameBox* frame, 
                                               const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float validate_depth_landmark_consistency(const FrameBox* frame,
                                             const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float detect_template_landmark_failures(const FrameBox* frame,
                                          const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float validate_3d_landmark_structure(const FrameBox* frame,
                                       const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    
    // Helper functions for depth-landmark fusion
    float analyze_landmark_distribution(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float analyze_depth_landmark_mismatch(const FrameBox* frame,
                                         const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float analyze_template_symmetry(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float validate_eye_socket_3d_structure(const FrameBox* frame,
                                          const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float validate_nose_bridge_3d_structure(const FrameBox* frame,
                                           const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float validate_facial_curvature_3d(const FrameBox* frame,
                                       const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    
    // CRITICAL: Comprehensive temporal analysis functions
    float analyze_micro_motion_temporal(const std::vector<FrameBox*>& frames);
    float analyze_depth_breathing_temporal(const std::vector<FrameBox*>& frames);
    float analyze_blink_patterns_temporal(const std::vector<FrameBox*>& frames);
    float analyze_landmark_temporal_consistency(const std::vector<FrameBox*>& frames);
    float validate_realsense_data_quality(const FrameBox* frame);
    float analyze_depth_temporal_consistency(const std::vector<FrameBox*>& frames);
    
    // Helper functions for temporal analysis
    float analyze_breathing_pattern_fft(const std::vector<float>& depth_values);
    float analyze_blink_frequency(const std::vector<float>& ear_values);
    
    // CRITICAL: Enhanced face obstacle detection functions
    float detect_eye_occlusion(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float detect_face_blocking_objects(const FrameBox* frame, const FaceROI& face);
    float analyze_landmark_occlusion(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    float detect_depth_obstacles(const FrameBox* frame, const FaceROI& face);
    float analyze_face_shape_anomalies(const FrameBox* frame, const FaceROI& face);
    
    // Helper functions for comprehensive obstacle detection
    float analyze_eye_region_occlusion(const std::vector<FrameBoxMetadata::Landmark>& landmarks, 
                                      const std::vector<int>& eye_indices);
    float analyze_face_depth_consistency(const FrameBox* frame, const FaceROI& face);
    float detect_color_anomalies(const FrameBox* frame, const FaceROI& face);
    float detect_ir_texture_anomalies(const FrameBox* frame, const FaceROI& face);
    float analyze_facial_region_occlusion(const std::vector<FrameBoxMetadata::Landmark>& landmarks, 
                                         const std::vector<int>& region_indices);
    float analyze_depth_discontinuities(const FrameBox* frame, const FaceROI& face);
    float analyze_depth_patterns(const FrameBox* frame, const FaceROI& face);
    float detect_closer_objects(const FrameBox* frame, const FaceROI& face);
    float analyze_landmark_distribution_anomalies(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    
    // MATERIAL-BASED SPOOF DETECTION (NEW APPROACH)
    // Core material analysis functions
    float analyze_ir_material_properties(const FrameBox* frame, const FaceROI& face);
    float analyze_depth_material_properties(const FrameBox* frame, const FaceROI& face);
    float analyze_thermal_patterns(const FrameBox* frame, const FaceROI& face);
    float analyze_stereo_ir_material(const FrameBox* frame, const FaceROI& face);
    float analyze_depth_ir_correlation(const FrameBox* frame, const FaceROI& face);
    float analyze_temporal_material_properties(const FrameBox* frame, const FaceROI& face);
    
    // Material analysis helper functions
    float analyze_ir_reflectance_patterns(const cv::Mat& ir_left, const cv::Mat& ir_right, const cv::Rect& face_roi);
    float analyze_depth_behavior_patterns(const cv::Mat& depth, const cv::Rect& face_roi);
    float analyze_thermal_gradients(const cv::Mat& ir_left, const cv::Mat& ir_right, const cv::Rect& face_roi);
    float analyze_material_consistency(const cv::Mat& ir_left, const cv::Mat& ir_right, const cv::Rect& face_roi);
    float analyze_material_correlation(const cv::Mat& depth, const cv::Mat& ir_left, const cv::Mat& ir_right, const cv::Rect& face_roi);
    float analyze_material_temporal_consistency(const std::vector<FrameBox*>& recent_frames, const FrameBox* current_frame, const FaceROI& face);
};

} // namespace mdai
