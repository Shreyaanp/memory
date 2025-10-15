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
    
    // Core detection algorithms (now ROI-driven)
    float analyze_depth_geometry(const FrameBox* frame, const FaceROI& face);
    float analyze_ir_texture(const FrameBox* frame, const FaceROI& face);  // Now includes stereo consistency
    float analyze_temporal_consistency(const FaceROI& face);
    float analyze_cross_modal_consistency(const FrameBox* frame, const FaceROI& face);
    
    // Temporal liveness checks
    bool detect_blink(const std::vector<cv::Point2f>& landmarks);
    float calculate_micro_motion();
    float analyze_depth_breathing(const FrameBox* frame, const FaceROI& face);
    
    // Attack type detection
    std::string detect_attack_type(const FrameBox* frame, const FaceROI& face);
    
    // Helper methods
    std::string generate_rejection_reason(const FrameBox* frame);
    float calculate_confidence(const FrameBox* frame, const FaceROI& face);
};

/**
 * @brief Combined Quality Gate + Anti-Spoofing Pipeline
 * 
 * Orchestrates the complete validation pipeline
 */
class AntiSpoofingPipeline {
public:
    AntiSpoofingPipeline(const AntiSpoofingConfig& config);
    
    /**
     * @brief Process complete pipeline (quality gates + anti-spoofing)
     * @param frame Input frame to analyze
     * @return true if frame passes all checks, false otherwise
     */
    bool process_frame(FrameBox* frame);
    
    /**
     * @brief Process with temporal context
     * @param frame Current frame
     * @param recent_frames Recent frames for temporal analysis
     * @return true if frame passes all checks
     */
    bool process_frame_with_temporal_context(FrameBox* frame, const std::vector<FrameBox*>& recent_frames);
    
    /**
     * @brief Get pipeline statistics
     */
    struct PipelineStats {
        uint64_t total_frames_processed = 0;
        uint64_t quality_gate_passed = 0;
        uint64_t quality_gate_failed = 0;
        uint64_t anti_spoofing_passed = 0;
        uint64_t anti_spoofing_failed = 0;
        uint64_t overall_passed = 0;
        uint64_t overall_failed = 0;
        
        float quality_gate_pass_rate = 0.0f;
        float anti_spoofing_pass_rate = 0.0f;
        float overall_pass_rate = 0.0f;
    };
    
    PipelineStats get_stats() const { return stats_; }
    void reset_stats() { stats_ = PipelineStats{}; }

private:
    AntiSpoofingConfig config_;
    std::unique_ptr<QualityGate> quality_gate_;
    std::unique_ptr<AntiSpoofingDetector> anti_spoofing_detector_;
    PipelineStats stats_;
    
    // Statistics tracking
    void update_stats(bool quality_passed, bool anti_spoofing_passed, bool overall_passed);
};

/**
 * @brief Adaptive threshold manager (placeholder for future implementation)
 * 
 * Will learn optimal thresholds based on environmental conditions and user behavior
 */
class AdaptiveThresholdManager {
public:
    AdaptiveThresholdManager();
    
    /**
     * @brief Get current adaptive thresholds
     * @param base_config Base configuration
     * @param environmental_factors Current environmental conditions
     * @return Adapted configuration
     */
    AntiSpoofingConfig get_adaptive_config(
        const AntiSpoofingConfig& base_config,
        const std::map<std::string, float>& environmental_factors
    );
    
    /**
     * @brief Learn from processing results
     * @param frame Processed frame
     * @param success Whether processing was successful
     */
    void learn_from_result(const FrameBox* frame, bool success);
    
    /**
     * @brief Get learning statistics
     */
    struct LearningStats {
        uint64_t total_samples = 0;
        uint64_t successful_samples = 0;
        float success_rate = 0.0f;
        std::map<std::string, float> environmental_correlations;
    };
    
    LearningStats get_learning_stats() const { return learning_stats_; }

private:
    LearningStats learning_stats_;
    
    // Future implementation will include:
    // - Machine learning models for threshold optimization
    // - Environmental sensor integration
    // - User behavior pattern recognition
    // - Real-time threshold adjustment
};

} // namespace mdai
