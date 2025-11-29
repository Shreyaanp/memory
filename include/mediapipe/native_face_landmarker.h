/**
 * Native MediaPipe Face Landmarker Wrapper for RDK X5
 * 
 * Replaces Python-based wrapper with native C++ implementation.
 * Uses the MediaPipe Tasks C API (libface_landmarker.so).
 * 
 * Performance: ~44 FPS (vs ~25 FPS with Python wrapper)
 */

#pragma once

#include <memory>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

namespace mdai {

/**
 * @brief Result structure containing 478 face landmarks
 */
struct NativeFaceMeshResult {
    // 478 3D landmarks in normalized coordinates [0-1]
    std::vector<cv::Point3f> landmarks;
    
    // Per-landmark visibility scores [0, 1]
    std::vector<float> visibility;
    
    // Per-landmark presence scores [0, 1]
    std::vector<float> presence;
    
    // Face bounding box in pixels
    cv::Rect bbox;
    
    // Overall face detection confidence
    float confidence;
    
    // Image dimensions
    int image_width;
    int image_height;
    
    /**
     * @brief Check if face is significantly occluded
     */
    bool IsOccluded(float visibility_threshold = 0.5f, 
                    float occlusion_ratio = 0.3f) const;
};

/**
 * @brief Configuration for native face landmarker
 */
struct NativeFaceLandmarkerConfig {
    // Path to face_landmarker.task model
    std::string model_path = "/home/mercleDev/mediapipe_arm64_final/models/face_landmarker.task";
    
    // Detection parameters
    int num_faces = 1;
    float min_face_detection_confidence = 0.5f;
    float min_face_presence_confidence = 0.5f;
    float min_tracking_confidence = 0.5f;
    
    // Output options
    bool output_face_blendshapes = false;
};

/**
 * @brief Native MediaPipe Face Landmarker (no Python!)
 * 
 * Uses libface_landmarker.so directly for 478-point face landmarks.
 * ~44 FPS on RDK X5.
 */
class NativeFaceLandmarker {
public:
    explicit NativeFaceLandmarker(const NativeFaceLandmarkerConfig& config = {});
    ~NativeFaceLandmarker();
    
    // Non-copyable
    NativeFaceLandmarker(const NativeFaceLandmarker&) = delete;
    NativeFaceLandmarker& operator=(const NativeFaceLandmarker&) = delete;
    
    /**
     * @brief Detect face landmarks in BGR image
     * @param bgr_image Input image (BGR format from OpenCV)
     * @param result Output result structure
     * @return true if detection succeeded (even if no face found)
     */
    bool Detect(const cv::Mat& bgr_image, NativeFaceMeshResult& result);
    
    /**
     * @brief Detect face landmarks for video frames (with tracking)
     * @param bgr_image Input image (BGR format)
     * @param timestamp_ms Frame timestamp in milliseconds
     * @param result Output result structure
     * @return true if detection succeeded
     */
    bool DetectForVideo(const cv::Mat& bgr_image, int64_t timestamp_ms, NativeFaceMeshResult& result);
    
    /**
     * @brief Check if detector is initialized
     */
    bool IsInitialized() const;
    
    /**
     * @brief Get last error message
     */
    std::string GetLastError() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mdai




