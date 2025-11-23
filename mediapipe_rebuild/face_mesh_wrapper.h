#pragma once

#include <memory>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

namespace mdai {

/**
 * @brief Result structure containing 468 face landmarks
 */
struct FaceMeshResult {
    // 468 3D landmarks in pixel coordinates
    std::vector<cv::Point3f> landmarks;
    
    // Per-landmark visibility scores [0, 1]
    // Lower values indicate occlusion
    std::vector<float> visibility;
    
    // Per-landmark presence scores [0, 1]
    std::vector<float> presence;
    
    // Face bounding box in pixels
    cv::Rect bbox;
    
    // Overall face detection confidence
    float confidence;
    
    // Image dimensions (for coordinate conversion)
    int image_width;
    int image_height;
    
    /**
     * @brief MediaPipe 468-point landmark indices
     * Standard face mesh topology
     */
    struct LandmarkIndices {
        // Face oval (36 points)
        static const std::vector<int> FACE_OVAL;
        
        // Left eye (16 points)
        static const std::vector<int> LEFT_EYE;
        
        // Right eye (16 points)
        static const std::vector<int> RIGHT_EYE;
        
        // Lips outer (12 points)
        static const std::vector<int> LIPS_OUTER;
        
        // Lips inner (8 points)
        static const std::vector<int> LIPS_INNER;
        
        // Left eyebrow (8 points)
        static const std::vector<int> LEFT_EYEBROW;
        
        // Right eyebrow (8 points)
        static const std::vector<int> RIGHT_EYEBROW;
        
        // Nose tip and bridge (9 points)
        static const std::vector<int> NOSE;
        
        // Forehead region (for rPPG)
        static const std::vector<int> FOREHEAD;
        
        // Left cheek
        static const std::vector<int> LEFT_CHEEK;
        
        // Right cheek
        static const std::vector<int> RIGHT_CHEEK;
    };
    
    /**
     * @brief Check if face is significantly occluded
     * @param visibility_threshold Minimum visibility score to consider visible
     * @param occlusion_ratio Maximum ratio of occluded landmarks allowed
     * @return true if face is too occluded
     */
    bool IsOccluded(float visibility_threshold = 0.5f, 
                    float occlusion_ratio = 0.3f) const;
    
    /**
     * @brief Get landmarks for a specific region
     * @param indices Region indices (e.g., LEFT_EYE)
     * @return Vector of landmarks for that region
     */
    std::vector<cv::Point3f> GetRegionLandmarks(const std::vector<int>& indices) const;
    
    /**
     * @brief Check if landmark is valid (visible and within bounds)
     */
    bool IsLandmarkValid(int index, float min_visibility = 0.5f) const;
};

/**
 * @brief Configuration for face mesh detector
 */
struct FaceMeshConfig {
    // Model path
    std::string model_path = "realsense_integration/models/face_landmarker.task";
    
    // Detection parameters
    int num_faces = 1;
    float min_detection_confidence = 0.5f;
    float min_tracking_confidence = 0.5f;
    float min_presence_confidence = 0.5f;
    
    // GPU settings
    bool use_gpu = true;
    int gpu_device_id = 0;
    
    // Output options
    bool output_face_blendshapes = false;
    bool output_facial_transformation_matrixes = false;
};

/**
 * @brief MediaPipe Face Mesh Detector
 * 
 * Detects 468 face landmarks with visibility scores.
 * Thread-safe for single-threaded use.
 */
class FaceMeshDetector {
public:
    /**
     * @brief Constructor
     * @param config Configuration options
     */
    explicit FaceMeshDetector(const FaceMeshConfig& config);
    
    /**
     * @brief Destructor
     */
    ~FaceMeshDetector();
    
    /**
     * @brief Detect face mesh in RGB image
     * @param rgb_image Input image (BGR format from OpenCV)
     * @param result Output result structure (only valid if return is true)
     * @return true if detection succeeded (even if no face found)
     */
    bool Detect(const cv::Mat& rgb_image, FaceMeshResult& result);
    
    /**
     * @brief Check if detector is properly initialized
     * @return true if ready to use
     */
    bool IsInitialized() const;
    
    /**
     * @brief Get last error message
     */
    std::string GetLastError() const;
    
private:
    // PIMPL pattern to hide MediaPipe implementation details
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mdai

