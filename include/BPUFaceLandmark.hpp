/**
 * @file BPUFaceLandmark.hpp
 * @brief BPU-accelerated face landmark detection (468 points)
 * 
 * Uses Horizon BPU hardware acceleration for fast inference (~1ms)
 * Compatible with MediaPipe face mesh 468-landmark topology
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <atomic>

extern "C" {
#include "dnn/hb_dnn.h"
}

namespace mdai {

/**
 * @brief Single face landmark point with x, y, z coordinates
 */
struct FaceLandmarkPoint {
    float x;  // Normalized x coordinate (0-1) in face crop space
    float y;  // Normalized y coordinate (0-1) in face crop space
    float z;  // Depth coordinate (relative)
    
    FaceLandmarkPoint() : x(0), y(0), z(0) {}
    FaceLandmarkPoint(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

/**
 * @brief Face landmark detection result
 */
struct FaceLandmarkResult {
    bool detected = false;                        // Face detected and landmarks extracted
    float confidence = 0.0f;                      // Detection confidence
    std::vector<FaceLandmarkPoint> landmarks;     // 468 landmarks
    cv::Rect face_roi;                            // Face region in original image
    
    // Key landmark indices (MediaPipe face mesh topology)
    static constexpr int NOSE_TIP = 4;
    static constexpr int LEFT_EYE_INNER = 133;
    static constexpr int LEFT_EYE_OUTER = 33;
    static constexpr int RIGHT_EYE_INNER = 362;
    static constexpr int RIGHT_EYE_OUTER = 263;
    static constexpr int CHIN = 152;
    static constexpr int FOREHEAD = 10;
    static constexpr int LEFT_MOUTH = 61;
    static constexpr int RIGHT_MOUTH = 291;
    static constexpr int LEFT_EAR = 234;
    static constexpr int RIGHT_EAR = 454;
    
    // Helper methods to get key landmarks in original image coordinates
    cv::Point2f get_nose_tip() const;
    cv::Point2f get_left_eye() const;
    cv::Point2f get_right_eye() const;
    cv::Point2f get_chin() const;
    cv::Point2f get_forehead() const;
    
    // Get landmark in original image coordinates
    cv::Point2f get_landmark_point(int index) const;
    
    // Calculate face metrics
    float get_face_width() const;   // In pixels
    float get_yaw_ratio() const;    // 0 = facing forward, 1 = profile
    float get_pitch_ratio() const;  // 0 = level, 1 = extreme tilt
    float get_roll_degrees() const; // Head tilt angle
    
    // Validation
    bool is_face_valid(float min_distance_cm = 25.0f, float max_distance_cm = 60.0f) const;
};

/**
 * @brief BPU-accelerated face landmark detector
 * 
 * Uses Horizon RDK X5 BPU for hardware-accelerated inference
 * Input: 192x192 RGB face crop
 * Output: 468 landmarks (MediaPipe compatible)
 */
class BPUFaceLandmark {
public:
    /**
     * @brief Constructor
     * @param model_path Path to the BPU .bin model file
     */
    explicit BPUFaceLandmark(const std::string& model_path = 
        "/home/mercleDev/codebase/models/face_landmark_bpu.bin");
    
    /**
     * @brief Destructor
     */
    ~BPUFaceLandmark();
    
    // Non-copyable
    BPUFaceLandmark(const BPUFaceLandmark&) = delete;
    BPUFaceLandmark& operator=(const BPUFaceLandmark&) = delete;
    
    /**
     * @brief Check if model is initialized
     */
    bool is_initialized() const { return initialized_.load(); }
    
    /**
     * @brief Detect landmarks in a face crop
     * @param face_crop 192x192 BGR face crop
     * @param original_roi ROI in original image (for coordinate mapping)
     * @return FaceLandmarkResult with 468 landmarks
     */
    FaceLandmarkResult detect(const cv::Mat& face_crop, const cv::Rect& original_roi);
    
    /**
     * @brief Detect landmarks in full image with automatic face detection
     * @param image Full BGR image
     * @param face_roi Output: detected face ROI
     * @return FaceLandmarkResult (empty if no face detected)
     */
    FaceLandmarkResult detect_in_image(const cv::Mat& image, cv::Rect& face_roi);
    
    /**
     * @brief Get average inference time in milliseconds
     */
    float get_avg_inference_time_ms() const { return avg_inference_time_ms_.load(); }
    
    /**
     * @brief Get model name
     */
    std::string get_model_name() const { return model_name_; }
    
    /**
     * @brief Draw landmarks on image
     * @param image Image to draw on (modified in place)
     * @param result Landmark detection result
     * @param color Landmark color (default green)
     */
    static void draw_landmarks(cv::Mat& image, const FaceLandmarkResult& result,
                               cv::Scalar color = cv::Scalar(0, 255, 0));

private:
    std::string model_path_;
    std::string model_name_;
    std::atomic<bool> initialized_{false};
    std::atomic<float> avg_inference_time_ms_{0.0f};
    
    // BPU handles
    hbPackedDNNHandle_t packed_handle_ = nullptr;
    hbDNNHandle_t dnn_handle_ = nullptr;
    
    // Input/output tensors
    hbDNNTensor input_tensor_;
    hbDNNTensor* output_tensors_ = nullptr;
    int32_t output_count_ = 0;
    
    // Input dimensions
    static constexpr int INPUT_WIDTH = 192;
    static constexpr int INPUT_HEIGHT = 192;
    static constexpr int NUM_LANDMARKS = 468;
    
    /**
     * @brief Initialize BPU model
     */
    bool initialize();
    
    /**
     * @brief Allocate tensors
     */
    bool allocate_tensors();
    
    /**
     * @brief Free tensors
     */
    void free_tensors();
    
    /**
     * @brief Preprocess image for model input
     */
    void preprocess(const cv::Mat& input, uint8_t* output_buffer);
    
    /**
     * @brief Parse model output to landmarks
     */
    FaceLandmarkResult parse_output(const cv::Rect& original_roi);
};

} // namespace mdai



