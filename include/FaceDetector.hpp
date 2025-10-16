#pragma once

#include "FrameBox.hpp"
#include <memory>
#include <string>

namespace mdai {

/**
 * @brief Abstract interface for face detection and landmark localization
 * 
 * This allows different backends (ONNX Runtime, OpenCV, etc.)
 * to be used interchangeably.
 */
class FaceDetector {
public:
    virtual ~FaceDetector() = default;
    
    /**
     * @brief Detect faces and landmarks in a frame
     * @param frame Frame to process (will update metadata in-place)
     * @return true if detection succeeded (even if no faces found)
     */
    virtual bool detect(FrameBox* frame) = 0;
    
    /**
     * @brief Get detector name/type
     */
    virtual std::string name() const = 0;
    
    /**
     * @brief Check if detector is initialized
     */
    virtual bool is_initialized() const = 0;
};

/**
 * @brief ONNX Runtime 68-landmark face detector
 * 
 * Uses ONNX Runtime with CUDA GPU acceleration to detect faces
 * and localize 68 facial landmarks (iBUG format).
 * 
 * Supports models like:
 * - PIPNet (Pixel-in-Pixel Network)
 * - 2D-FAN (Face Alignment Network)
 * - MobileNet-based face alignment
 * 
 * Performance: 100-150 FPS on GPU, 30-50 FPS on CPU
 */
class ONNXFaceLandmarkDetector : public FaceDetector {
public:
    /**
     * @brief Constructor
     * @param model_path Path to ONNX model file (.onnx)
     * @param use_gpu Whether to use CUDA GPU acceleration (default: true)
     * @param input_size Model input size (default: 256 for most models)
     */
    explicit ONNXFaceLandmarkDetector(
        const std::string& model_path = "models/face_landmarks_68.onnx",
        bool use_gpu = true,
        int input_size = 256
    );
    
    ~ONNXFaceLandmarkDetector() override;
    
    bool detect(FrameBox* frame) override;
    std::string name() const override;
    bool is_initialized() const override { return initialized_; }

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    bool initialized_ = false;
    bool use_gpu_ = true;
};

/**
 * @brief Factory function to create the best available face detector
 * 
 * Priority:
 * 1. ONNX Runtime 68-landmark detector (if model available)
 * 2. nullptr (no detector available)
 */
std::unique_ptr<FaceDetector> create_face_detector(
    const std::string& model_path = "models/face_landmarks_68.onnx"
);

} // namespace mdai
