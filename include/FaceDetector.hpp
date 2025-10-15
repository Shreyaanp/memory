#pragma once

#include "FrameBox.hpp"
#include <memory>
#include <string>

namespace mdai {

/**
 * @brief Abstract interface for face detection
 * 
 * This allows different face detection backends (MediaPipe, OpenCV DNN, etc.)
 * to be used interchangeably.
 */
class FaceDetector {
public:
    virtual ~FaceDetector() = default;
    
    /**
     * @brief Detect faces in a frame and populate metadata
     * @param frame Frame to process (will update metadata in-place)
     * @return true if detection succeeded (even if no faces found)
     */
    virtual bool detect(FrameBox* frame) = 0;
    
    /**
     * @brief Get detector name/type
     */
    virtual std::string name() const = 0;
};

/**
 * @brief MediaPipe GPU face detector
 * 
 * Uses MediaPipe's GPU-accelerated face detection.
 * Requires MediaPipe to be installed and built.
 */
class MediaPipeFaceDetector : public FaceDetector {
public:
    MediaPipeFaceDetector();
    ~MediaPipeFaceDetector() override;
    
    bool detect(FrameBox* frame) override;
    std::string name() const override { return "MediaPipe GPU"; }
    
    bool is_initialized() const { return initialized_; }

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    bool initialized_ = false;
};

/**
 * @brief OpenCV DNN face detector (fallback)
 * 
 * Uses OpenCV's DNN module with a pre-trained model (e.g., ResNet-SSD).
 * Slower than MediaPipe GPU but widely compatible.
 */
class OpenCVDNNFaceDetector : public FaceDetector {
public:
    OpenCVDNNFaceDetector(const std::string& model_path = "", 
                           const std::string& config_path = "");
    ~OpenCVDNNFaceDetector() override = default;
    
    bool detect(FrameBox* frame) override;
    std::string name() const override { return "OpenCV DNN"; }
    
    bool is_initialized() const { return initialized_; }

private:
    bool initialized_ = false;
#ifdef HAVE_OPENCV
    void* net_;  // cv::dnn::Net pointer (forward declaration to avoid OpenCV dependency in header)
#endif
};

/**
 * @brief Factory function to create the best available face detector
 * 
 * Priority:
 * 1. MediaPipe GPU (if available)
 * 2. OpenCV DNN (fallback)
 * 3. nullptr (no detector available)
 */
std::unique_ptr<FaceDetector> create_face_detector();

} // namespace mdai

