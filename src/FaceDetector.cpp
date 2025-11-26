#include "FaceDetector.hpp"
#include <iostream>

#ifdef HAVE_OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#endif

// MediaPipe Face Mesh wrapper (468 points)
#include "face_mesh_wrapper.h"

namespace mdai {

// ============================================================================
// MediaPipe Face Mesh Detector Implementation (468 points)
// ============================================================================

class MediaPipeFaceDetector::Impl {
public:
    Impl() : initialized_(false) {
        // Configure MediaPipe Face Mesh
        // Note: model_path is not used by the Python MediaPipe wrapper
        // The wrapper uses Python's mediapipe.solutions.face_mesh.FaceMesh directly
        FaceMeshConfig config;
        config.num_faces = 1;
        config.min_detection_confidence = 0.5f;
        config.min_tracking_confidence = 0.5f;
        config.min_presence_confidence = 0.5f;
        config.use_gpu = true;  // Enable GPU acceleration (MediaPipe Python handles this automatically)
        
        detector_ = std::make_unique<FaceMeshDetector>(config);
        initialized_ = detector_->IsInitialized();
        
        if (initialized_) {
            std::cout << "✓ MediaPipe Face Mesh (468 points) initialized successfully" << std::endl;
        } else {
            std::cerr << "❌ Failed to initialize MediaPipe Face Mesh: " 
                      << detector_->GetLastError() << std::endl;
            std::cerr << "   Make sure MediaPipe Python is installed: pip3 install mediapipe" << std::endl;
        }
    }
    
    ~Impl() = default;
    
    bool detect_faces(FrameBox* frame) {
        if (!initialized_ || !frame) {
            return false;
        }
        
        cv::Mat color_mat = frame->get_color_mat();
        if (color_mat.empty()) {
            return false;
        }
        
        // Rotate image for MediaPipe (camera is mounted 90° CCW, so rotate 90° CW)
        // MediaPipe expects upright faces for reliable detection
        cv::Mat rotated;
        cv::rotate(color_mat, rotated, cv::ROTATE_90_CLOCKWISE);
        
        // Detect face mesh on rotated image
        FaceMeshResult face_mesh;
        bool success = detector_->Detect(rotated, face_mesh);
        
        // Reset face detection metadata
        frame->metadata.face_detected = false;
        frame->metadata.face_detection_confidence = 0.0f;
        frame->metadata.landmarks.clear();
        
        if (!success) {
            // Detection failed
            return false;
        }
        
        // Check if face was detected
        if (face_mesh.landmarks.empty() || face_mesh.confidence < 0.3f) {
            return true;  // No face, but detection succeeded
        }
        
        // Check for occlusion
        if (face_mesh.IsOccluded(0.5f, 0.3f)) {
            // More than 30% of landmarks occluded - reject
            frame->metadata.face_detected = false;
            frame->metadata.face_rejection_reason = "Face partially occluded";
            return true;
        }
        
        // Valid face detected
        frame->metadata.face_detected = true;
        frame->metadata.face_detection_confidence = face_mesh.confidence;
        
        // Transform coordinates back from rotated space to original frame
        // Rotated image: 480x848 (original 848x480 rotated 90° CW)
        // Both landmarks and bbox are in PIXEL coordinates of rotated image
        // Inverse transform (90° CCW): (rx, ry) -> (ry, rotated_width - rx)
        int rotated_width = rotated.cols;   // 480
        
        // Transform bbox (pixel coords in rotated space 480x848)
        // rotated (x, y, w, h) -> original (y, rotated_width - x - w, h, w)
        frame->metadata.face_x = face_mesh.bbox.y;                              // orig_x = rotated_y
        frame->metadata.face_y = rotated_width - face_mesh.bbox.x - face_mesh.bbox.width;  // orig_y
        frame->metadata.face_w = face_mesh.bbox.height;                         // orig_w = rotated_h
        frame->metadata.face_h = face_mesh.bbox.width;                          // orig_h = rotated_w
        
        // Store all 468 landmarks with coordinate transform
        // Landmarks are in PIXEL coords of rotated image (480x848)
        // Transform: orig_x = ry, orig_y = rotated_width - rx
        frame->metadata.landmarks.reserve(face_mesh.landmarks.size());
        for (const auto& lm : face_mesh.landmarks) {
            float orig_x = lm.y;                        // rotated_y -> orig_x
            float orig_y = rotated_width - lm.x;        // rotated_width - rotated_x -> orig_y
            frame->metadata.landmarks.emplace_back(orig_x, orig_y, lm.z);
        }
        
        return true;
    }
    
    bool is_initialized() const { return initialized_; }
    
private:
    std::unique_ptr<FaceMeshDetector> detector_;
    bool initialized_;
};

MediaPipeFaceDetector::MediaPipeFaceDetector() 
    : impl_(std::make_unique<Impl>()) {
    initialized_ = impl_->is_initialized();
}

MediaPipeFaceDetector::~MediaPipeFaceDetector() = default;

bool MediaPipeFaceDetector::detect(FrameBox* frame) {
    return impl_->detect_faces(frame);
}

// ============================================================================
// OpenCV DNN Face Detector Implementation
// ============================================================================

#ifdef HAVE_OPENCV

OpenCVDNNFaceDetector::OpenCVDNNFaceDetector(const std::string& model_path, 
                                             const std::string& config_path) {
    // Use default Caffe ResNet-SSD face detector if paths not provided
    std::string model = model_path.empty() ? 
        "models/res10_300x300_ssd_iter_140000.caffemodel" : model_path;
    std::string config = config_path.empty() ? 
        "models/deploy_ssd.prototxt" : config_path;
    
    try {
        cv::dnn::Net* net = new cv::dnn::Net(cv::dnn::readNetFromCaffe(config, model));
        
        // Auto-detect best backend (CUDA on Jetson/GPU, CPU otherwise)
        bool cuda_available = false;
        
#if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 2
        // Check if CUDA is available
        try {
            net->setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            net->setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            cuda_available = true;
            std::cout << "✓ OpenCV DNN face detector initialized (CUDA GPU)" << std::endl;
        } catch (...) {
            cuda_available = false;
        }
#endif
        
        if (!cuda_available) {
            // Fallback to CPU with optimizations
            net->setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net->setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            std::cout << "✓ OpenCV DNN face detector initialized (CPU)" << std::endl;
        }
        
        net_ = net;
        initialized_ = true;
    } catch (const cv::Exception& e) {
        std::cerr << "Failed to load OpenCV DNN model: " << e.what() << std::endl;
        std::cerr << "Tried: " << config << " / " << model << std::endl;
        net_ = nullptr;
        initialized_ = false;
    }
}

bool OpenCVDNNFaceDetector::detect(FrameBox* frame) {
    if (!initialized_ || !frame || !net_) {
        return false;
    }
    
    cv::dnn::Net* net = static_cast<cv::dnn::Net*>(net_);
    cv::Mat color_mat = frame->get_color_mat();
    
    if (color_mat.empty()) {
        return false;
    }
    
    // Prepare input blob (ResNet-SSD expects 300x300)
    cv::Mat blob = cv::dnn::blobFromImage(
        color_mat, 
        1.0,                                    // scale
        cv::Size(300, 300),                     // size
        cv::Scalar(104.0, 177.0, 123.0),       // mean subtraction (BGR)
        false,                                  // swapRB (already BGR)
        false                                   // crop
    );
    
    net->setInput(blob);
    cv::Mat detections = net->forward();
    
    // Reset face detection metadata
    frame->metadata.face_detected = false;
    frame->metadata.face_detection_confidence = 0.0f;
    
    // Process detections
    // Output shape: [1, 1, N, 7] where N is number of detections
    // Each detection: [image_id, label, confidence, x1, y1, x2, y2]
    cv::Mat detection_mat(detections.size[2], detections.size[3], CV_32F, detections.ptr<float>());
    
    float best_confidence = 0.0f;
    int best_idx = -1;
    
    for (int i = 0; i < detection_mat.rows; i++) {
        float confidence = detection_mat.at<float>(i, 2);
        
        // Confidence threshold: 0.7 for high precision
        if (confidence > 0.7f && confidence > best_confidence) {
            best_confidence = confidence;
            best_idx = i;
        }
    }
    
    if (best_idx >= 0) {
        // Extract bbox (normalized coordinates 0-1)
        float x1_norm = detection_mat.at<float>(best_idx, 3);
        float y1_norm = detection_mat.at<float>(best_idx, 4);
        float x2_norm = detection_mat.at<float>(best_idx, 5);
        float y2_norm = detection_mat.at<float>(best_idx, 6);
        
        // Convert to pixel coordinates
        int x1 = static_cast<int>(x1_norm * color_mat.cols);
        int y1 = static_cast<int>(y1_norm * color_mat.rows);
        int x2 = static_cast<int>(x2_norm * color_mat.cols);
        int y2 = static_cast<int>(y2_norm * color_mat.rows);
        
        // Clamp to image boundaries
        x1 = std::max(0, std::min(x1, color_mat.cols - 1));
        y1 = std::max(0, std::min(y1, color_mat.rows - 1));
        x2 = std::max(0, std::min(x2, color_mat.cols - 1));
        y2 = std::max(0, std::min(y2, color_mat.rows - 1));
        
        frame->metadata.face_detected = true;
        frame->metadata.face_x = x1;
        frame->metadata.face_y = y1;
        frame->metadata.face_w = x2 - x1;
        frame->metadata.face_h = y2 - y1;
        frame->metadata.face_detection_confidence = best_confidence;
    }
    
    return true;
}

#else  // !HAVE_OPENCV

OpenCVDNNFaceDetector::OpenCVDNNFaceDetector(const std::string&, const std::string&) {
    std::cerr << "OpenCV not available" << std::endl;
    initialized_ = false;
}

bool OpenCVDNNFaceDetector::detect(FrameBox* frame) {
    (void)frame;
    return false;
}

#endif  // HAVE_OPENCV

// ============================================================================
// Factory Function
// ============================================================================

std::unique_ptr<FaceDetector> create_face_detector() {
    // Always use MediaPipe Face Mesh (468 points) with GPU acceleration
    auto mp_detector = std::make_unique<MediaPipeFaceDetector>();
    if (mp_detector->is_initialized()) {
        std::cout << "✓ Using MediaPipe Face Mesh (468 landmarks, GPU accelerated)" << std::endl;
        return mp_detector;
    }
    
    std::cerr << "❌ MediaPipe initialization failed, trying OpenCV DNN fallback..." << std::endl;

#ifdef HAVE_OPENCV
    auto cv_detector = std::make_unique<OpenCVDNNFaceDetector>();
    if (cv_detector->is_initialized()) {
        std::cout << "⚠ Using OpenCV DNN face detector (fallback)" << std::endl;
        return cv_detector;
    }
    std::cerr << "❌ OpenCV DNN initialization failed" << std::endl;
#endif

    std::cerr << "❌ No face detector available!" << std::endl;
    return nullptr;
}

} // namespace mdai

