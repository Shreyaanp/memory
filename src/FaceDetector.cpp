#include "FaceDetector.hpp"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#ifdef HAVE_OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#endif

// ONNX Runtime headers
#include <onnxruntime_cxx_api.h>

namespace mdai {

// ============================================================================
// ONNX Runtime 68-Landmark Face Detector Implementation
// ============================================================================

class ONNXFaceLandmarkDetector::Impl {
public:
    Impl(const std::string& model_path, bool use_gpu, int input_size)
        : model_path_(model_path)
        , use_gpu_(use_gpu)
        , input_size_(input_size)
        , initialized_(false)
    {
        try {
            // Initialize ONNX Runtime environment
            env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "FaceLandmarkDetector");
            
            // Create session options
            Ort::SessionOptions session_options;
            session_options.SetIntraOpNumThreads(4);
            session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            
            // Enable CUDA provider if requested
            if (use_gpu_) {
                try {
                    OrtCUDAProviderOptions cuda_options{};
                    cuda_options.device_id = 0;
                    // cuda_options.cudnn_conv_algo_search = 1;  // Default search (commented out for compatibility)
                    cuda_options.gpu_mem_limit = SIZE_MAX;
                    cuda_options.arena_extend_strategy = 0;
                    cuda_options.do_copy_in_default_stream = 1;
                    
                    session_options.AppendExecutionProvider_CUDA(cuda_options);
                    std::cout << "✓ ONNX Runtime CUDA provider enabled" << std::endl;
                } catch (const Ort::Exception& e) {
                    std::cerr << "Failed to enable CUDA provider: " << e.what() << std::endl;
                    std::cerr << "Falling back to CPU" << std::endl;
                    use_gpu_ = false;
                }
            }
            
            // Load model
            try {
#ifdef _WIN32
                std::wstring model_path_w(model_path_.begin(), model_path_.end());
                session_ = std::make_unique<Ort::Session>(*env_, model_path_w.c_str(), session_options);
#else
                session_ = std::make_unique<Ort::Session>(*env_, model_path_.c_str(), session_options);
#endif
                
                // Get input/output info
                Ort::AllocatorWithDefaultOptions allocator;
                
                // Input info
                size_t num_input_nodes = session_->GetInputCount();
                if (num_input_nodes > 0) {
                    auto input_name_allocated = session_->GetInputNameAllocated(0, allocator);
                    input_name_ = std::string(input_name_allocated.get());
                    auto input_type_info = session_->GetInputTypeInfo(0);
                    auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
                    input_shape_ = input_tensor_info.GetShape();
                    
                    // Update input size if model specifies it
                    if (input_shape_.size() >= 3 && input_shape_[2] > 0) {
                        input_size_ = static_cast<int>(input_shape_[2]);
                    }
                }
                
                // Output info
                size_t num_output_nodes = session_->GetOutputCount();
                if (num_output_nodes > 0) {
                    auto output_name_allocated = session_->GetOutputNameAllocated(0, allocator);
                    output_name_ = std::string(output_name_allocated.get());
                }
                
                initialized_ = true;
                
                std::cout << "✓ ONNX 68-landmark face detector initialized" << std::endl;
                std::cout << "  Model: " << model_path_ << std::endl;
                std::cout << "  Input size: " << input_size_ << "x" << input_size_ << std::endl;
                std::cout << "  Backend: " << (use_gpu_ ? "CUDA GPU" : "CPU") << std::endl;
                
            } catch (const Ort::Exception& e) {
                std::cerr << "Failed to load ONNX model: " << e.what() << std::endl;
                std::cerr << "Model path: " << model_path_ << std::endl;
                initialized_ = false;
            }
            
#ifdef HAVE_OPENCV
            // DISABLED HAAR CASCADE - it was blocking landmark detection
            // Process full image or use temporal tracking instead
            use_predetection_ = false;
            std::cout << "✓ Direct landmark detection (no Haar pre-filtering)" << std::endl;
#endif
            
        } catch (const std::exception& e) {
            std::cerr << "Failed to initialize ONNX face detector: " << e.what() << std::endl;
            initialized_ = false;
        }
    }
    
    ~Impl() = default;
    
    bool detect_landmarks(FrameBox* frame) {
        if (!initialized_ || !frame) {
            return false;
        }
        
#ifdef HAVE_OPENCV
        cv::Mat color_mat = frame->get_color_mat();
        if (color_mat.empty()) {
            return false;
        }
        
        // Reset face detection metadata
        frame->metadata.face_detected = false;
        frame->metadata.landmarks.valid = false;
        
        // Step 1: Pre-detect face region (optional, for performance)
        cv::Rect face_roi;
        if (use_predetection_) {
            std::vector<cv::Rect> faces;
            cv::Mat gray;
            cv::cvtColor(color_mat, gray, cv::COLOR_BGR2GRAY);
            // More lenient detection: scale=1.05 (was 1.1), neighbors=2 (was 3), minSize=30 (was 60)
            face_cascade_.detectMultiScale(gray, faces, 1.05, 2, 0, cv::Size(30, 30));
            
            if (faces.empty()) {
                // No face detected by pre-detector
                static int no_face_count = 0;
                if (++no_face_count % 90 == 0) {  // Print every 90 frames
                    std::cerr << "⚠️ Haar cascade not finding faces (checked " << no_face_count << " frames)" << std::endl;
                }
                return true;  // Not an error, just no face
            }
            
            static int face_count = 0;
            if (++face_count % 30 == 0) {  // Print every 30 frames
                std::cout << "✓ Haar cascade found face #" << face_count << " → Running ONNX landmark detection..." << std::endl;
            }
            
            // Use largest face
            face_roi = *std::max_element(faces.begin(), faces.end(),
                [](const cv::Rect& a, const cv::Rect& b) {
                    return a.area() < b.area();
                });
            
            // Expand ROI by 30% for better landmark detection
            int expand_x = static_cast<int>(face_roi.width * 0.15);
            int expand_y = static_cast<int>(face_roi.height * 0.15);
            face_roi.x = std::max(0, face_roi.x - expand_x);
            face_roi.y = std::max(0, face_roi.y - expand_y);
            face_roi.width = std::min(color_mat.cols - face_roi.x, face_roi.width + 2 * expand_x);
            face_roi.height = std::min(color_mat.rows - face_roi.y, face_roi.height + 2 * expand_y);
            
        } else {
            // Use full image
            face_roi = cv::Rect(0, 0, color_mat.cols, color_mat.rows);
        }
        
        // Step 2: Prepare input for ONNX model
        cv::Mat face_crop = color_mat(face_roi).clone();
        cv::Mat input_image;
        cv::resize(face_crop, input_image, cv::Size(input_size_, input_size_));
        
        // Convert to float and normalize (0-255 -> 0-1)
        input_image.convertTo(input_image, CV_32FC3, 1.0 / 255.0);
        
        // Convert to CHW format (model expects channels-first)
        std::vector<cv::Mat> channels(3);
        cv::split(input_image, channels);
        
        std::vector<float> input_tensor_values;
        input_tensor_values.reserve(3 * input_size_ * input_size_);
        
        for (auto& channel : channels) {
            input_tensor_values.insert(input_tensor_values.end(),
                                      (float*)channel.data,
                                      (float*)channel.data + input_size_ * input_size_);
        }
        
        // Create input tensor
        std::vector<int64_t> input_shape = {1, 3, input_size_, input_size_};
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            input_tensor_values.data(),
            input_tensor_values.size(),
            input_shape.data(),
            input_shape.size()
        );
        
        // Step 3: Run inference
        const char* input_names[] = {input_name_.c_str()};
        const char* output_names[] = {output_name_.c_str()};
        
        try {
            auto output_tensors = session_->Run(
                Ort::RunOptions{nullptr},
                input_names, &input_tensor, 1,
                output_names, 1
            );
            
            // Step 4: Parse landmarks
            float* output_data = output_tensors[0].GetTensorMutableData<float>();
            auto output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
            
            // FAN models output heatmaps: [1, 68, H, W] where each landmark has an H×W probability map
            int num_landmarks = 68;
            
            if (output_shape.size() == 4 && output_shape[1] == 68) {
                // Shape: [1, 68, H, W] - Heatmap format (FAN, HRNet, etc.)
                int heatmap_h = output_shape[2];
                int heatmap_w = output_shape[3];
                
                for (int i = 0; i < num_landmarks; i++) {
                    // Find max activation in this landmark's heatmap
                    int offset = i * heatmap_h * heatmap_w;
                    float max_val = -1e9;
                    int max_y = 0, max_x = 0;
                    
                    for (int y = 0; y < heatmap_h; y++) {
                        for (int x = 0; x < heatmap_w; x++) {
                            float val = output_data[offset + y * heatmap_w + x];
                            if (val > max_val) {
                                max_val = val;
                                max_y = y;
                                max_x = x;
                            }
                        }
                    }
                    
                    // Convert heatmap coordinates to image coordinates
                    float norm_x = static_cast<float>(max_x) / heatmap_w;
                    float norm_y = static_cast<float>(max_y) / heatmap_h;
                    
                    // Scale to face crop coordinates, then to full image
                    float img_x = norm_x * face_crop.cols + face_roi.x;
                    float img_y = norm_y * face_crop.rows + face_roi.y;
                    
                    landmark_x_[i] = img_x;
                    landmark_y_[i] = img_y;
                }
            } else if (output_shape.size() == 3 && output_shape[1] == 68 && output_shape[2] == 2) {
                // Shape: [1, 68, 2] - Direct coordinates
                for (int i = 0; i < num_landmarks; i++) {
                    float x = output_data[i * 2 + 0];
                    float y = output_data[i * 2 + 1];
                    
                    // Scale to face ROI coordinates
                    x = x * face_crop.cols + face_roi.x;
                    y = y * face_crop.rows + face_roi.y;
                    
                    landmark_x_[i] = x;
                    landmark_y_[i] = y;
                }
            } else if (output_shape.size() == 2 && output_shape[1] == 136) {
                // Shape: [1, 136] (interleaved x, y, x, y, ...)
                for (int i = 0; i < num_landmarks; i++) {
                    float x = output_data[i * 2 + 0];
                    float y = output_data[i * 2 + 1];
                    
                    // Scale to face ROI coordinates
                    x = x * face_crop.cols + face_roi.x;
                    y = y * face_crop.rows + face_roi.y;
                    
                    landmark_x_[i] = x;
                    landmark_y_[i] = y;
                }
            } else {
                std::cerr << "Unexpected output shape for landmarks: [";
                for (size_t i = 0; i < output_shape.size(); i++) {
                    std::cerr << output_shape[i];
                    if (i < output_shape.size() - 1) std::cerr << ", ";
                }
                std::cerr << "]" << std::endl;
                return false;
            }
            
            // Step 5: Populate FrameBox metadata
            frame->metadata.landmarks.set_all_landmarks(landmark_x_, landmark_y_);
            frame->metadata.face_detected = true;
            frame->metadata.face_x = face_roi.x;
            frame->metadata.face_y = face_roi.y;
            frame->metadata.face_w = face_roi.width;
            frame->metadata.face_h = face_roi.height;
            frame->metadata.face_detection_confidence = 1.0f;  // ONNX models don't provide confidence
            
            // DEBUG: Verify landmarks were set
            static int landmark_success_count = 0;
            if (++landmark_success_count % 30 == 0) {
                std::cout << "✅ Landmarks extracted successfully! (frame " << landmark_success_count << ")" << std::endl;
                std::cout << "   Face ROI: " << face_roi.width << "x" << face_roi.height << std::endl;
                std::cout << "   Sample landmarks - Nose tip: (" << landmark_x_[30] << ", " << landmark_y_[30] << ")" << std::endl;
                std::cout << "   Metadata landmarks.valid: " << frame->metadata.landmarks.valid << std::endl;
            }
            
            return true;
            
        } catch (const Ort::Exception& e) {
            static int error_count = 0;
            if (++error_count % 10 == 0) {
                std::cerr << "❌ ONNX inference error #" << error_count << ": " << e.what() << std::endl;
            }
            return false;
        } catch (const std::exception& e) {
            static int error_count2 = 0;
            if (++error_count2 % 10 == 0) {
                std::cerr << "❌ Landmark extraction error #" << error_count2 << ": " << e.what() << std::endl;
            }
            return false;
        }
#else
        std::cerr << "OpenCV not available - cannot process frames" << std::endl;
        return false;
#endif
    }
    
    bool is_initialized() const { return initialized_; }
    bool is_using_gpu() const { return use_gpu_; }
    
private:
    std::string model_path_;
    int input_size_;
    bool use_gpu_;
    bool initialized_;
    bool use_predetection_ = false;
    
    // ONNX Runtime members
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    std::vector<int64_t> input_shape_;
    std::string input_name_;
    std::string output_name_;
    
    // Temporary storage for landmarks
    float landmark_x_[68];
    float landmark_y_[68];
    
#ifdef HAVE_OPENCV
    cv::CascadeClassifier face_cascade_;
#endif
};

// ============================================================================
// ONNXFaceLandmarkDetector Public Interface
// ============================================================================

ONNXFaceLandmarkDetector::ONNXFaceLandmarkDetector(
    const std::string& model_path,
    bool use_gpu,
    int input_size)
    : impl_(std::make_unique<Impl>(model_path, use_gpu, input_size))
{
    initialized_ = impl_->is_initialized();
    use_gpu_ = impl_->is_using_gpu();
}

ONNXFaceLandmarkDetector::~ONNXFaceLandmarkDetector() = default;

bool ONNXFaceLandmarkDetector::detect(FrameBox* frame) {
    return impl_->detect_landmarks(frame);
}

std::string ONNXFaceLandmarkDetector::name() const {
    return std::string("ONNX 68-Landmark (") + (use_gpu_ ? "GPU" : "CPU") + ")";
}

// ============================================================================
// Factory Function
// ============================================================================

std::unique_ptr<FaceDetector> create_face_detector(const std::string& model_path) {
    // Try to create ONNX detector
    auto detector = std::make_unique<ONNXFaceLandmarkDetector>(model_path, true, 256);
    
    if (detector->is_initialized()) {
        std::cout << "Using " << detector->name() << " face detector" << std::endl;
        return detector;
    }
    
    std::cerr << "Failed to initialize face detector!" << std::endl;
    std::cerr << "Please ensure model file exists at: " << model_path << std::endl;
    return nullptr;
}

} // namespace mdai
