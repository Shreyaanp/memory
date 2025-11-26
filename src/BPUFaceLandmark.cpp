/**
 * @file BPUFaceLandmark.cpp
 * @brief Implementation of BPU-accelerated face landmark detection
 */

#include "BPUFaceLandmark.hpp"
#include <iostream>
#include <chrono>
#include <cstring>

namespace mdai {

// =============================================================================
// FaceLandmarkResult Implementation
// =============================================================================

cv::Point2f FaceLandmarkResult::get_landmark_point(int index) const {
    if (index < 0 || index >= static_cast<int>(landmarks.size())) {
        return cv::Point2f(0, 0);
    }
    const auto& lm = landmarks[index];
    // Map from normalized coordinates to original image coordinates
    return cv::Point2f(
        face_roi.x + lm.x * face_roi.width,
        face_roi.y + lm.y * face_roi.height
    );
}

cv::Point2f FaceLandmarkResult::get_nose_tip() const {
    return get_landmark_point(NOSE_TIP);
}

cv::Point2f FaceLandmarkResult::get_left_eye() const {
    return get_landmark_point(LEFT_EYE_OUTER);
}

cv::Point2f FaceLandmarkResult::get_right_eye() const {
    return get_landmark_point(RIGHT_EYE_OUTER);
}

cv::Point2f FaceLandmarkResult::get_chin() const {
    return get_landmark_point(CHIN);
}

cv::Point2f FaceLandmarkResult::get_forehead() const {
    return get_landmark_point(FOREHEAD);
}

float FaceLandmarkResult::get_face_width() const {
    if (landmarks.size() < 468) return 0.0f;
    auto left_ear = get_landmark_point(LEFT_EAR);
    auto right_ear = get_landmark_point(RIGHT_EAR);
    return std::abs(right_ear.x - left_ear.x);
}

float FaceLandmarkResult::get_yaw_ratio() const {
    if (landmarks.size() < 468) return 0.0f;
    
    auto nose = get_nose_tip();
    auto left_eye = get_left_eye();
    auto right_eye = get_right_eye();
    
    float eye_center_x = (left_eye.x + right_eye.x) / 2.0f;
    float nose_offset = nose.x - eye_center_x;
    float eye_width = std::abs(right_eye.x - left_eye.x);
    
    return eye_width > 0 ? std::abs(nose_offset / eye_width) : 0.0f;
}

float FaceLandmarkResult::get_pitch_ratio() const {
    if (landmarks.size() < 468) return 0.0f;
    
    auto nose = get_nose_tip();
    auto forehead = get_forehead();
    auto chin = get_chin();
    
    float face_height = std::abs(forehead.y - chin.y);
    float nose_y_ratio = face_height > 0 ? (nose.y - forehead.y) / face_height : 0.5f;
    
    return std::abs(nose_y_ratio - 0.5f) * 2.0f;
}

float FaceLandmarkResult::get_roll_degrees() const {
    if (landmarks.size() < 468) return 0.0f;
    
    auto left_eye = get_left_eye();
    auto right_eye = get_right_eye();
    
    float dx = right_eye.x - left_eye.x;
    float dy = right_eye.y - left_eye.y;
    
    return std::atan2(dy, dx) * 180.0f / CV_PI;
}

bool FaceLandmarkResult::is_face_valid(float min_distance_cm, float max_distance_cm) const {
    if (!detected || landmarks.size() < 468) return false;
    
    // Estimate distance based on face width (pixels)
    float face_width = get_face_width();
    if (face_width <= 0) return false;
    
    // Reference: 150 pixels face width at 40cm
    const float REF_FACE_WIDTH_PX = 150.0f;
    const float REF_DISTANCE_CM = 40.0f;
    float distance_cm = (REF_FACE_WIDTH_PX * REF_DISTANCE_CM) / face_width;
    
    // Check distance
    if (distance_cm < min_distance_cm || distance_cm > max_distance_cm) {
        return false;
    }
    
    // Check orientation (not too extreme)
    float yaw = get_yaw_ratio();
    float pitch = get_pitch_ratio();
    float roll = std::abs(get_roll_degrees());
    
    return yaw < 0.75f && pitch < 0.6f && roll < 25.0f;
}

// =============================================================================
// BPUFaceLandmark Implementation
// =============================================================================

BPUFaceLandmark::BPUFaceLandmark(const std::string& model_path)
    : model_path_(model_path) {
    
    if (!initialize()) {
        std::cerr << "❌ Failed to initialize BPU Face Landmark model" << std::endl;
    }
}

BPUFaceLandmark::~BPUFaceLandmark() {
    free_tensors();
    
    if (packed_handle_) {
        hbDNNRelease(packed_handle_);
        packed_handle_ = nullptr;
    }
}

bool BPUFaceLandmark::initialize() {
    std::cout << "🔄 Loading BPU Face Landmark model: " << model_path_ << std::endl;
    
    // Load model from file
    const char* model_file = model_path_.c_str();
    int32_t ret = hbDNNInitializeFromFiles(&packed_handle_, &model_file, 1);
    
    if (ret != 0 || !packed_handle_) {
        std::cerr << "❌ hbDNNInitializeFromFiles failed: " << ret << std::endl;
        return false;
    }
    
    // Get model names
    const char** model_names = nullptr;
    int32_t model_count = 0;
    ret = hbDNNGetModelNameList(&model_names, &model_count, packed_handle_);
    
    if (ret != 0 || model_count == 0) {
        std::cerr << "❌ hbDNNGetModelNameList failed" << std::endl;
        return false;
    }
    
    model_name_ = model_names[0];
    std::cout << "   Model name: " << model_name_ << std::endl;
    
    // Get model handle
    ret = hbDNNGetModelHandle(&dnn_handle_, packed_handle_, model_name_.c_str());
    
    if (ret != 0 || !dnn_handle_) {
        std::cerr << "❌ hbDNNGetModelHandle failed" << std::endl;
        return false;
    }
    
    // Verify input/output counts
    int32_t input_count = 0, output_count = 0;
    hbDNNGetInputCount(&input_count, dnn_handle_);
    hbDNNGetOutputCount(&output_count, dnn_handle_);
    
    std::cout << "   Inputs: " << input_count << ", Outputs: " << output_count << std::endl;
    
    if (input_count != 1 || output_count != 2) {
        std::cerr << "❌ Unexpected input/output count" << std::endl;
        return false;
    }
    
    // Get input tensor properties
    hbDNNTensorProperties input_props;
    hbDNNGetInputTensorProperties(&input_props, dnn_handle_, 0);
    
    std::cout << "   Input shape: [";
    for (int i = 0; i < input_props.validShape.numDimensions; ++i) {
        std::cout << input_props.validShape.dimensionSize[i];
        if (i < input_props.validShape.numDimensions - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    // Allocate tensors
    if (!allocate_tensors()) {
        std::cerr << "❌ Failed to allocate tensors" << std::endl;
        return false;
    }
    
    output_count_ = output_count;
    initialized_.store(true);
    
    std::cout << "✅ BPU Face Landmark model loaded successfully" << std::endl;
    return true;
}

bool BPUFaceLandmark::allocate_tensors() {
    // Allocate input tensor
    hbDNNTensorProperties input_props;
    hbDNNGetInputTensorProperties(&input_props, dnn_handle_, 0);
    
    memset(&input_tensor_, 0, sizeof(input_tensor_));
    input_tensor_.properties = input_props;
    
    // Allocate input buffer
    int input_size = input_props.alignedByteSize;
    int ret = hbSysAllocCachedMem(&input_tensor_.sysMem[0], input_size);
    if (ret != 0) {
        std::cerr << "❌ Failed to allocate input tensor: " << ret << std::endl;
        return false;
    }
    
    // Allocate output tensors
    int32_t output_count = 0;
    hbDNNGetOutputCount(&output_count, dnn_handle_);
    
    output_tensors_ = new hbDNNTensor[output_count];
    
    for (int i = 0; i < output_count; ++i) {
        hbDNNTensorProperties output_props;
        hbDNNGetOutputTensorProperties(&output_props, dnn_handle_, i);
        
        memset(&output_tensors_[i], 0, sizeof(hbDNNTensor));
        output_tensors_[i].properties = output_props;
        
        int output_size = output_props.alignedByteSize;
        ret = hbSysAllocCachedMem(&output_tensors_[i].sysMem[0], output_size);
        if (ret != 0) {
            std::cerr << "❌ Failed to allocate output tensor " << i << std::endl;
            return false;
        }
    }
    
    return true;
}

void BPUFaceLandmark::free_tensors() {
    // Free input tensor
    if (input_tensor_.sysMem[0].virAddr) {
        hbSysFreeMem(&input_tensor_.sysMem[0]);
        input_tensor_.sysMem[0].virAddr = nullptr;
    }
    
    // Free output tensors
    if (output_tensors_) {
        for (int i = 0; i < output_count_; ++i) {
            if (output_tensors_[i].sysMem[0].virAddr) {
                hbSysFreeMem(&output_tensors_[i].sysMem[0]);
            }
        }
        delete[] output_tensors_;
        output_tensors_ = nullptr;
    }
}

void BPUFaceLandmark::preprocess(const cv::Mat& input, uint8_t* output_buffer) {
    cv::Mat resized, rgb;
    
    // Resize to 192x192 if needed
    if (input.cols != INPUT_WIDTH || input.rows != INPUT_HEIGHT) {
        cv::resize(input, resized, cv::Size(INPUT_WIDTH, INPUT_HEIGHT));
    } else {
        resized = input;
    }
    
    // Convert BGR to RGB
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    
    // Copy to output buffer (NHWC format: batch=1, H=192, W=192, C=3)
    memcpy(output_buffer, rgb.data, INPUT_WIDTH * INPUT_HEIGHT * 3);
}

FaceLandmarkResult BPUFaceLandmark::parse_output(const cv::Rect& original_roi) {
    FaceLandmarkResult result;
    result.face_roi = original_roi;
    
    // Output 0: 1404 floats = 468 landmarks x 3 (x, y, z)
    // Output 1: 1 float = confidence
    
    // Get landmarks from output 0
    float* landmarks_data = reinterpret_cast<float*>(output_tensors_[0].sysMem[0].virAddr);
    
    // Flush cache to ensure we read updated data
    hbSysFlushMem(&output_tensors_[0].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
    hbSysFlushMem(&output_tensors_[1].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
    
    result.landmarks.reserve(NUM_LANDMARKS);
    
    for (int i = 0; i < NUM_LANDMARKS; ++i) {
        float x = landmarks_data[i * 3 + 0];
        float y = landmarks_data[i * 3 + 1];
        float z = landmarks_data[i * 3 + 2];
        
        // Normalize from pixel space (0-192) to (0-1)
        x /= static_cast<float>(INPUT_WIDTH);
        y /= static_cast<float>(INPUT_HEIGHT);
        
        result.landmarks.emplace_back(x, y, z);
    }
    
    // Get confidence from output 1
    float* confidence_data = reinterpret_cast<float*>(output_tensors_[1].sysMem[0].virAddr);
    result.confidence = confidence_data[0];
    
    // Check if detection is valid (positive confidence typically means face detected)
    result.detected = (result.confidence > -5.0f);
    
    return result;
}

FaceLandmarkResult BPUFaceLandmark::detect(const cv::Mat& face_crop, const cv::Rect& original_roi) {
    FaceLandmarkResult result;
    
    if (!initialized_.load()) {
        std::cerr << "⚠️ BPU Face Landmark not initialized" << std::endl;
        return result;
    }
    
    if (face_crop.empty()) {
        return result;
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // Preprocess input
    preprocess(face_crop, reinterpret_cast<uint8_t*>(input_tensor_.sysMem[0].virAddr));
    
    // Flush input cache
    hbSysFlushMem(&input_tensor_.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    
    // Run inference
    hbDNNInferCtrlParam ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&ctrl_param);
    
    hbDNNTaskHandle_t task_handle = nullptr;
    hbDNNTensor* output_ptr = output_tensors_;
    
    int32_t ret = hbDNNInfer(&task_handle, &output_ptr, &input_tensor_, dnn_handle_, &ctrl_param);
    
    if (ret != 0) {
        std::cerr << "❌ hbDNNInfer failed: " << ret << std::endl;
        return result;
    }
    
    // Wait for completion
    ret = hbDNNWaitTaskDone(task_handle, 0);
    if (ret != 0) {
        std::cerr << "❌ hbDNNWaitTaskDone failed: " << ret << std::endl;
        hbDNNReleaseTask(task_handle);
        return result;
    }
    
    // Release task
    hbDNNReleaseTask(task_handle);
    
    // Parse output
    result = parse_output(original_roi);
    
    // Update timing
    auto end = std::chrono::high_resolution_clock::now();
    float elapsed_ms = std::chrono::duration<float, std::milli>(end - start).count();
    
    // Exponential moving average
    float alpha = 0.1f;
    avg_inference_time_ms_.store(
        alpha * elapsed_ms + (1.0f - alpha) * avg_inference_time_ms_.load()
    );
    
    return result;
}

FaceLandmarkResult BPUFaceLandmark::detect_in_image(const cv::Mat& image, cv::Rect& face_roi) {
    FaceLandmarkResult result;
    
    if (image.empty()) {
        return result;
    }
    
    // If face_roi is not provided, use center crop as fallback
    // In production, caller should provide face ROI from a face detector
    if (face_roi.width == 0 || face_roi.height == 0) {
        int size = std::min(image.cols, image.rows);
        face_roi.x = (image.cols - size) / 2;
        face_roi.y = (image.rows - size) / 2;
        face_roi.width = size;
        face_roi.height = size;
    }
    
    // Clamp ROI to image bounds
    face_roi.x = std::max(0, face_roi.x);
    face_roi.y = std::max(0, face_roi.y);
    face_roi.width = std::min(face_roi.width, image.cols - face_roi.x);
    face_roi.height = std::min(face_roi.height, image.rows - face_roi.y);
    
    // Extract face crop
    cv::Mat face_crop = image(face_roi);
    
    // Run detection
    return detect(face_crop, face_roi);
}

void BPUFaceLandmark::draw_landmarks(cv::Mat& image, const FaceLandmarkResult& result,
                                      cv::Scalar color) {
    if (!result.detected || result.landmarks.empty()) {
        return;
    }
    
    // Draw all landmarks as small dots
    for (size_t i = 0; i < result.landmarks.size(); ++i) {
        cv::Point2f pt = result.get_landmark_point(static_cast<int>(i));
        cv::circle(image, pt, 1, color, -1);
    }
    
    // Highlight key landmarks
    struct KeyLandmark {
        int index;
        cv::Scalar color;
        const char* name;
    };
    
    std::vector<KeyLandmark> key_landmarks = {
        {FaceLandmarkResult::NOSE_TIP, cv::Scalar(0, 0, 255), "Nose"},
        {FaceLandmarkResult::LEFT_EYE_OUTER, cv::Scalar(255, 0, 0), "L-Eye"},
        {FaceLandmarkResult::RIGHT_EYE_OUTER, cv::Scalar(255, 0, 0), "R-Eye"},
        {FaceLandmarkResult::CHIN, cv::Scalar(0, 255, 255), "Chin"},
        {FaceLandmarkResult::FOREHEAD, cv::Scalar(255, 0, 255), "Forehead"},
    };
    
    for (const auto& kl : key_landmarks) {
        cv::Point2f pt = result.get_landmark_point(kl.index);
        cv::circle(image, pt, 5, kl.color, 2);
    }
    
    // Draw face oval (outline)
    std::vector<int> face_oval = {
        10, 338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288,
        397, 365, 379, 378, 400, 377, 152, 148, 176, 149, 150, 136,
        172, 58, 132, 93, 234, 127, 162, 21, 54, 103, 67, 109
    };
    
    for (size_t i = 0; i < face_oval.size(); ++i) {
        cv::Point2f pt1 = result.get_landmark_point(face_oval[i]);
        cv::Point2f pt2 = result.get_landmark_point(face_oval[(i + 1) % face_oval.size()]);
        cv::line(image, pt1, pt2, cv::Scalar(0, 255, 255), 1);
    }
}

} // namespace mdai



