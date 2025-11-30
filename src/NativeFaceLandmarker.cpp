/**
 * Native MediaPipe Face Landmarker Implementation
 * 
 * Uses the MediaPipe Tasks C API (libface_landmarker.so).
 * No Python dependency!
 */

#include "mediapipe/native_face_landmarker.h"

// MediaPipe C API headers
#include "mediapipe/tasks/c/vision/face_landmarker/face_landmarker.h"
#include "mediapipe/tasks/c/vision/core/common.h"
#include "mediapipe/tasks/c/components/containers/landmark.h"
#include "mediapipe/tasks/c/core/base_options.h"

#include <iostream>
#include <chrono>

namespace mdai {

// ============================================================================
// NativeFaceMeshResult Implementation
// ============================================================================

bool NativeFaceMeshResult::IsOccluded(float visibility_threshold, float occlusion_ratio) const {
    if (visibility.empty()) return false;
    
    int occluded_count = 0;
    for (float vis : visibility) {
        if (vis < visibility_threshold) {
            occluded_count++;
        }
    }
    
    float ratio = static_cast<float>(occluded_count) / visibility.size();
    return ratio > occlusion_ratio;
}

// ============================================================================
// NativeFaceLandmarker::Impl
// ============================================================================

class NativeFaceLandmarker::Impl {
public:
    Impl(const NativeFaceLandmarkerConfig& config)
        : config_(config)
        , landmarker_(nullptr)
        , initialized_(false)
        , frame_count_(0)
    {
        Initialize();
    }
    
    ~Impl() {
        if (landmarker_) {
            char* error_msg = nullptr;
            face_landmarker_close(landmarker_, &error_msg);
            if (error_msg) {
                std::cerr << "[NativeFaceLandmarker] Close error: " << error_msg << std::endl;
                free(error_msg);
            }
            landmarker_ = nullptr;
        }
    }
    
    bool Initialize() {
        if (initialized_) return true;
        
        // Configure options
        FaceLandmarkerOptions options = {};
        options.base_options.model_asset_path = config_.model_path.c_str();
        options.base_options.model_asset_buffer = nullptr;
        options.base_options.model_asset_buffer_count = 0;
        options.running_mode = VIDEO;  // Use VIDEO mode for tracking
        options.num_faces = config_.num_faces;
        options.min_face_detection_confidence = config_.min_face_detection_confidence;
        options.min_face_presence_confidence = config_.min_face_presence_confidence;
        options.min_tracking_confidence = config_.min_tracking_confidence;
        options.output_face_blendshapes = config_.output_face_blendshapes;
        options.output_facial_transformation_matrixes = false;
        options.result_callback = nullptr;  // Not using live stream mode
        
        // Create landmarker
        char* error_msg = nullptr;
        landmarker_ = face_landmarker_create(&options, &error_msg);
        
        if (!landmarker_) {
            last_error_ = error_msg ? error_msg : "Failed to create face landmarker";
            if (error_msg) free(error_msg);
            std::cerr << "❌ [NativeFaceLandmarker] Init failed: " << last_error_ << std::endl;
            return false;
        }
        
        initialized_ = true;
        std::cout << "✓ Native MediaPipe Face Landmarker initialized (478 points, ~44 FPS)" << std::endl;
        return true;
    }
    
    bool Detect(const cv::Mat& bgr_image, NativeFaceMeshResult& result) {
        try {
            // Use DetectForVideo with auto-incrementing timestamp
            int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()
            ).count();
            return DetectForVideo(bgr_image, timestamp, result);
        } catch (const std::exception& e) {
            last_error_ = std::string("❌ Detect exception: ") + e.what();
            std::cerr << last_error_ << std::endl;
            return false;
        } catch (...) {
            last_error_ = "❌ Detect unknown exception";
            std::cerr << last_error_ << std::endl;
            return false;
        }
    }
    
    bool DetectForVideo(const cv::Mat& bgr_image, int64_t timestamp_ms, NativeFaceMeshResult& result) {
        try {
            if (!initialized_ || !landmarker_) {
                last_error_ = "Landmarker not initialized";
                return false;
            }
            
            if (bgr_image.empty()) {
                last_error_ = "Empty input image";
                return false;
            }
            
            // Clear previous results
            result.landmarks.clear();
            result.visibility.clear();
            result.presence.clear();
            result.confidence = 0.0f;
            result.image_width = bgr_image.cols;
            result.image_height = bgr_image.rows;
            
            // Convert BGR to RGB
            cv::Mat rgb_image;
            cv::cvtColor(bgr_image, rgb_image, cv::COLOR_BGR2RGB);
            
            // Create MpImage
            MpImage mp_image;
            mp_image.type = MpImage::IMAGE_FRAME;
            mp_image.image_frame.format = SRGB;
            mp_image.image_frame.image_buffer = rgb_image.data;
            mp_image.image_frame.width = rgb_image.cols;
            mp_image.image_frame.height = rgb_image.rows;
            
            // Detect
            FaceLandmarkerResult mp_result = {};
            char* error_msg = nullptr;
            
            int status = face_landmarker_detect_for_video(landmarker_, mp_image, timestamp_ms, &mp_result, &error_msg);
            
            if (status != 0) {
                last_error_ = error_msg ? error_msg : "Detection failed";
                if (error_msg) free(error_msg);
                return false;
            }
            
            // Parse results
            if (mp_result.face_landmarks_count > 0 && mp_result.face_landmarks != nullptr) {
                const NormalizedLandmarks& face = mp_result.face_landmarks[0];
                
                result.landmarks.reserve(face.landmarks_count);
                result.visibility.reserve(face.landmarks_count);
                result.presence.reserve(face.landmarks_count);
                
                float min_x = 1.0f, min_y = 1.0f, max_x = 0.0f, max_y = 0.0f;
                float total_presence = 0.0f;
                
                for (uint32_t i = 0; i < face.landmarks_count; i++) {
                    const NormalizedLandmark& lm = face.landmarks[i];
                    
                    // Convert normalized coords to pixel coords
                    float x_pix = lm.x * bgr_image.cols;
                    float y_pix = lm.y * bgr_image.rows;
                    float z_pix = lm.z * bgr_image.cols;  // Z is relative depth
                    
                    result.landmarks.emplace_back(x_pix, y_pix, z_pix);
                    
                    float vis = lm.has_visibility ? lm.visibility : 1.0f;
                    float pres = lm.has_presence ? lm.presence : 1.0f;
                    
                    result.visibility.push_back(vis);
                    result.presence.push_back(pres);
                    
                    total_presence += pres;
                    
                    // Track bounding box
                    if (lm.x < min_x) min_x = lm.x;
                    if (lm.x > max_x) max_x = lm.x;
                    if (lm.y < min_y) min_y = lm.y;
                    if (lm.y > max_y) max_y = lm.y;
                }
                
                // Set bounding box (pixel coords)
                result.bbox = cv::Rect(
                    static_cast<int>(min_x * bgr_image.cols),
                    static_cast<int>(min_y * bgr_image.rows),
                    static_cast<int>((max_x - min_x) * bgr_image.cols),
                    static_cast<int>((max_y - min_y) * bgr_image.rows)
                );
                
                // Average presence as confidence
                result.confidence = face.landmarks_count > 0 ? 
                    total_presence / face.landmarks_count : 0.0f;
            }
            
            // Clean up result
            face_landmarker_close_result(&mp_result);
            
            frame_count_++;
            last_error_ = "";
            return true;
        } catch (const std::exception& e) {
            last_error_ = std::string("❌ DetectForVideo exception: ") + e.what();
            std::cerr << last_error_ << std::endl;
            return false;
        } catch (...) {
            last_error_ = "❌ DetectForVideo unknown exception";
            std::cerr << last_error_ << std::endl;
            return false;
        }
    }
    
    bool IsInitialized() const { return initialized_; }
    std::string GetLastError() const { return last_error_; }

private:
    NativeFaceLandmarkerConfig config_;
    void* landmarker_;
    bool initialized_;
    std::string last_error_;
    uint64_t frame_count_;
};

// ============================================================================
// NativeFaceLandmarker Public Interface
// ============================================================================

NativeFaceLandmarker::NativeFaceLandmarker(const NativeFaceLandmarkerConfig& config)
    : impl_(std::make_unique<Impl>(config))
{
}

NativeFaceLandmarker::~NativeFaceLandmarker() = default;

bool NativeFaceLandmarker::Detect(const cv::Mat& bgr_image, NativeFaceMeshResult& result) {
    try {
        return impl_->Detect(bgr_image, result);
    } catch (const std::exception& e) {
        std::cerr << "❌ NativeFaceLandmarker::Detect exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ NativeFaceLandmarker::Detect unknown exception" << std::endl;
        return false;
    }
}

bool NativeFaceLandmarker::DetectForVideo(const cv::Mat& bgr_image, int64_t timestamp_ms, NativeFaceMeshResult& result) {
    try {
        return impl_->DetectForVideo(bgr_image, timestamp_ms, result);
    } catch (const std::exception& e) {
        std::cerr << "❌ NativeFaceLandmarker::DetectForVideo exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ NativeFaceLandmarker::DetectForVideo unknown exception" << std::endl;
        return false;
    }
}

bool NativeFaceLandmarker::IsInitialized() const {
    return impl_->IsInitialized();
}

std::string NativeFaceLandmarker::GetLastError() const {
    return impl_->GetLastError();
}

}  // namespace mdai




