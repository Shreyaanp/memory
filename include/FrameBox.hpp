#pragma once

/**
 * @file FrameBox.hpp
 * @brief Simplified FrameBox for IR-only mode (no depth, no color, no anti-spoofing)
 * 
 * This is the streamlined version that only handles single IR camera input.
 */

#include <atomic>
#include <mutex>
#include <condition_variable>
#include <map>
#include <string>
#include <set>
#include <chrono>
#include <memory>
#include <tuple>
#include <cstring>
#include <vector>
#include <array>

// Forward declaration for OpenCV (optional)
#ifdef HAVE_OPENCV
#include <opencv2/opencv.hpp>
#endif

namespace mdai {

/**
 * @brief Simplified metadata for IR-only FrameBox
 */
struct FrameBoxMetadata {
    // Face ROI (from detection stage)
    bool face_detected = false;
    int face_x = 0;
    int face_y = 0;
    int face_w = 0;
    int face_h = 0;
    float face_detection_confidence = 0.0f;
    std::string face_rejection_reason = "";
    
    // Face landmarks (478 points from MediaPipe Face Mesh)
    // Each landmark is (x, y, z) where x, y are pixel coordinates
    // and z is relative depth from MediaPipe
    struct Landmark {
        float x, y, z;
        float visibility;  // MediaPipe visibility score (0-1, >0.5 = visible)
        float presence;    // MediaPipe presence score (0-1)
        Landmark(float x_ = 0, float y_ = 0, float z_ = 0, float v = 1.0f, float p = 1.0f) 
            : x(x_), y(y_), z(z_), visibility(v), presence(p) {}
    };
    std::vector<Landmark> landmarks;
    
    // Frame dimensions (for landmark normalization)
    // Frame may be rotated 90° CW at source for portrait orientation
    int frame_width = 1280;
    int frame_height = 800;
    
    // Generic extensible fields (kept for compatibility)
    std::map<std::string, float> scores;
    std::map<std::string, std::string> labels;
    std::map<std::string, bool> flags;
};

/**
 * @brief Simplified FrameBox containing IR frame and metadata
 * 
 * IR-only mode: Single infrared stream, no depth, no color, no anti-spoofing.
 * Used for face detection and nose tracking with MediaPipe.
 */
class FrameBox {
public:
    // Core frame data
    uint64_t sequence_id = 0;
    
    // Timestamp (hardware or system time in milliseconds)
    double timestamp = 0.0;
    
    // IR frame data (grayscale Y8)
    std::vector<uint8_t> ir_data;
    int ir_width = 0;
    int ir_height = 0;
    
    // Camera settings snapshot (minimal)
    float exposure = 0.0f;
    float gain = 0.0f;
    
    // Extensible metadata
    FrameBoxMetadata metadata;
    
    // Reference counting for memory management
    std::atomic<int> ref_count{0};
    
    // Pipeline stage completion tracking
    std::set<std::string> completed_stages;
    std::mutex stage_mutex;
    std::condition_variable stage_cv;
    
    // Metadata update protection
    std::mutex metadata_mutex;
    
    /**
     * @brief Default constructor
     */
    FrameBox();
    
    /**
     * @brief Copy constructor
     */
    FrameBox(const FrameBox& other);
    
    /**
     * @brief Move constructor
     */
    FrameBox(FrameBox&& other) noexcept;
    
    /**
     * @brief Copy assignment
     */
    FrameBox& operator=(const FrameBox& other);
    
    /**
     * @brief Move assignment
     */
    FrameBox& operator=(FrameBox&& other) noexcept;
    
    /**
     * @brief Notify that a pipeline stage has completed
     */
    void notify_stage_complete(const std::string& stage_name);
    
    /**
     * @brief Wait for a specific pipeline stage to complete
     */
    bool wait_for_stage(const std::string& stage_name, int timeout_ms = 5000);
    
    /**
     * @brief Check if a pipeline stage has completed
     */
    bool is_stage_complete(const std::string& stage_name) const;
    
    /**
     * @brief Get frame dimensions
     */
    std::pair<int, int> get_dimensions() const;
    
    /**
     * @brief Check if this FrameBox has valid frame data
     */
    bool is_valid() const;
    
    // ===== IR ACCESS METHODS =====
    
#ifdef HAVE_OPENCV
    /**
     * @brief Get IR data as OpenCV Mat (grayscale)
     */
    cv::Mat get_ir_mat() const;
    
    /**
     * @brief Get IR data converted to BGR (for MediaPipe)
     * Simple grayscale→BGR conversion
     */
    cv::Mat get_ir_as_bgr() const;
#endif
    
    /**
     * @brief Get IR data as raw pointer
     */
    const uint8_t* get_ir_ptr() const;
    
    /**
     * @brief Get IR data as vector reference
     */
    const std::vector<uint8_t>& get_ir_vector() const;
    
    /**
     * @brief Get IR data as numpy-compatible format
     */
    std::tuple<const void*, int, int, const char*> get_ir_numpy() const;
    
    /**
     * @brief Get frame data as JSON string
     */
    std::string to_json() const;
    
    /**
     * @brief Increment reference count
     */
    void acquire();
    
    /**
     * @brief Decrement reference count
     */
    int release();
};

} // namespace mdai
