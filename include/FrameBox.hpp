#pragma once

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
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
 * @brief Extensible metadata header for FrameBox
 * 
 * This allows consumers to add their own processing results and flags
 * without modifying the core FrameBox structure.
 */
struct FrameBoxMetadata {
    // Processing stage tracking
    bool quality_gate_processed = false;
    bool anti_spoofing_processed = false;
    bool face_recognition_processed = false;
    bool batch_ml_processed = false;
    
    // Quality Gate Results
    struct QualityGateResults {
        // Individual gate results
        bool lighting_ok = false;
        bool motion_stable = false;
        bool face_positioned = false;
        bool sensors_synced = false;
        bool camera_stable = false;
        
        // Quality scores (0.0 to 1.0)
        float lighting_score = 0.0f;
        float motion_score = 0.0f;
        float positioning_score = 0.0f;
        float synchronization_score = 0.0f;
        float stability_score = 0.0f;
        
        // Overall quality assessment
        float overall_quality_score = 0.0f;
        bool quality_gate_passed = false;
        std::string quality_issues = "";
    } quality_gate;
    
    // Face ROI (from detection stage)
    bool face_detected = false;
    int face_x = 0;
    int face_y = 0;
    int face_w = 0;
    int face_h = 0;
    float face_detection_confidence = 0.0f;
    std::string face_rejection_reason = "";
    
    // Face landmarks (468 points from MediaPipe Face Mesh)
    // Each landmark is (x, y, z) where x, y are pixel coordinates
    // and z is relative depth
    struct Landmark {
        float x, y, z;
        float visibility;  // MediaPipe visibility score (0-1, >0.5 = visible)
        float presence;    // MediaPipe presence score (0-1)
        Landmark(float x_ = 0, float y_ = 0, float z_ = 0, float v = 1.0f, float p = 1.0f) 
            : x(x_), y(y_), z(z_), visibility(v), presence(p) {}
    };
    std::vector<Landmark> landmarks;
    
    // Anti-Spoofing Results
    struct AntiSpoofingResults {
        // Component scores (0.0 to 1.0)
        float depth_analysis_score = 0.0f;
        float ir_texture_score = 0.0f;
        float temporal_consistency_score = 0.0f;
        float cross_modal_score = 0.0f;
        
        // Overall anti-spoofing assessment
        float overall_liveness_score = 0.0f;
        bool is_live = false;
        float confidence = 0.0f;
        
        // Detection details
        std::string detected_attack_type = "";  // "screen", "mask", "photo", "ai_generated", "none"
        std::string rejection_reason = "";
        
        // Component-specific flags
        bool depth_anomaly_detected = false;
        bool ir_material_mismatch = false;
        bool temporal_inconsistency = false;
        bool cross_modal_disagreement = false;
    } anti_spoofing;
    
    // Face recognition results
    std::string detected_face_id = "";
    float face_confidence = 0.0f;
    
    // ML results (extensible)
    std::string emotion = "";
    int estimated_age = 0;
    
    // Combined processing decision
    bool is_ready_for_processing = false;
    std::string final_rejection_reason = "";
    
    // Generic extensible fields
    std::map<std::string, float> scores;      // e.g., {"liveness": 0.95, "quality": 0.88}
    std::map<std::string, std::string> labels; // e.g., {"expression": "smiling"}
    std::map<std::string, bool> flags;         // e.g., {"mask_detected": true}
};

/**
 * @brief Main data structure containing synchronized camera frames and metadata
 * 
 * FrameBox encapsulates:
 * - Synchronized depth, color, and IR frames from RealSense
 * - Camera intrinsics and extrinsics
 * - Camera settings snapshot (exposure, gain, emitter)
 * - Extensible metadata for processing pipeline
 * - Stage completion tracking for pipeline dependencies
 */
class FrameBox {
public:
    // Core frame data
    uint64_t sequence_id = 0;
    
    // Timestamps (hardware or system time in milliseconds)
    double time_depth = 0.0;
    double time_color = 0.0;
    double time_ir_left = 0.0;
    double time_ir_right = 0.0;
    
    // Frame data (raw data - no rs2::frame references to avoid SDK memory buildup)
    std::vector<uint16_t> depth_data;   // Depth values in depth units
    std::vector<uint8_t> color_data;    // Color RGB/BGR data
    std::vector<uint8_t> ir_left_data;  // IR left grayscale
    std::vector<uint8_t> ir_right_data; // IR right grayscale
    
    // Frame dimensions
    int depth_width = 0;
    int depth_height = 0;
    int color_width = 0;
    int color_height = 0;
    int ir_width = 0;
    int ir_height = 0;
    int color_bytes_per_pixel = 3; // RGB/BGR = 3, RGBA = 4
    
    // Camera intrinsic parameters (per stream)
    rs2_intrinsics depth_intrinsics;
    rs2_intrinsics color_intrinsics;
    rs2_intrinsics ir_intrinsics;
    
    // Extrinsic transformation (depth to color coordinate system)
    rs2_extrinsics extrinsics_depth_to_color;
    
    // Depth scale (meters per depth unit, typically 0.001)
    float depth_scale = 0.001f;
    
    // Camera settings snapshot
    float exposure = 0.0f;        // Microseconds
    float gain = 0.0f;            // Analog gain
    int emitter_state = 0;        // 0=off, 1=on, 2=auto, 3=LED
    float laser_power = 0.0f;     // mW
    
    // Filter settings (if post-processing applied)
    struct FilterSettings {
        float spatial_alpha = 0.5f;
        float spatial_delta = 20.0f;
        float temporal_alpha = 0.4f;
        float temporal_delta = 20.0f;
        int persistence_control = 3;
    } filter_settings;
    
    // Extensible metadata header
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
     * @brief Copy constructor (shallow copy of frames due to reference counting)
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
     * @brief Notify that a pipeline stage has completed processing this frame
     * @param stage_name Name of the completed stage (e.g., "anti_spoofing")
     */
    void notify_stage_complete(const std::string& stage_name);
    
    /**
     * @brief Wait for a specific pipeline stage to complete
     * @param stage_name Name of the stage to wait for
     * @param timeout_ms Timeout in milliseconds (default: 5000ms)
     * @return true if stage completed within timeout, false otherwise
     */
    bool wait_for_stage(const std::string& stage_name, int timeout_ms = 5000);
    
    /**
     * @brief Check if a pipeline stage has completed
     * @param stage_name Name of the stage to check
     * @return true if stage is complete, false otherwise
     */
    bool is_stage_complete(const std::string& stage_name) const;
    
    /**
     * @brief Get frame dimensions (from depth frame)
     * @return pair of (width, height)
     */
    std::pair<int, int> get_dimensions() const;
    
    /**
     * @brief Check if this FrameBox has valid frame data
     * @return true if contains valid frames, false otherwise
     */
    bool is_valid() const;
    
    /**
     * @brief Get depth value at specific pixel (in meters)
     * @param x Pixel x coordinate
     * @param y Pixel y coordinate
     * @return Depth in meters, or 0 if invalid
     */
    float get_depth_at(int x, int y) const;
    
    /**
     * @brief Deproject pixel to 3D point in camera coordinates
     * @param x Pixel x coordinate  
     * @param y Pixel y coordinate
     * @return 3D point as array [X, Y, Z] in meters
     */
    std::array<float, 3> deproject_pixel_to_point(int x, int y) const;
    
    // ===== UNIFIED ACCESS SYSTEM =====
    
    /**
     * @brief Get depth data as OpenCV Mat (if OpenCV available)
     * @return OpenCV Mat with depth data, or empty Mat if not available
     */
#ifdef HAVE_OPENCV
    cv::Mat get_depth_mat() const;
    
    /**
     * @brief Get color data as OpenCV Mat (if OpenCV available)
     * @return OpenCV Mat with color data, or empty Mat if not available
     */
    cv::Mat get_color_mat() const;
#else
    void* get_depth_mat() const { return nullptr; }
    void* get_color_mat() const { return nullptr; }
#endif
    
    /**
     * @brief Get depth data as raw pointer
     * @return Pointer to depth data, or nullptr if not available
     */
    const uint16_t* get_depth_ptr() const;
    
    /**
     * @brief Get color data as raw pointer
     * @return Pointer to color data, or nullptr if not available
     */
    const uint8_t* get_color_ptr() const;
    
    /**
     * @brief Get depth data as vector reference
     * @return Reference to depth data vector
     */
    const std::vector<uint16_t>& get_depth_vector() const;
    
    /**
     * @brief Get color data as vector reference
     * @return Reference to color data vector
     */
    const std::vector<uint8_t>& get_color_vector() const;
    
    /**
     * @brief Get IR left data as vector reference
     * @return Reference to IR left data vector
     */
    const std::vector<uint8_t>& get_ir_left_vector() const;
    
    /**
     * @brief Get IR right data as vector reference
     * @return Reference to IR right data vector
     */
    const std::vector<uint8_t>& get_ir_right_vector() const;
    
    /**
     * @brief Get depth data as numpy-compatible format (for Python)
     * @return Tuple of (data_ptr, width, height, dtype)
     */
    std::tuple<const void*, int, int, const char*> get_depth_numpy() const;
    
    /**
     * @brief Get color data as numpy-compatible format (for Python)
     * @return Tuple of (data_ptr, width, height, dtype)
     */
    std::tuple<const void*, int, int, const char*> get_color_numpy() const;
    
    /**
     * @brief Get frame data as JSON string (for web APIs)
     * @return JSON string with frame metadata and data info
     */
    std::string to_json() const;
    
    /**
     * @brief Get frame data as binary blob (for network transfer)
     * @return Binary data as vector of bytes
     */
    std::vector<uint8_t> to_binary() const;
    
    /**
     * @brief Create from binary blob (for network transfer)
     * @param data Binary data
     * @return true if successful, false otherwise
     */
    bool from_binary(const std::vector<uint8_t>& data);
    
    /**
     * @brief Increment reference count
     */
    void acquire();
    
    /**
     * @brief Decrement reference count
     * @return Current reference count after decrement
     */
    int release();
};

} // namespace mdai
