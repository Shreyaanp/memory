#pragma once

#include "FrameBox.hpp"
#include "RingBuffer.hpp"
#include "CameraInitializer.hpp"
#include <librealsense2/rs.hpp>
#include <thread>
#include <atomic>
#include <memory>
#include <functional>

namespace mdai {

/**
 * @brief Camera configuration structure
 */
struct CameraConfig {
    // Stream configuration
    int depth_width = 848;
    int depth_height = 480;
    int depth_fps = 30;
    
    int color_width = 848;
    int color_height = 480;
    int color_fps = 30;
    
    bool enable_ir = true;  // Enable infrared streams
    int ir_width = 848;
    int ir_height = 480;
    int ir_fps = 30;
    
    // Sensor options
    int internal_queue_size = 2;       // Small queue for low latency
    int emitter_enabled = 1;           // 0=off, 1=on, 2=auto, 3=LED
    float laser_power = 150.0f;        // mW
    
    // Color sensor options
    bool auto_exposure = true;
    float manual_exposure = 16000.0f;  // microseconds (if auto_exposure=false)
    float manual_gain = 64.0f;         // Gain value (if auto_exposure=false)
    
    // Post-processing filters
    bool enable_spatial_filter = false;
    float spatial_alpha = 0.5f;
    float spatial_delta = 20.0f;
    
    bool enable_temporal_filter = false;
    float temporal_alpha = 0.4f;
    float temporal_delta = 20.0f;
    
    bool enable_hole_filling = false;
    
    // Alignment
    bool align_to_color = false;  // Align depth frames to color
    
    // Device selection
    std::string device_serial = "";  // Empty = use any device
};

/**
 * @brief Producer class - captures frames from RealSense camera
 * 
 * The Producer:
 * - Configures and starts the RealSense pipeline
 * - Uses non-blocking poll_for_frames() to capture
 * - Packages frames into FrameBox with all metadata
 * - Writes to DynamicRingBuffer (never blocks)
 * - Handles camera disconnection and reconnection
 */
class Producer {
public:
    /**
     * @brief Constructor
     * @param config Camera configuration
     * @param ring_buffer Pointer to ring buffer to write frames
     */
    Producer(const CameraConfig& config, DynamicRingBuffer* ring_buffer);
    
    /**
     * @brief Destructor - stops capture thread
     */
    ~Producer();
    
    /**
     * @brief Start the capture thread
     * @return true if started successfully, false otherwise
     */
    bool start();
    
    /**
     * @brief Stop the capture thread
     */
    void stop();
    
    /**
     * @brief Check if producer is running
     * @return true if running, false otherwise
     */
    bool is_running() const;
    
    /**
     * @brief Get current configuration
     * @return Camera configuration
     */
    const CameraConfig& get_config() const;
    
    /**
     * @brief Update camera options (while running)
     * @param option RealSense option to update
     * @param value New value
     * @return true if updated successfully
     */
    bool set_option(rs2_option option, float value);
    
    /**
     * @brief Get current option value
     * @param option RealSense option to query
     * @return Current value, or 0 if not available
     */
    float get_option(rs2_option option) const;
    
    /**
     * @brief Get frames per second (actual capture rate)
     * @return FPS
     */
    float get_fps() const;
    
    /**
     * @brief Get total frames captured
     * @return Total frames
     */
    uint64_t get_total_frames_captured() const;
    
    /**
     * @brief Check if camera is connected
     * @return true if connected, false otherwise
     */
    bool is_camera_connected() const;
    
    /**
     * @brief Set error callback
     * @param callback Function to call on error (receives error message)
     */
    void set_error_callback(std::function<void(const std::string&)> callback);
    
    /**
     * @brief Set status callback
     * @param callback Function to call on status updates
     */
    void set_status_callback(std::function<void(const std::string&)> callback);

private:
    CameraConfig config_;
    DynamicRingBuffer* ring_buffer_;
    
    // RealSense objects
    rs2::pipeline pipe_;
    rs2::config rs_config_;
    rs2::pipeline_profile profile_;
    
    // Camera initialization
    std::unique_ptr<CameraInitializer> camera_initializer_;
    
    // Post-processing filters
    rs2::spatial_filter spatial_filter_;
    rs2::temporal_filter temporal_filter_;
    rs2::hole_filling_filter hole_filling_filter_;
    rs2::align* aligner_ = nullptr;
    
    // Capture thread
    std::unique_ptr<std::thread> capture_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> camera_connected_{false};
    
    // Statistics
    std::atomic<uint64_t> total_frames_captured_{0};
    std::atomic<uint64_t> sequence_id_{0};
    
    // FPS calculation
    std::chrono::steady_clock::time_point last_fps_calc_time_;
    std::atomic<float> current_fps_{0.0f};
    uint64_t frames_since_last_calc_{0};
    
    // Callbacks
    std::function<void(const std::string&)> error_callback_;
    std::function<void(const std::string&)> status_callback_;
    std::mutex callback_mutex_;
    
    // Camera intrinsics/extrinsics (queried once at startup)
    rs2_intrinsics depth_intrinsics_;
    rs2_intrinsics color_intrinsics_;
    rs2_intrinsics ir_intrinsics_;
    rs2_extrinsics extrinsics_depth_to_color_;
    float depth_scale_;
    
    /**
     * @brief Configure the RealSense pipeline
     * @return true if configured successfully
     */
    bool configure_pipeline();
    
    /**
     * @brief Configure sensor options
     */
    void configure_sensor_options();
    
    /**
     * @brief Query and store intrinsics/extrinsics
     */
    void query_camera_parameters();
    
    /**
     * @brief Main capture loop (runs in separate thread)
     */
    void capture_loop();
    
    /**
     * @brief Process a frameset and create FrameBox
     * @param frames Frameset from camera
     * @return FrameBox with all data
     */
    FrameBox process_frameset(rs2::frameset& frames);
    
    /**
     * @brief Apply post-processing filters
     * @param frames Frameset to filter
     * @return Filtered frameset
     */
    rs2::frameset apply_filters(rs2::frameset frames);
    
    /**
     * @brief Calculate FPS
     */
    void update_fps();
    
    /**
     * @brief Call error callback (thread-safe)
     */
    void report_error(const std::string& error);
    
    /**
     * @brief Call status callback (thread-safe)
     */
    void report_status(const std::string& status);
};

} // namespace mdai

