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
    bool enable_depth = true;
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
    
    // Sensor options - OPTIMIZED for spoof detection
    int internal_queue_size = 1;       // Lower latency for real-time analysis
    int emitter_enabled = 1;           // Always on for consistent IR material analysis
    float laser_power = 200.0f;        // Higher power for better depth and IR quality
    
    // Color sensor options - FIXED for consistent spoof detection
    bool auto_exposure = false;           // Fixed exposure for consistent IR analysis
    float manual_exposure = 8000.0f;      // Shorter exposure for better IR material analysis
    float manual_gain = 32.0f;            // Lower gain for better quality and less noise
    
    // Post-processing filters - ENABLED for spoof detection
    bool enable_spatial_filter = true;     // Smooth depth surfaces for material analysis
    float spatial_alpha = 0.5f;
    float spatial_delta = 20.0f;
    
    bool enable_temporal_filter = true;    // Consistent depth over time for liveness
    float temporal_alpha = 0.4f;
    float temporal_delta = 20.0f;
    
    bool enable_hole_filling = true;       // Fill missing depth data for complete analysis
    
    // Alignment - ENABLED for accurate multi-modal fusion
    bool align_to_color = true;   // Align depth frames to color for accurate face analysis
    
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
     * @brief Check if camera is healthy (receiving frames within timeout)
     * @param timeout_ms Timeout in milliseconds (default 3000ms)
     * @return true if frames received within timeout, false otherwise
     */
    bool is_camera_healthy(int timeout_ms = 3000) const;
    
    /**
     * @brief Get milliseconds since last frame was received
     * @return Milliseconds since last frame, or -1 if no frame received yet
     */
    int64_t get_ms_since_last_frame() const;
    
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
    
    // Frame health monitoring
    mutable std::mutex frame_time_mutex_;
    std::chrono::steady_clock::time_point last_frame_time_;
    std::atomic<bool> first_frame_received_{false};
    
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
