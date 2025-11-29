#pragma once

/**
 * @file Producer.hpp
 * @brief Simplified Producer for IR-only mode
 * 
 * Single IR stream capture from RealSense camera.
 * No depth, no color, no laser, no filters.
 */

#include "FrameBox.hpp"
#include "RingBuffer.hpp"
#include <librealsense2/rs.hpp>
#include <thread>
#include <atomic>
#include <memory>
#include <functional>

namespace mdai {

/**
 * @brief Simplified camera configuration for IR-only mode
 */
struct CameraConfig {
    // IR stream configuration
    int ir_width = 848;
    int ir_height = 480;
    int ir_fps = 30;
    
    // Device selection
    std::string device_serial = "";  // Empty = use any device
    
    // IR sensor options
    bool auto_exposure = true;       // Auto exposure for IR
    float manual_exposure = 8000.0f; // Manual exposure if auto_exposure=false
    float manual_gain = 32.0f;       // Manual gain if auto_exposure=false
};

/**
 * @brief Simplified Producer - IR-only frame capture
 * 
 * Captures single IR stream from RealSense camera (left IR sensor).
 * Laser emitter is DISABLED for passive IR operation.
 * No depth, no color, no filters, no alignment.
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
     */
    bool start();
    
    /**
     * @brief Stop the capture thread
     */
    void stop();
    
    /**
     * @brief Check if producer is running
     */
    bool is_running() const;
    
    /**
     * @brief Get current configuration
     */
    const CameraConfig& get_config() const;
    
    /**
     * @brief Get frames per second (actual capture rate)
     */
    float get_fps() const;
    
    /**
     * @brief Get total frames captured
     */
    uint64_t get_total_frames_captured() const;
    
    /**
     * @brief Check if camera is connected
     */
    bool is_camera_connected() const;
    
    /**
     * @brief Check if camera is healthy (receiving frames)
     */
    bool is_camera_healthy(int timeout_ms = 3000) const;
    
    /**
     * @brief Get milliseconds since last frame
     */
    int64_t get_ms_since_last_frame() const;
    
    /**
     * @brief Set error callback
     */
    void set_error_callback(std::function<void(const std::string&)> callback);
    
    /**
     * @brief Set status callback
     */
    void set_status_callback(std::function<void(const std::string&)> callback);
    
private:
    CameraConfig config_;
    DynamicRingBuffer* ring_buffer_;
    
    // RealSense objects
    rs2::pipeline pipe_;
    rs2::config rs_config_;
    rs2::pipeline_profile profile_;
    
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
    
    // IR intrinsics
    rs2_intrinsics ir_intrinsics_;
    
    /**
     * @brief Configure the RealSense pipeline for IR-only
     */
    bool configure_pipeline();
    
    /**
     * @brief Configure IR sensor options
     */
    void configure_sensor_options();
    
    /**
     * @brief Query and store IR intrinsics
     */
    void query_camera_parameters();
    
    /**
     * @brief Main capture loop
     */
    void capture_loop();
    
    /**
     * @brief Process IR frame and create FrameBox
     */
    FrameBox process_ir_frame(rs2::video_frame& ir_frame);
    
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
