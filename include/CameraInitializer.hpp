#pragma once

#include <librealsense2/rs.hpp>
#include <atomic>
#include <functional>
#include <string>
#include <utility>

namespace mdai {

/**
 * @brief Camera initialization manager
 * 
 * This class handles device detection and validation for RealSense cameras.
 * Actual sensor stabilization happens via frame discarding in the Producer.
 */
class CameraInitializer {
public:
    using StatusCallback = std::function<void(const std::string&)>;
    using ProgressCallback = std::function<void(int)>; // 0-100
    
    enum class Phase {
        CONNECTING,
        READY
    };
    
    enum class Result {
        SUCCESS,
        CONNECTION_FAILED,
        SENSOR_ERROR,
        TIMEOUT,
        CANCELLED
    };

private:
    rs2::config config_;
    std::string locked_serial_;
    
    std::atomic<bool> initialized_{false};
    std::atomic<bool> cancelled_{false};
    std::atomic<Phase> current_phase_{Phase::CONNECTING};
    
    StatusCallback status_callback_;
    ProgressCallback progress_callback_;
    
public:
    /**
     * @brief Constructor
     * @param config Camera configuration
     * @param device_serial Optional serial to lock onto (empty = auto-detect)
     */
    explicit CameraInitializer(const rs2::config& config,
                               std::string device_serial = "");
    
    ~CameraInitializer() = default;
    
    void set_status_callback(StatusCallback callback);
    void set_progress_callback(ProgressCallback callback);
    
    /**
     * @brief Initialize camera - validates device exists and is accessible
     * @param timeout_ms Maximum time to wait (unused, kept for API compat)
     * @return Result of initialization
     */
    Result initialize(int timeout_ms = 30000);
    
    void cancel();
    
    bool is_ready() const { return initialized_.load(); }
    
    // Kept for API compatibility, does nothing
    void set_quick_mode(bool /*enable*/) {}
    
    Phase get_current_phase() const { return current_phase_.load(); }
    
    int get_progress() const { return initialized_.load() ? 100 : 0; }
    
    /**
     * @brief Get the locked device serial number
     * @return Device serial number
     */
    const std::string& get_device_serial() const { return locked_serial_; }

private:
    void report_status(const std::string& message);
    void report_progress(int progress);
    bool is_cancelled() const { return cancelled_.load(); }
    
    /**
     * @brief Find and lock onto a RealSense device
     * @return true if device found, false otherwise
     */
    bool find_and_lock_device();
};

} // namespace mdai
