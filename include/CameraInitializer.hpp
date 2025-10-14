#pragma once

#include <librealsense2/rs.hpp>
#include <chrono>
#include <atomic>
#include <thread>
#include <functional>
#include <string>
#include <utility>

namespace mdai {

/**
 * @brief Camera initialization and stabilization manager
 * 
 * This class handles the proper initialization sequence for RealSense cameras,
 * including sensor warm-up, autofocus stabilization, exposure adjustment,
 * and other camera-specific initialization requirements.
 */
class CameraInitializer {
public:
    // Callback types
    using StatusCallback = std::function<void(const std::string&)>;
    using ProgressCallback = std::function<void(int)>; // 0-100
    
    // Initialization phases
    enum class Phase {
        CONNECTING,
        SENSOR_WARMUP,
        AUTOFOCUS_STABILIZATION,
        EXPOSURE_STABILIZATION,
        FINAL_CALIBRATION,
        READY
    };
    
    // Initialization result
    enum class Result {
        SUCCESS,
        CONNECTION_FAILED,
        SENSOR_ERROR,
        TIMEOUT,
        CANCELLED
    };

private:
    rs2::pipeline pipe_;
    rs2::config config_;
    rs2::pipeline_profile profile_;
    std::string locked_serial_;
    
    std::atomic<bool> initialized_{false};
    std::atomic<bool> cancelled_{false};
    std::atomic<Phase> current_phase_{Phase::CONNECTING};
    
    // Timing constants (in milliseconds)
    static constexpr int SENSOR_WARMUP_TIME = 2000;        // 2 seconds
    static constexpr int AUTOFOCUS_STABILIZATION_TIME = 3000; // 3 seconds  
    static constexpr int EXPOSURE_STABILIZATION_TIME = 2000;  // 2 seconds
    static constexpr int FINAL_CALIBRATION_TIME = 1000;      // 1 second
    
    // Callbacks
    StatusCallback status_callback_;
    ProgressCallback progress_callback_;
    
    // Internal state
    std::chrono::steady_clock::time_point start_time_;
    int total_phases_ = 5; // Total number of phases
    
public:
    /**
     * @brief Constructor
     * @param config Camera configuration
     * @param device_serial Optional serial to lock onto (empty = auto-detect)
     */
    explicit CameraInitializer(const rs2::config& config,
                               std::string device_serial = "");
    
    /**
     * @brief Destructor
     */
    ~CameraInitializer();
    
    /**
     * @brief Set status callback for logging
     * @param callback Function to call with status messages
     */
    void set_status_callback(StatusCallback callback);
    
    /**
     * @brief Set progress callback for initialization progress
     * @param callback Function to call with progress (0-100)
     */
    void set_progress_callback(ProgressCallback callback);
    
    /**
     * @brief Initialize camera with proper stabilization
     * @param timeout_ms Maximum time to wait for initialization (0 = no timeout)
     * @return Result of initialization
     */
    Result initialize(int timeout_ms = 30000);
    
    /**
     * @brief Cancel ongoing initialization
     */
    void cancel();
    
    /**
     * @brief Check if camera is initialized and ready
     * @return true if ready, false otherwise
     */
    bool is_ready() const { return initialized_.load(); }
    
    /**
     * @brief Get current initialization phase
     * @return Current phase
     */
    Phase get_current_phase() const { return current_phase_.load(); }
    
    /**
     * @brief Get the pipeline profile after successful initialization
     * @return Pipeline profile
     */
    rs2::pipeline_profile get_profile() const { return profile_; }
    
    /**
     * @brief Get the pipeline for frame capture
     * @return Pipeline object
     */
    rs2::pipeline get_pipeline() { return std::move(pipe_); }
    
    /**
     * @brief Get initialization progress (0-100)
     * @return Progress percentage
     */
    int get_progress() const;
    
    /**
     * @brief Get elapsed time since initialization started
     * @return Elapsed time in milliseconds
     */
    int get_elapsed_time_ms() const;

private:
    /**
     * @brief Report status message
     * @param message Status message
     */
    void report_status(const std::string& message);
    
    /**
     * @brief Report progress update
     * @param progress Progress percentage (0-100)
     */
    void report_progress(int progress);
    
    /**
     * @brief Execute sensor warm-up phase
     * @return true if successful, false if cancelled
     */
    bool execute_sensor_warmup();
    
    /**
     * @brief Execute autofocus stabilization phase
     * @return true if successful, false if cancelled
     */
    bool execute_autofocus_stabilization();
    
    /**
     * @brief Execute exposure stabilization phase
     * @return true if successful, false if cancelled
     */
    bool execute_exposure_stabilization();
    
    /**
     * @brief Execute final calibration phase
     * @return true if successful, false if cancelled
     */
    bool execute_final_calibration();
    
    /**
     * @brief Wait for specified duration with progress updates
     * @param duration_ms Duration to wait in milliseconds
     * @param phase_name Name of the phase for logging
     * @return true if completed, false if cancelled
     */
    bool wait_with_progress(int duration_ms, const std::string& phase_name);
    
    /**
     * @brief Check if initialization was cancelled
     * @return true if cancelled, false otherwise
     */
    bool is_cancelled() const { return cancelled_.load(); }

    /**
     * @brief Resolve and cache the RealSense device we should operate on
     * @param phase Human readable phase name for logging
     * @return Handle to the selected device
     */
    rs2::device get_locked_device(const std::string& phase);
};

} // namespace mdai
