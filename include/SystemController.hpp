#pragma once

/**
 * @file SystemController.hpp
 * @brief Simplified SystemController for IR-only mode (no anti-spoofing)
 */

#include "Producer.hpp"
#include "RingBuffer.hpp"
#include "FaceDetector.hpp"
#include "SerialCommunicator.hpp"
#include "NetworkManager.hpp"
#include "CryptoUtils.hpp"
#include "TrustZoneIdentity.hpp"

#include <memory>
#include <atomic>
#include <thread>
#include <mutex>

#ifdef HAVE_OPENCV
#include <opencv2/opencv.hpp>
#endif

namespace mdai {

enum class SystemState {
    BOOT,                    // Screen 1: Logo
    PROVISIONING,            // Screen 2: Connecting to WiFi
    PROVISIONED,             // Screen 4: Connection Success (3s display)
    AWAIT_ADMIN_QR,          // Screen 3: Waiting for WiFi QR
    IDLE,                    // Screen 6: Waiting for session QR
    WIFI_CHANGE_CONNECTING,  // Connecting to new WiFi
    WIFI_CHANGE_SUCCESS,     // Screen 4: WiFi change success
    WIFI_CHANGE_FAILED,      // Screen 12: WiFi change failed
    READY,                   // Screen 7: Connected, waiting for start
    COUNTDOWN,               // Screen 8: Countdown (5,4,3,2,1)
    WARMUP,                  // Not used (kept for compatibility)
    ALIGN,                   // Screen 9: Face alignment
    PROCESSING,              // Screen 10: Processing
    SUCCESS,                 // Screen 11: Success (wait for delete)
    ERROR,                   // Screen 12: Error (3s → IDLE)
    DELETE_SCREEN            // Screen 15: Deleting data (3s → IDLE)
};

/**
 * @brief Simplified SystemController for IR-only face verification
 * 
 * No anti-spoofing, no camera mode switching, single IR stream.
 */
class SystemController {
public:
    SystemController();
    ~SystemController();

    bool initialize();
    void run();
    void stop();

private:
    // Components
    std::unique_ptr<Producer> producer_;
    std::unique_ptr<DynamicRingBuffer> ring_buffer_;
    std::unique_ptr<FaceDetector> face_detector_;
    std::unique_ptr<SerialCommunicator> serial_comm_;
    std::unique_ptr<NetworkManager> network_mgr_;

    // State Management
    std::atomic<SystemState> current_state_{SystemState::BOOT};
    std::atomic<bool> running_{false};
    
    // Timer for state transitions
    std::chrono::steady_clock::time_point state_timer_start_;
    std::atomic<bool> state_timer_active_{false};
    SystemState state_timer_target_{SystemState::IDLE};
    int state_timer_duration_ms_{0};
    
    // Configuration
    std::string device_id_;
    std::string hardware_id_;
    std::string previous_ssid_;
    std::string previous_password_;
    std::string session_id_;
    // platform_id removed - handled server-side only
    
    // WebSocket auth
    std::string pending_ws_token_;
    std::mutex ws_auth_mutex_;
    
    // Middleware Config
    std::string middleware_host_ = "mdai.mercle.ai";
    
    // Result ACK tracking
    std::atomic<bool> result_ack_received_{false};
    
    // Camera health monitoring
    std::atomic<bool> camera_error_detected_{false};
    std::string camera_error_message_;
    
    // Hardware failure tracking (for reboot logic)
    int hardware_failure_count_{0};
    std::chrono::steady_clock::time_point first_failure_time_;
    static constexpr int MAX_HARDWARE_RETRIES = 3;
    static constexpr int HARDWARE_FAILURE_REBOOT_TIMEOUT_SEC = 30;
    
    // Face Validation States (simplified - no depth-based distance)
    enum class FaceValidationState {
        VALID,              // Face detected and OK
        NO_FACE,            // No face detected
        EXTREME_ROTATION    // Head rotated too far
    };
    
    // Spiral tracking constants
    static constexpr float SPIRAL_COMPLETE_ANGLE = 1.75f * 3.14159f;
    static constexpr float PROGRESS_BOOST = 1.6f;
    static constexpr float MAGNETIC_STRENGTH = 0.35f;
    static constexpr float SPIRAL_RADIUS = 0.12f;
    static constexpr float SPIRAL_CENTER_X = 0.5f;
    static constexpr float SPIRAL_CENTER_Y = 0.45f;
    static constexpr int NO_FACE_TOLERANCE_FRAMES = 10;
    static constexpr float MOTION_PAUSE_THRESHOLD = 0.003f;
    static constexpr float NOSE_SMOOTHING_FACTOR = 0.4f;
    static constexpr float EXTREME_YAW_THRESHOLD = 0.90f;
    
    // Motion tracking for spiral
    struct MotionTracker {
        std::vector<cv::Point2f> nose_positions;
        float progress = 0.0f;
        float total_rotation = 0.0f;
        float last_angle = 0.0f;
        bool angle_initialized = false;
        int no_face_counter = 0;
        int no_face_grace_active = 0;
        FaceValidationState validation_state = FaceValidationState::NO_FACE;
        
        float smoothed_nose_x = 0.5f;
        float smoothed_nose_y = 0.5f;
        float display_x = 232.5f;
        float display_y = 232.5f;
        float smoothed_x = 232.5f;
        float smoothed_y = 232.5f;
        bool smoothing_initialized = false;
        
        void reset() {
            nose_positions.clear();
            progress = 0.0f;
            total_rotation = 0.0f;
            last_angle = 0.0f;
            angle_initialized = false;
            no_face_counter = 0;
            no_face_grace_active = 0;
            validation_state = FaceValidationState::NO_FACE;
            smoothed_nose_x = 0.5f;
            smoothed_nose_y = 0.5f;
            display_x = 232.5f;
            display_y = 232.5f;
            smoothed_x = 232.5f;
            smoothed_y = 232.5f;
            smoothing_initialized = false;
        }
    } motion_tracker_;

    // Helper Methods
    void set_state(SystemState new_state);
    void process_frame(FrameBox* frame);
    void clear_session();  // Clears all session data, disconnects WebSocket
    
    // State Handlers
    void handle_boot();
    void handle_await_admin_qr(FrameBox* frame);
    void handle_idle(FrameBox* frame);
    void handle_warmup(FrameBox* frame);
    void handle_align(FrameBox* frame);
    void handle_processing();
    void handle_face_timeout();
    
    // QR Handlers
    bool handle_wifi_qr(const std::string& qr_data);
    std::string load_qr_shared_key();
    
    // Camera Management (simplified - single mode)
    void initialize_camera();
    
    // Network Callbacks
    void on_websocket_message(const std::string& message);
    
    // Logic
    float calculate_circular_motion_progress(const cv::Point2f& nose);
    bool load_device_config();
    bool check_wifi_on_boot();
    
    // Face Validation (simplified)
    FaceValidationState validate_face_position(FrameBox* frame);
    bool is_face_orientation_extreme(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    
    // Hardware Health
    bool check_camera_health();
    void handle_hardware_error(const std::string& component, const std::string& error_msg);
};

} // namespace mdai
