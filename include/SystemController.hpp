#pragma once

#include "Producer.hpp"
#include "RingBuffer.hpp"
#include "FaceDetector.hpp"
#include "SerialCommunicator.hpp"
#include "NetworkManager.hpp"
#include "CryptoUtils.hpp"
#include "AntiSpoofing.hpp"
#include "TrustZoneIdentity.hpp"

#include <memory>
#include <atomic>
#include <thread>
#include <mutex>

namespace mdai {

enum class SystemState {
    BOOT,                    // Screen 1: Logo
    PROVISIONING,            // Screen 2: Connecting to WiFi
    PROVISIONED,             // Screen 3: Connection Success (3s display)
    AWAIT_ADMIN_QR,          // No screen (boot checks WiFi first)
    IDLE,                    // Screen 6: Waiting for QR (User or WiFi Change)
    WIFI_CHANGE_CONNECTING,  // Screen 11-12: Connecting to new WiFi
    WIFI_CHANGE_SUCCESS,     // Screen 13: WiFi change success
    WIFI_CHANGE_FAILED,      // Screen 14: WiFi change failed + fallback
    READY,                   // Screen 7: Connected, waiting for user to start
    COUNTDOWN,               // Screen 8: Countdown (5,4,3,2,1)
    WARMUP,                  // Camera warming up (5 frames)
    ALIGN,                   // Active Liveness (Recording + Face Mesh)
    PROCESSING,              // Batch Anti-Spoofing Analysis
    SUCCESS,                 // Success
    ERROR,                   // Error
    LOGOUT                   // Thank you / Logout
};

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
    std::unique_ptr<AntiSpoofingPipeline> anti_spoofing_;

    // State Management
    std::atomic<SystemState> current_state_{SystemState::BOOT};
    std::atomic<bool> running_{false};
    
    // Timer for state transitions (e.g., Thank You -> IDLE)
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
    std::string platform_id_;
    
    // WebSocket auth credentials (for auto-reauth on reconnect)
    std::string pending_ws_token_;
    std::mutex ws_auth_mutex_;
    
    // Camera Mode Tracking
    enum class CameraMode { UNINITIALIZED, RGB_ONLY, FULL };
    std::atomic<CameraMode> current_camera_mode_{CameraMode::UNINITIALIZED};
    
    // Middleware Config
    std::string middleware_host_ = "mdai.mercle.ai"; // EC2 instance
    
    // Test Mode Flag (set to false for real verification)
    bool test_mode_ = false;  // Set to false to enable full verification flow (including countdown)
    
    // UI Test Mode - For testing Screen 0 (nose tracking UI)
    // When true: Skips all screens except Screen 0, Screen 11 (success), Screen 12 (error)
    bool ui_test_mode_ = false;  // PRODUCTION MODE
    
    // Result ACK tracking
    std::atomic<bool> result_ack_received_{false};
    
    // Camera health monitoring
    std::atomic<bool> camera_error_detected_{false};
    std::string camera_error_message_;
    
    // Face Validation States
    enum class FaceValidationState {
        VALID,              // Face detected, distance OK, orientation OK
        NO_FACE,            // No face detected
        TOO_CLOSE,          // Face too close (< 25cm)
        TOO_FAR,            // Face too far (> 60cm)
        EXTREME_ROTATION    // Head rotated too far (can't complete spiral)
    };
    
    // =========================================================================
    // Face Validation Constants (CALIBRATION REQUIRED)
    // =========================================================================
    // These values need to be calibrated based on actual camera setup:
    //
    // REFERENCE_FACE_WIDTH: Measure face bounding box width at REFERENCE_DISTANCE
    //   - Place a person at 40cm from camera
    //   - Log the face_w value from FrameBox metadata
    //   - Update this constant with that value
    //
    // Distance formula: distance_cm = (REFERENCE_FACE_WIDTH * REFERENCE_DISTANCE) / face_width
    //
    // CALIBRATION for 848x480 resolution:
    //   - Original: 150px at 1280x720
    //   - Scale factor: 848/1280 = 0.6625
    //   - New value: 150 * 0.6625 ≈ 100px
    //
    // EXTREME_YAW_THRESHOLD: How far nose can be toward an eye before invalid
    //   - 0.75 = nose 75% of the way from eye midpoint to one eye
    //   - Lower = stricter (require more centered head)
    //   - Higher = more lenient
    // =========================================================================
    static constexpr float REFERENCE_FACE_WIDTH = 100.0f;  // pixels at 40cm for 848x480 resolution
    static constexpr float REFERENCE_DISTANCE = 40.0f;     // cm (reference measurement distance)
    static constexpr float MIN_DISTANCE_CM = 12.0f;        // Too close threshold
    static constexpr float MAX_DISTANCE_CM = 60.0f;        // Too far threshold
    // Note: With camera rotated 90° CCW, the landmark coordinates swap X/Y
    // So this threshold may need adjustment. For now, keep high to avoid false extremes.
    static constexpr float EXTREME_YAW_THRESHOLD = 0.90f;  // 90% toward eye = extreme rotation
    static constexpr float SPIRAL_COMPLETE_ANGLE = 2.0f * 3.14159f;  // 360 degrees in radians
    static constexpr float MOTION_PAUSE_THRESHOLD = 0.005f; // Min angular velocity to count
    
    // Circular Motion Tracking with Face Validation
    struct MotionTracker {
        std::vector<cv::Point2f> nose_positions;
        float progress = 0.0f;           // 0.0 to 1.0
        float cumulative_angle = 0.0f;   // Total angular displacement
        float last_angle = 0.0f;         // Previous angle for delta calculation
        bool angle_initialized = false;  // First angle captured
        int no_face_counter = 0;
        FaceValidationState validation_state = FaceValidationState::NO_FACE;
        float estimated_distance_cm = 0.0f;
        
        void reset() {
            nose_positions.clear();
            progress = 0.0f;
            cumulative_angle = 0.0f;
            last_angle = 0.0f;
            angle_initialized = false;
            no_face_counter = 0;
            validation_state = FaceValidationState::NO_FACE;
            estimated_distance_cm = 0.0f;
        }
    } motion_tracker_;

    // Helper Methods
    void set_state(SystemState new_state);
    void process_frame(FrameBox* frame);
    
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
    std::string load_qr_shared_key();  // Load QR encryption key from config
    
    // Camera Management
    void configure_camera_for_state(SystemState state);
    
    // Network Callbacks
    void on_websocket_message(const std::string& message);
    
    // Logic
    float calculate_circular_motion_progress(const cv::Point2f& nose);
    bool load_device_config();
    bool check_wifi_on_boot();
    
    // Face Validation
    FaceValidationState validate_face_position(FrameBox* frame);
    float estimate_face_distance(float face_width_pixels);
    bool is_face_orientation_extreme(const std::vector<FrameBoxMetadata::Landmark>& landmarks);
    
    // Hardware Health Monitoring
    bool check_camera_health();
    bool check_display_health();
    bool check_network_health();
    void handle_hardware_error(const std::string& component, const std::string& error_msg);
};

} // namespace mdai
