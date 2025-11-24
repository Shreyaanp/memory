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
    PLACEHOLDER_SCREEN_7,    // Screen 7: [PLACEHOLDER - TO BE IMPLEMENTED LATER]
    READY,                   // Connected, waiting for user to start
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
    
    // Configuration
    std::string device_id_;
    std::string hardware_id_;
    std::string previous_ssid_;
    std::string previous_password_;
    std::string session_id_;
    std::string platform_id_;
    
    // Camera Mode Tracking
    enum class CameraMode { UNINITIALIZED, RGB_ONLY, FULL };
    std::atomic<CameraMode> current_camera_mode_{CameraMode::UNINITIALIZED};
    
    // Middleware Config
    std::string middleware_host_ = "mdai.mercle.ai"; // EC2 instance
    
    // Test Mode Flag (set to false for real verification)
    bool test_mode_ = true;  // TODO: Set to false when ready for production
    
    // Circular Motion Tracking
    struct MotionTracker {
        std::vector<cv::Point2f> nose_positions;
        float progress = 0.0f; // 0.0 to 1.0
        int no_face_counter = 0;
        
        void reset() {
            nose_positions.clear();
            progress = 0.0f;
            no_face_counter = 0;
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
};

} // namespace mdai
