#include "SystemController.hpp"
#include "TrustZoneIdentity.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <nlohmann/json.hpp>
#include <zbar.h>
#include <librealsense2/rs.hpp>  // For camera recovery device query

namespace mdai {

SystemController::SystemController() {
    ring_buffer_ = std::make_unique<DynamicRingBuffer>(32, 6ULL * 1024 * 1024 * 1024);
    
    SerialCommunicator::Config serial_cfg;
    // Auto-detect will be done by SerialCommunicator::connect() or try_reconnect()
    // Default to ttyACM0, but auto_detect_port() will find the correct port
    serial_cfg.port_name = "/dev/ttyACM0";  // Default, will be auto-detected on connect
    serial_comm_ = std::make_unique<SerialCommunicator>(serial_cfg);
    
    network_mgr_ = std::make_unique<NetworkManager>();
}

SystemController::~SystemController() {
    stop();
}

std::string SystemController::load_qr_shared_key() {
    // Try to load from device config first
    std::ifstream config_file("/opt/mdai/device_config.json");
    if (config_file.is_open()) {
        nlohmann::json config;
        try {
            config_file >> config;
            if (config.contains("qr_shared_key")) {
                std::string key = config["qr_shared_key"].get<std::string>();
                std::cout << "✅ Loaded QR shared key from config" << std::endl;
                return key;
            }
        } catch (const std::exception& e) {
            std::cerr << "⚠ Config parse error: " << e.what() << std::endl;
        }
        config_file.close();
    }
    
    // Fallback to environment variable
    const char* env_key = std::getenv("QR_SHARED_KEY");
    if (env_key) {
        std::cout << "✅ Loaded QR shared key from environment" << std::endl;
        return std::string(env_key);
    }
    
    std::cout << "⚠ No QR shared key found, using default (INSECURE!)" << std::endl;
    return "";  // Will use hardcoded fallback
}

bool SystemController::initialize() {
    // Load device configuration (keys, device_id, hardware_id)
    if (!load_device_config()) {
        std::cerr << "⚠ Device not registered. Run deploy-complete-system.sh first!" << std::endl;
        return false;
    }
    
    if (!serial_comm_->connect()) {
        std::cerr << "⚠ Serial connection failed" << std::endl;
    }
    
    face_detector_ = create_face_detector();
    if (!face_detector_) return false;

    network_mgr_->set_message_callback([this](const std::string& msg) {
        this->on_websocket_message(msg);
    });
    
    // Set connect callback to send auth IMMEDIATELY when WebSocket connects
    // This ensures auth is sent within milliseconds of handshake, not waiting for polling
    network_mgr_->set_connect_callback([this]() {
        std::lock_guard<std::mutex> lock(ws_auth_mutex_);
        if (!pending_ws_token_.empty()) {
            nlohmann::json auth_msg = {
                {"type", "auth"},
                {"bearer_token", pending_ws_token_},
                {"device_id", device_id_}
            };
            network_mgr_->send_message(auth_msg.dump());
            std::cout << "📤 [CONNECT CALLBACK] Sent auth immediately on connect" << std::endl;
        }
    });
    
    // UI Test Mode: Skip all screens, go directly to Screen 0 for tracking test
    if (ui_test_mode_) {
        std::cout << "🧪 UI TEST MODE: Starting Screen 0 (nose tracking test)" << std::endl;
        std::cout << "   - Screen 0: Tracking test" << std::endl;
        std::cout << "   - Screen 11: Success (spiral complete)" << std::endl;
        std::cout << "   - Screen 12: Error (5s no face) → 3s display → back to Screen 0" << std::endl;
        
        // Configure camera for RGB-ONLY mode (MediaPipe only needs RGB, no depth/IR needed)
        CameraConfig test_config;
        test_config.enable_depth = false;
        test_config.enable_ir = false;
        test_config.emitter_enabled = 0;
        test_config.laser_power = 0.0f;
        test_config.align_to_color = false;
        test_config.color_width = 848;   // Same as production FULL mode for consistency
        test_config.color_height = 480;
        test_config.color_fps = 30;
        test_config.depth_width = 0;
        test_config.depth_height = 0;
        test_config.enable_spatial_filter = false;
        test_config.enable_temporal_filter = false;
        test_config.enable_hole_filling = false;
        test_config.auto_exposure = true;
        
        std::cout << "📹 Camera: RGB-only mode (640x480@30fps) for MediaPipe test" << std::endl;
        
        // Start camera with RGB-only config
        producer_ = std::make_unique<Producer>(test_config, ring_buffer_.get());
        producer_->start();
        
        // Disable recording in ring buffer (not needed for tracking test)
        ring_buffer_->set_recording_active(false);
        
        current_camera_mode_.store(CameraMode::RGB_ONLY);
        
        // Go directly to ALIGN state with Screen 0
        set_state(SystemState::ALIGN);
        return true;
    }
    
    // Normal boot sequence: Show logo, then check WiFi
    set_state(SystemState::BOOT);
    
    return true;
}

void SystemController::run() {
    running_ = true;
    while (running_) {
        if (!producer_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // HIGHEST PRIORITY: Check WiFi connectivity
        // If WiFi disconnects during ANY state, immediately go to Screen 3 (WiFi QR scan)
        // If WiFi connects while in AWAIT_ADMIN_QR state, transition to PROVISIONED
        static int wifi_check_counter = 0;
        static int wifi_fail_count = 0;  // Track consecutive failures to avoid false positives
        const int WIFI_FAIL_THRESHOLD = 3;  // Require 3 consecutive failures (~1.5s) before aborting
        
        if (++wifi_check_counter >= 100) {  // Check every ~500ms (100 * 5ms) for faster detection
            wifi_check_counter = 0;
            
            if (network_mgr_->is_connected_to_internet()) {
                // WiFi is connected - reset failure counter
                if (wifi_fail_count > 0) {
                    std::cout << "✅ WiFi recovered after " << wifi_fail_count << " failed check(s)" << std::endl;
                }
                wifi_fail_count = 0;
                
                // Check if we need to transition from AWAIT_ADMIN_QR
                if (current_state_ == SystemState::AWAIT_ADMIN_QR) {
                    std::string ssid = network_mgr_->get_current_ssid();
                    std::string ip = network_mgr_->get_ip_address();
                    std::cout << "✅ WiFi connected while in AWAIT_ADMIN_QR: " << ssid << " (" << ip << ")" << std::endl;
                    std::cout << "🔄 Transitioning to PROVISIONED state" << std::endl;
                    // Transition to PROVISIONED state which will show Screen 4 → Screen 5 → Screen 6
                    set_state(SystemState::PROVISIONED);
                }
            } else {
                // WiFi check failed - increment counter
                wifi_fail_count++;
                if (wifi_fail_count < WIFI_FAIL_THRESHOLD) {
                    std::cout << "⚠️  WiFi check failed (" << wifi_fail_count << "/" << WIFI_FAIL_THRESHOLD << ") - waiting..." << std::endl;
                }
                
                // Only abort after multiple consecutive failures
                if (wifi_fail_count >= WIFI_FAIL_THRESHOLD &&
                    current_state_ != SystemState::AWAIT_ADMIN_QR && 
                    current_state_ != SystemState::BOOT &&
                    current_state_ != SystemState::ERROR &&
                    current_state_ != SystemState::LOGOUT) {
                    // WiFi truly disconnected - ABORT
                    std::cerr << "⚠️  CRITICAL: WiFi LOST (" << wifi_fail_count << " consecutive failures) - ABORTING" << std::endl;
                    wifi_fail_count = 0;  // Reset for next time
                    
                    // Clean up current session
                    session_id_.clear();
                    platform_id_.clear();
                    motion_tracker_.reset();
                    ring_buffer_->set_recording_active(false);
                    ring_buffer_->clear();
                    
                    // Disconnect WebSocket if active
                    if (network_mgr_->is_connected()) {
                        network_mgr_->disconnect();
                    }
                    
                    // Go directly to Screen 3 (Looking for WiFi)
                    serial_comm_->send_state(3);
                    
                    // Go to AWAIT_ADMIN_QR state
                    set_state(SystemState::AWAIT_ADMIN_QR);
                    std::cout << "📡 WiFi lost - showing Screen 3 (Looking for WiFi)" << std::endl;
                }
            }
        }

        // Check state timer (for automatic transitions like ERROR -> IDLE, LOGOUT -> IDLE)
        if (state_timer_active_) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - state_timer_start_
            ).count();
            
            if (elapsed >= state_timer_duration_ms_) {
                state_timer_active_ = false;
                std::cout << "🔄 Timer expired (" << elapsed << "ms) - transitioning from " 
                          << static_cast<int>(current_state_.load()) << " to " 
                          << static_cast<int>(state_timer_target_) << std::endl;
                set_state(state_timer_target_);
            } else {
                // Debug: Log timer progress every second
                static int timer_log_counter = 0;
                if (++timer_log_counter >= 200) {  // Every ~1 second
                    timer_log_counter = 0;
                    std::cout << "⏳ Timer active: " << elapsed << "/" << state_timer_duration_ms_ 
                              << "ms elapsed (state: " << static_cast<int>(current_state_.load()) << ")" << std::endl;
                }
            }
        }

        // CAMERA HEALTH MONITORING - Check for camera errors
        // Only check during states where camera recovery makes sense
        // Skip during: BOOT, COUNTDOWN, PROCESSING (don't interrupt critical operations)
        static int camera_health_counter = 0;
        SystemState health_check_state = current_state_.load();
        bool skip_health_check = (health_check_state == SystemState::BOOT ||
                                  health_check_state == SystemState::COUNTDOWN ||
                                  health_check_state == SystemState::PROCESSING ||
                                  health_check_state == SystemState::PROVISIONING);
        
        if (!skip_health_check && ++camera_health_counter >= 400) {  // Check every ~2 seconds (400 * 5ms)
            camera_health_counter = 0;
            
            // Check if error was reported via callback
            if (camera_error_detected_.load()) {
                std::cerr << "🔧 Camera error detected via callback: " << camera_error_message_ << std::endl;
                camera_error_detected_.store(false);  // Reset flag before handling
                handle_hardware_error("Camera", camera_error_message_);
            }
        }

        FrameBox* frame = ring_buffer_->get_latest_frame();
        if (frame) {
            process_frame(frame);
            ring_buffer_->release_frame(frame);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

void SystemController::stop() {
    running_ = false;
    if (producer_) producer_->stop();
    if (network_mgr_) network_mgr_->disconnect();
}

void SystemController::set_state(SystemState new_state) {
    current_state_ = new_state;
    
    switch (new_state) {
        case SystemState::BOOT:
            std::thread([this]() { handle_boot(); }).detach();
            break;
            
        case SystemState::AWAIT_ADMIN_QR:
            serial_comm_->send_state(3); // Screen 3: "Scan WiFi QR" (only if WiFi not connected)
            configure_camera_for_state(SystemState::AWAIT_ADMIN_QR);
            break;
            
        case SystemState::PROVISIONING:
            serial_comm_->send_state(2); // Screen 2: "Connecting to WiFi" (reuse boot screen)
            break;
            
        case SystemState::PROVISIONED: {
            // Get SSID for Screen 4 display
            std::string ssid = network_mgr_->get_current_ssid();
            if (ssid.empty()) ssid = "WiFi Connected";
            
            serial_comm_->send_state_with_text(4, ssid); // Screen 4: "WiFi Connected" screen (show SSID, 3s)
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(3));
                // Show Screen 5: "mDai Ready" for 3 seconds before going to IDLE
                serial_comm_->send_state(5); // Screen 5: mDai Ready (transition screen)
                std::cout << "✅ Showing 'mDai Ready' transition screen" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(3));
                // Transition to IDLE (which will show Screen 6: Scan Session QR)
                std::cout << "🔄 Transitioning to IDLE state" << std::endl;
                set_state(SystemState::IDLE);
            }).detach();
            break;
        }
            
        case SystemState::IDLE:
            serial_comm_->send_state(6); // Screen 6: "Scan Session QR" (MAIN IDLE STATE)
            configure_camera_for_state(SystemState::IDLE);
            break;
            
        case SystemState::READY:
            serial_comm_->send_state(7); // Screen 7: "Ready to Start"
            
            // Switch camera to FULL mode now (RGB+Depth+IR+Emitter)
            // This gives camera time to warm up/stabilize before countdown starts
            std::cout << "📹 READY: Switching camera to FULL mode for warmup..." << std::endl;
            configure_camera_for_state(SystemState::READY);
            std::cout << "📹 Camera in FULL mode (Depth+IR+Emitter ON) - warming up" << std::endl;
            
            // Recording stays OFF until ALIGN state (Screen 9)
            // Camera stabilizes during READY and COUNTDOWN screens
            
            // Safety Timeout: If user doesn't start within 30 seconds, go back to IDLE
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(30));
                if (current_state_ == SystemState::READY) {
                    std::cout << "⚠️  Session Timeout (User didn't start)" << std::endl;
                    network_mgr_->send_message("{\"type\":\"error\", \"code\":\"timeout_ready\", \"message\":\"User did not start verification\"}");
                    network_mgr_->disconnect();
                    set_state(SystemState::IDLE);
                }
            }).detach();
            break;
            
        case SystemState::COUNTDOWN:
            // Show Screen 8 and start countdown from 5 to 1
            std::cout << "🔢 COUNTDOWN state entered - showing Screen 8" << std::endl;
            // First send screen state to ensure Screen 8 is displayed
            if (serial_comm_->send_state(8)) {
                std::cout << "✅ Screen 8 state sent successfully" << std::endl;
            } else {
                std::cerr << "❌ Failed to send Screen 8 state" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Small delay for screen to initialize
            if (serial_comm_->send_state_with_text(8, "5")) {
                std::cout << "✅ Screen 8 text '5' sent successfully" << std::endl;
            } else {
                std::cerr << "❌ Failed to send Screen 8 text" << std::endl;
            }
            std::cout << "🔢 Starting countdown: 5..." << std::endl;
            
            // Countdown thread: 5, 4, 3, 2, 1
            std::thread([this]() {
                for (int count = 5; count >= 1; count--) {
                    if (current_state_ != SystemState::COUNTDOWN) {
                        std::cout << "⚠️  Countdown interrupted (state changed)" << std::endl;
                        return;
                    }
                    
                    // Update screen with current countdown number
                    std::string count_str = std::to_string(count);
                    serial_comm_->send_state_with_text(8, count_str);
                    std::cout << "🔢 Countdown: " << count << std::endl;
                    
                    // Wait 1 second before next number (except after showing "1")
                    if (count > 1) {
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                }
                
                // After countdown completes, transition directly to ALIGN (Screen 9)
                if (current_state_ == SystemState::COUNTDOWN) {
                    std::cout << "✅ Countdown complete → ALIGN" << std::endl;
                    set_state(SystemState::ALIGN);
                }
            }).detach();
            break;
            
        case SystemState::WARMUP:
            serial_comm_->send_state(7); // Screen 7: Warmup (prepare for nose tracking)
            // Camera already switched to Full in READY state
            // Just wait for camera to stabilize (2-3 seconds)
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(3));
                if (current_state_ == SystemState::WARMUP) {
                    std::cout << "✅ Camera warmup complete → ALIGN" << std::endl;
                    set_state(SystemState::ALIGN);
                }
            }).detach();
            break;
            
        case SystemState::ALIGN:
            if (ui_test_mode_) {
                serial_comm_->send_state(0); // Screen 0: UI test page
                std::cout << "🧪 UI TEST: Screen 0 active - tracking started" << std::endl;
                // In test mode, don't enable recording (we just want to test tracking)
                ring_buffer_->set_recording_active(false);
            } else {
                serial_comm_->send_state(9); // Screen 9: Production nose tracking
                ring_buffer_->set_recording_active(true);
            }
            motion_tracker_.reset();
            ring_buffer_->clear();
            break;
            
        case SystemState::PROCESSING:
            ring_buffer_->set_recording_active(false); 
            serial_comm_->send_state(10); // Screen 10: "Processing" screen
            std::thread([this]() { handle_processing(); }).detach();
            break;
            
        case SystemState::SUCCESS:
            serial_comm_->send_state(11); // Screen 11: "Success Animation" (green, 3 seconds)
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(3));
                if (ui_test_mode_) {
                    std::cout << "🧪 UI TEST: Success complete → back to Screen 0" << std::endl;
                    set_state(SystemState::ALIGN); // Return to Screen 0 for more testing
                } else {
                set_state(SystemState::LOGOUT);
                }
            }).detach();
            break;
            
        case SystemState::ERROR:
            if (ui_test_mode_) {
                // In UI test mode, skip error screen and go directly back to Screen 0
                std::cout << "🧪 UI TEST: Error occurred - resetting to Screen 0" << std::endl;
                motion_tracker_.reset();
                set_state(SystemState::ALIGN); // Return to Screen 0 immediately
                break;
            }
            
            serial_comm_->send_state(12); // Screen 12: "Error Animation" (red, 3 seconds, returns to IDLE)
            std::cout << "❌ ERROR state entered - showing error for 3 seconds..." << std::endl;
            // Error message should already be sent via send_error() before entering ERROR state
            
            // Clean up session on error
            session_id_.clear();
            platform_id_.clear();
            motion_tracker_.reset();
            
            // Disconnect WebSocket if connected and clear auth token
            {
                std::lock_guard<std::mutex> lock(ws_auth_mutex_);
                pending_ws_token_.clear();
            }
            if (network_mgr_ && network_mgr_->is_connected()) {
                network_mgr_->disconnect();
                std::cout << "✅ WebSocket disconnected due to error" << std::endl;
            }
            
            configure_camera_for_state(SystemState::IDLE);
            
            // Start timer for automatic return (3 seconds)
            state_timer_start_ = std::chrono::steady_clock::now();
            state_timer_target_ = SystemState::IDLE;
            state_timer_duration_ms_ = 3000; // 3 seconds
            state_timer_active_ = true;
            std::cout << "⏰ Timer set: will return to IDLE in 3 seconds" << std::endl;
            std::cout.flush();  // Force immediate log write
            break;
            
        case SystemState::LOGOUT:
            serial_comm_->send_state(15); // Screen 15: "Deleting Data / Thank You"
            std::cout << "👋 Showing Thank You screen - returning to IDLE in 4 seconds..." << std::endl;
            std::cout.flush();  // Force immediate write
            
            // TODO: Sync mobile and device (lasts 4 seconds)
            // TODO: After sync, send ping to WebSocket to trigger server→mobile data transfer
            // TODO: When we receive confirmation message, return to IDLE screen (Screen 6)
            
            // Clean up session
            session_id_.clear();
            platform_id_.clear();
            motion_tracker_.reset();
            
            // Close WebSocket connection and clear auth token
            {
                std::lock_guard<std::mutex> lock(ws_auth_mutex_);
                pending_ws_token_.clear();
            }
            if (network_mgr_) {
            network_mgr_->disconnect();
                std::cout << "✅ WebSocket disconnected" << std::endl;
                std::cout.flush();
            }
            
            // Switch camera back: Full → RGB-only (low power mode for QR scanning)
            configure_camera_for_state(SystemState::IDLE);
            std::cout << "🔄 Camera mode: Full → RGB-only (QR scanning mode)" << std::endl;
            std::cout.flush();
            
            // Start timer for automatic return to IDLE (4 seconds for data sync)
            std::cout << "⏰ Setting timer..." << std::endl;
            std::cout.flush();
            state_timer_start_ = std::chrono::steady_clock::now();
            state_timer_target_ = SystemState::IDLE;
            state_timer_duration_ms_ = 4000; // 4 seconds
            state_timer_active_ = true;
            std::cout << "⏰ Timer ACTIVE - will return to IDLE in 4 seconds" << std::endl;
            std::cout.flush();
            break;
            
        default:
            break;
    }
}

void SystemController::configure_camera_for_state(SystemState state) {
    CameraConfig config;
    CameraMode target_mode;
    
    // Determine target camera mode based on state
    // LOW POWER (RGB-only): Boot, WiFi setup, Idle (QR scanning)
    // FULL MODE (RGB+Depth+IR): Ready, Countdown, Align, Processing, etc.
    // 
    // Resolution: ALL streams use 848x480 for consistency
    // This matches RealSense default and ensures proper alignment between RGB/Depth/IR
    
    if (state == SystemState::BOOT ||
        state == SystemState::AWAIT_ADMIN_QR || 
        state == SystemState::IDLE ||
        // READY removed - now triggers FULL mode for camera warmup before countdown
        state == SystemState::WIFI_CHANGE_CONNECTING ||
        state == SystemState::WIFI_CHANGE_SUCCESS ||
        state == SystemState::WIFI_CHANGE_FAILED) {
        
        target_mode = CameraMode::RGB_ONLY;
        config.enable_ir = false;
        config.enable_depth = false;
        config.emitter_enabled = 0;
        config.laser_power = 0.0f;
        config.align_to_color = false;
        config.color_width = 848;    // Same as FULL mode & IR for consistency
        config.color_height = 480;
        config.color_fps = 30;
        config.depth_width = 0;
        config.depth_height = 0;
        config.enable_spatial_filter = false;
        config.enable_temporal_filter = false;
        config.enable_hole_filling = false;
        config.auto_exposure = true;  // Let the camera adapt lighting for QR readability
        
    } else {
        // FULL MODE: READY, COUNTDOWN, WARMUP, ALIGN, PROCESSING
        // Camera switches to full mode at READY so it can warm up during "Ready?" screen
        // All streams at 848x480 for proper alignment
        target_mode = CameraMode::FULL;
        config.enable_ir = true;
        config.enable_depth = true;
        config.depth_width = 848;
        config.depth_height = 480;
        config.color_width = 848;    // Same as Depth/IR for proper alignment
        config.color_height = 480;
        config.color_fps = 30;
        config.ir_width = 848;
        config.ir_height = 480;
        config.ir_fps = 30;
        config.align_to_color = true;
        config.emitter_enabled = 1;
        config.laser_power = 300.0f;
        config.enable_spatial_filter = true;
        config.enable_temporal_filter = true;
        config.enable_hole_filling = true;
    }
    
    
    // Only reconfigure if changing modes (low power → high power or vice versa)
    if (current_camera_mode_.load() == target_mode && producer_) {
        std::cout << "📹 Camera already in correct mode - no reconfiguration needed" << std::endl;
        return;
    }
    
    // **CRITICAL FIX**: Only stop camera when switching from LOW to HIGH power
    // LOW POWER mode should NEVER be closed once started
    bool use_quick_init = false;
    
    if (current_camera_mode_.load() == CameraMode::RGB_ONLY && target_mode == CameraMode::FULL) {
        // Switching from low power to high power - camera already warm, use quick init
        std::cout << "📹 Camera: Upgrading from Low Power → High Power (quick init)" << std::endl;
        use_quick_init = true;  // Camera is already warm
        if (producer_) {
            producer_->stop();
            // Reduced wait time since camera is warm
            std::cout << "⏳ Waiting 1 second for camera hardware to release..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } else if (current_camera_mode_.load() == CameraMode::FULL && target_mode == CameraMode::RGB_ONLY) {
        // Switching from high power to low power - stop high power mode
        std::cout << "📹 Camera: Downgrading from High Power → Low Power (quick init)" << std::endl;
        use_quick_init = true;  // Camera is already warm
        if (producer_) {
            producer_->stop();
            // Reduced wait time since camera is warm
            std::cout << "⏳ Waiting 1 second for camera hardware to release..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } else if (current_camera_mode_.load() == CameraMode::UNINITIALIZED) {
        // First time initialization - full warmup needed
        std::cout << "📹 Camera: Initial startup in " << (target_mode == CameraMode::RGB_ONLY ? "Low Power" : "High Power") << " mode" << std::endl;
    }
    
    // Set quick init flag
    config.quick_init = use_quick_init;
    
    // Log mode change
    if (target_mode == CameraMode::RGB_ONLY) {
        std::cout << "📹 Camera: RGB-only mode (" << config.color_width << "x" 
                  << config.color_height << "@" << config.color_fps << "fps) - Low Power - PERSISTENT" << std::endl;
    } else {
        std::cout << "📹 Camera: Full mode (" << config.color_width << "x" << config.color_height 
                  << " RGB + " << config.depth_width << "x" << config.depth_height << " Depth + IR + Laser)" << std::endl;
    }
    
    producer_ = std::make_unique<Producer>(config, ring_buffer_.get());
    
    // Set up error callback for camera health monitoring
    producer_->set_error_callback([this](const std::string& error) {
        std::cerr << "📹 Camera Error: " << error << std::endl;
        camera_error_detected_.store(true);
        camera_error_message_ = error;
    });
    
    producer_->set_status_callback([this](const std::string& status) {
        std::cout << "📹 Camera Status: " << status << std::endl;
    });
    
    producer_->start();
    
    // Update current mode tracker
    current_camera_mode_.store(target_mode);
}

void SystemController::process_frame(FrameBox* frame) {
    SystemState state = current_state_.load();
    switch (state) {
        case SystemState::AWAIT_ADMIN_QR: handle_await_admin_qr(frame); break;
        case SystemState::IDLE: handle_idle(frame); break;
        case SystemState::READY: handle_idle(frame); break;  // READY also scans for session QR
        case SystemState::COUNTDOWN: break;  // COUNTDOWN doesn't need frame processing
        case SystemState::WARMUP: handle_warmup(frame); break;
        case SystemState::ALIGN: handle_align(frame); break;
        // BOOT and other states don't need frame processing
        default: break;
    }
}

void SystemController::handle_await_admin_qr(FrameBox* frame) {
#ifdef HAVE_OPENCV
    static int frame_skip = 0;
    if (frame_skip++ % 5 != 0) return; // Process every 5th frame for efficiency
    
    cv::Mat gray;
    cv::cvtColor(frame->get_color_mat(), gray, cv::COLOR_BGR2GRAY);
    cv::QRCodeDetector qr_decoder;
    std::string decoded_info = qr_decoder.detectAndDecode(gray);
    
    if (!decoded_info.empty()) {
        std::cout << "📱 QR Code Detected (Admin/WiFi)" << std::endl;
        
        // This is a WiFi provisioning QR with encrypted challenge-response
        if (handle_wifi_qr(decoded_info)) {
            std::cout << "✅ WiFi Provisioning Complete" << std::endl;
        } else {
            std::cout << "⚠ WiFi Provisioning Failed" << std::endl;
        }
    }
#endif
}

void SystemController::handle_idle(FrameBox* frame) {
#ifdef HAVE_OPENCV
    // IMPORTANT: Re-check state before processing - state might have changed
    SystemState current = current_state_.load();
    if (current != SystemState::IDLE && current != SystemState::READY) {
        return;  // Don't process QR codes if we're not in IDLE/READY
    }
    
    // Debounce: Don't process QR if we recently processed one
    static auto last_qr_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_qr_time).count();
    
    static int frame_skip = 0;
    static int scan_count = 0;
    if (frame_skip++ % 5 != 0) return; // Process every 5th frame

    scan_count++;
    if (scan_count % 50 == 0) {
        std::cout << "[QR Scanner] Active - scan #" << scan_count << " (ZBar ready)" << std::endl;
    }

    cv::Mat color_img = frame->get_color_mat();
    if (color_img.empty()) {
        return;
    }
    
    cv::Mat gray;
    cv::cvtColor(color_img, gray, cv::COLOR_BGR2GRAY);
    
    // Use ZBar with enhanced settings for better QR detection
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_X_DENSITY, 1);  // Scan every pixel
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_Y_DENSITY, 1);  // Scan every line
    
    // Try to enhance contrast for better QR detection
    cv::Mat enhanced;
    cv::equalizeHist(gray, enhanced);  // Improve contrast
    
    zbar::Image zbar_image(enhanced.cols, enhanced.rows, "Y800", enhanced.data, enhanced.cols * enhanced.rows);
    int n = scanner.scan(zbar_image);
    
    // Debug: Save enhanced frame every 100 scans
    if (scan_count % 100 == 0) {
        cv::imwrite("/tmp/qr_enhanced_latest.jpg", enhanced);
        std::cout << "[QR Debug] Saved enhanced frame to /tmp/qr_enhanced_latest.jpg" << std::endl;
    }
    
    if (scan_count % 50 == 0 && n == 0) {
        // Periodically save frame for debugging
        static int save_count = 0;
        if (save_count++ < 3) {
            cv::imwrite("/tmp/qr_scan_debug_" + std::to_string(save_count) + ".jpg", gray);
            std::cout << "[QR Debug] Saved frame to /tmp/qr_scan_debug_" << save_count << ".jpg for inspection" << std::endl;
        }
    }
    
    std::string decoded_info;
    if (n > 0) {
        std::cout << "🎯 ZBar detected " << n << " QR code(s)!" << std::endl;
        for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
             symbol != zbar_image.symbol_end(); ++symbol) {
            decoded_info = symbol->get_data();
            std::cout << "📱 QR Data length: " << decoded_info.length() << " bytes" << std::endl;
            
            // 📸 SAVE QR IMAGE FOR VERIFICATION
            static int qr_capture_count = 0;
            std::string qr_image_path = "/tmp/qr_captured_" + std::to_string(++qr_capture_count) + ".jpg";
            cv::imwrite(qr_image_path, color_img);
            std::cout << "📸 QR Image saved: " << qr_image_path << std::endl;
            
            break; // Take first QR code
        }
    }
    
    if (!decoded_info.empty()) {
        // Debounce: Ignore QR if we processed one in the last 5 seconds
        if (elapsed < 5000) {
            return;  // Too soon after last QR, skip
        }
        last_qr_time = std::chrono::steady_clock::now();
        
        std::cout << "📱 QR Code Detected (Idle State)" << std::endl;
        
        // FILTER: Ignore tiny QR codes (false positives from ZBar)
        if (decoded_info.length() < 20) {
            std::cout << "⚠️  Ignoring tiny QR code (" << decoded_info.length() << " bytes) - likely false positive" << std::endl;
            return; // Skip processing
        }
        
        // STEP 1: Detect QR type by prefix or structure
        // Session QR: "S:" prefix or JSON with qr_encrypted
        // WiFi QR: "W:" prefix or JSON without qr_encrypted
        bool is_session_qr = false;
        bool is_wifi_qr = false;
        std::string encrypted_data;
        
        // Check for explicit prefix first (new format)
        if (decoded_info.length() > 2) {
            std::string prefix = decoded_info.substr(0, 2);
            if (prefix == "S:") {
                // Session QR with "S:" prefix
                is_session_qr = true;
                encrypted_data = decoded_info.substr(2);  // Remove "S:" prefix
                std::cout << "🔐 Detected: SESSION QR (S: prefix, " << encrypted_data.length() << " chars)" << std::endl;
            } else if (prefix == "W:") {
                // WiFi QR with "W:" prefix
                is_wifi_qr = true;
                encrypted_data = decoded_info.substr(2);  // Remove "W:" prefix
                std::cout << "🔐 Detected: WIFI QR (W: prefix, " << encrypted_data.length() << " chars)" << std::endl;
            }
        }
        
        // If no prefix, try legacy JSON format detection
        if (!is_session_qr && !is_wifi_qr) {
            try {
                nlohmann::json qr_json = nlohmann::json::parse(decoded_info);
                
                if (qr_json.contains("qr_encrypted")) {
                    // Legacy Session QR (JSON format)
                    is_session_qr = true;
                    encrypted_data = qr_json["qr_encrypted"].get<std::string>();
                    std::cout << "🔐 Detected: SESSION QR (legacy JSON format)" << std::endl;
                } else {
                    // Legacy WiFi QR (JSON format without qr_encrypted)
                    is_wifi_qr = true;
                    encrypted_data = decoded_info;
                    std::cout << "🔐 Detected: WIFI QR (legacy JSON format)" << std::endl;
                }
            } catch (...) {
                // Not JSON and no prefix - check if it's legacy raw Session QR (< 100 chars)
                if (decoded_info.length() < 100 && decoded_info.length() > 20) {
                    // Likely legacy Session QR without prefix
                    is_session_qr = true;
                    encrypted_data = decoded_info;
                    std::cout << "🔐 Detected: SESSION QR (legacy raw format, " << decoded_info.length() << " chars)" << std::endl;
                } else {
                    // Unknown format - show error
                    std::cerr << "⚠ Unknown QR format (length: " << decoded_info.length() << ")" << std::endl;
                    serial_comm_->send_error("Invalid QR format");
                    serial_comm_->send_state(12); // Screen 12: Error Animation
                    // Return to IDLE after 3 seconds
                    state_timer_start_ = std::chrono::steady_clock::now();
                    state_timer_target_ = SystemState::IDLE;
                    state_timer_duration_ms_ = 3000;
                    state_timer_active_ = true;
                    return;
                }
            }
        }
        
        // STEP 2: Handle Session QR (from mobile app)
        if (is_session_qr) {
            // Prefer configured key, fallback to legacy default
            std::string qr_key = load_qr_shared_key();
            if (qr_key.empty()) {
                // Production encryption key (must match EC2 server's QR_ENCRYPTION_KEY)
                qr_key = "ad5295ee97afce972b8ac5fb9d8314b33578cf85f22e84429939c4ea981d9320";
            }
            
            try {
                // Decrypt QR (no compression, no JSON - just raw token!)
                std::string decrypted_data = CryptoUtils::aes256_decrypt(encrypted_data, qr_key);
                std::cout << "✅ QR decrypted successfully" << std::endl;
                
                // Check if decrypted data has "S:" prefix (new format)
                std::string ws_token;
                if (decrypted_data.length() > 2 && decrypted_data.substr(0, 2) == "S:") {
                    ws_token = decrypted_data.substr(2);  // Strip "S:" prefix
                    std::cout << "🔐 Detected: New format with S: prefix" << std::endl;
                } else {
                    ws_token = decrypted_data;  // Legacy format without prefix
                    std::cout << "🔐 Detected: Legacy format without prefix" << std::endl;
                }
                
                // Validate token is 8 chars - silently ignore invalid QR
                if (ws_token.empty() || ws_token.length() != 8) {
                    std::cerr << "⚠ Invalid token in QR (expected 8 chars, got " << ws_token.length() << ") - ignoring" << std::endl;
                    return;  // Silent ignore - keep scanning
                }
                
                std::cout << "🔐 Token (HMAC): " << ws_token << std::endl;
                
                // Store token for auto-auth on connect (and reconnect)
                {
                    std::lock_guard<std::mutex> lock(ws_auth_mutex_);
                    pending_ws_token_ = ws_token;
                }
                
                // Connect to WebSocket (async - wait for connection)
                std::cout << "🔌 Connecting to WebSocket..." << std::endl;
                
                // No session_id in path - server will lookup by ws_token
                std::string ws_path = "/ws/device";
                
                // Start connection (spawns background thread)
                // Auth will be sent automatically via connect_callback
                network_mgr_->connect_to_middleware(middleware_host_, 443, ws_path, device_id_);
                
                // Wait for connection to be established (max 10 seconds)
                int wait_count = 0;
                while (!network_mgr_->is_connected() && wait_count < 100) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    wait_count++;
                }
                
                if (network_mgr_->is_connected()) {
                    std::cout << "✅ WebSocket connected (auth sent via callback)" << std::endl;
                    // Auth already sent via connect_callback - just wait for auth_success
                    // State will change to READY after receiving auth_success from server
                } else {
                    std::cerr << "❌ WebSocket connection failed (timeout)" << std::endl;
                    serial_comm_->send_error("Connection failed");
                    serial_comm_->send_state(12); // Screen 12: Error Animation
                    // Disconnect and return to IDLE after 3 seconds
                    network_mgr_->disconnect();
                    state_timer_start_ = std::chrono::steady_clock::now();
                    state_timer_target_ = SystemState::IDLE;
                    state_timer_duration_ms_ = 3000;
                    state_timer_active_ = true;
                }
                
            } catch (const std::exception& e) {
                // Silent ignore - not our QR code (decryption failed means wrong key = not from our server)
                std::cerr << "⚠ QR decryption failed (not our QR) - ignoring: " << e.what() << std::endl;
                // Don't show error screen, just continue scanning
            }
        }
        // STEP 3: Handle WiFi QR (from admin portal)
        else if (is_wifi_qr) {
            // User scanned WiFi QR in IDLE state - wrong QR type!
            std::cerr << "⚠ WiFi QR scanned in IDLE state - ignoring (need Session QR)" << std::endl;
            // Just ignore it - don't show error screen for wrong QR type
            return;
        }
    }
#endif
}

void SystemController::handle_warmup(FrameBox* /* frame */) {
    static int warmup_frames = 0;
    static SystemState last_state = SystemState::BOOT;
    
    // Reset counter when entering WARMUP state
    if (last_state != SystemState::WARMUP) {
        warmup_frames = 0;
        last_state = SystemState::WARMUP;
    }
    
    // Wait for 2 seconds (60 frames @ 30fps) for camera to stabilize
    if (warmup_frames++ > 60) {
        if (warmup_frames == 62) {
             network_mgr_->send_message("{\"type\":\"status\", \"payload\":\"warmed_up\"}");
            std::cout << "✅ Camera Warmed Up - Waiting for App Start..." << std::endl;
        }
    }
}

void SystemController::handle_align(FrameBox* frame) {
    // Safety check: if frame is null, camera may have disconnected
    if (!frame) {
        std::cerr << "⚠️  Null frame in ALIGN - camera may be disconnected" << std::endl;
        motion_tracker_.no_face_counter++;
        if (motion_tracker_.no_face_counter > 150) {  // ~5 seconds
            std::cerr << "❌ Camera disconnected during ALIGN" << std::endl;
            serial_comm_->send_error("Camera disconnected");
            set_state(SystemState::ERROR);
        }
        return;
    }
    
    // Startup grace period: 15 seconds (450 frames @ 30fps) before enabling face timeout
    // This allows camera to stabilize and MediaPipe to initialize properly
    static int startup_frames = 0;
    static const int STARTUP_GRACE_PERIOD = 450;  // 15 seconds
    static const int FACE_TIMEOUT_FRAMES = 150;   // 5 seconds after grace period
    
    startup_frames++;
    bool in_grace_period = (startup_frames < STARTUP_GRACE_PERIOD);
    
    // Debug: Log frame info periodically
    static int frame_count = 0;
    if (++frame_count % 60 == 0) {
        std::cout << "📷 Frame #" << frame_count 
                  << " size=" << frame->color_width << "x" << frame->color_height;
        if (in_grace_period) {
            std::cout << " [STARTUP: " << (STARTUP_GRACE_PERIOD - startup_frames) / 30 << "s remaining]";
        }
        std::cout << std::endl;
    }
    
    // Run face detection
    if (!face_detector_->detect(frame)) {
        motion_tracker_.no_face_counter++;
        motion_tracker_.validation_state = FaceValidationState::NO_FACE;
        
        // Debug: Log no face periodically
        if (motion_tracker_.no_face_counter % 30 == 0) {
            std::cout << "👤 No face detected (count=" << motion_tracker_.no_face_counter << ")";
            if (in_grace_period) {
                std::cout << " [grace period - no timeout]";
            }
            std::cout << std::endl;
        }
        
        // Send grey target (no face)
        serial_comm_->send_tracking_data(233, 233, 
            static_cast<int>(motion_tracker_.progress * 100), false);
        
        // Check timeout only after grace period
        if (!in_grace_period && motion_tracker_.no_face_counter > FACE_TIMEOUT_FRAMES) {
            handle_face_timeout();
            startup_frames = 0;  // Reset startup counter on timeout
        }
        return;
    }
    
    // Get landmarks and check size FIRST (before any access)
    auto& landmarks = frame->metadata.landmarks;
    if (landmarks.size() < 468) {  // MediaPipe Face Mesh has 468 landmarks
        motion_tracker_.no_face_counter++;
        motion_tracker_.validation_state = FaceValidationState::NO_FACE;
        
        // Debug: Log landmark count issue
        if (motion_tracker_.no_face_counter % 30 == 0) {
            std::cout << "⚠️ Face detected but only " << landmarks.size() << " landmarks (need 468)" << std::endl;
        }
        
        serial_comm_->send_tracking_data(233, 233, 
            static_cast<int>(motion_tracker_.progress * 100), false);
        return;
    }
    
    // Debug: Log successful face detection
    static int face_detect_count = 0;
    if (++face_detect_count % 60 == 0) {
        std::cout << "✅ Face detected with " << landmarks.size() << " landmarks" << std::endl;
    }
    
    // Validate face position (distance + orientation) - now safe to access landmarks
    motion_tracker_.validation_state = validate_face_position(frame);
    bool is_valid = (motion_tracker_.validation_state == FaceValidationState::VALID);
    
    // Reset no-face counter since we have a face
        if (frame->metadata.face_detected) {
            motion_tracker_.no_face_counter = 0;
    } else {
        motion_tracker_.no_face_counter++;
    }
    
    // Extract normalized nose position (0.0-1.0)
    float nose_x_norm = landmarks[4].x / frame->color_width;
    float nose_y_norm = landmarks[4].y / frame->color_height;
                
    // Convert to screen coordinates (466x466 display)
    int raw_x = static_cast<int>(nose_x_norm * 466.0f);
    int raw_y = static_cast<int>(nose_y_norm * 466.0f);
    
    // Apply IMU-based rotation correction for camera orientation
    int screen_x, screen_y;
    if (producer_) {
        producer_->transform_coordinates(raw_x, raw_y, 465, 465, screen_x, screen_y);
    } else {
        screen_x = raw_x;
        screen_y = raw_y;
    }
    
    // Clamp to display bounds
    screen_x = std::max(0, std::min(465, screen_x));
    screen_y = std::max(0, std::min(465, screen_y));
    
    // Calculate progress only if face position is valid
    float progress = motion_tracker_.progress;
    if (is_valid) {
        progress = calculate_circular_motion_progress(cv::Point2f(nose_x_norm, nose_y_norm));
        
        // Debug logging every 30 frames (~1 second)
        static int debug_counter = 0;
        if (++debug_counter % 30 == 0) {
            std::cout << "🎯 Tracking: x=" << screen_x << " y=" << screen_y 
                      << " progress=" << static_cast<int>(progress * 100) << "%"
                      << " angle=" << static_cast<int>(motion_tracker_.cumulative_angle * 180.0f / 3.14159f) << "°"
                      << " dist=" << static_cast<int>(motion_tracker_.estimated_distance_cm) << "cm"
                      << " valid=" << (is_valid ? "YES" : "NO") << std::endl;
        }
    }
    
    // Send combined tracking data to display
    serial_comm_->send_tracking_data(screen_x, screen_y, 
        static_cast<int>(progress * 100), is_valid);
    
    // Debug: Log tracking even when invalid (every 60 frames)
    static int track_debug = 0;
    if (++track_debug % 60 == 0) {
        std::string val_str;
        switch (motion_tracker_.validation_state) {
            case FaceValidationState::VALID: val_str = "VALID"; break;
            case FaceValidationState::NO_FACE: val_str = "NO_FACE"; break;
            case FaceValidationState::TOO_CLOSE: val_str = "TOO_CLOSE"; break;
            case FaceValidationState::TOO_FAR: val_str = "TOO_FAR"; break;
            case FaceValidationState::EXTREME_ROTATION: val_str = "EXTREME_ROT"; break;
        }
        std::cout << "📡 Tracking sent: X:" << screen_x << " Y:" << screen_y 
                  << " P:" << static_cast<int>(progress * 100) << "% C:" << (is_valid ? 1 : 0)
                  << " [" << val_str << " dist=" << static_cast<int>(motion_tracker_.estimated_distance_cm) << "cm]" << std::endl;
    }
                
    // Send detailed progress to mobile app via WebSocket periodically
                static int mobile_update_skip = 0;
                if (mobile_update_skip++ % 10 == 0) {
        std::string validation_str;
        switch (motion_tracker_.validation_state) {
            case FaceValidationState::VALID: validation_str = "valid"; break;
            case FaceValidationState::NO_FACE: validation_str = "no_face"; break;
            case FaceValidationState::TOO_CLOSE: validation_str = "too_close"; break;
            case FaceValidationState::TOO_FAR: validation_str = "too_far"; break;
            case FaceValidationState::EXTREME_ROTATION: validation_str = "extreme_rotation"; break;
        }
        
                    nlohmann::json progress_msg = {
                        {"type", "progress"},
                        {"state", "align"},
                        {"progress", static_cast<int>(progress * 100)},
            {"validation", validation_str},
            {"distance_cm", motion_tracker_.estimated_distance_cm},
                        {"nose_position", {
                {"x", nose_x_norm},
                {"y", nose_y_norm},
                {"screen_x", screen_x},
                {"screen_y", screen_y}
                        }}
                    };
                    network_mgr_->send_message(progress_msg.dump());
                }
                
    // Check for completion
                if (progress >= 1.0f) {
        std::cout << "✅ Spiral motion complete!" << std::endl;
        if (ui_test_mode_) {
            // UI test mode: Skip processing, go directly to success
            set_state(SystemState::SUCCESS);
        } else {
                    set_state(SystemState::PROCESSING);
                }
            }
    
    // Check timeout (150 frames = ~5 seconds without valid face)
    if (motion_tracker_.no_face_counter > 150) {
        handle_face_timeout();
    }
}

void SystemController::handle_face_timeout() {
        std::cout << "Face Timeout!" << std::endl;
        nlohmann::json error_msg = {
            {"type", "error"},
            {"code", "timeout_no_face"},
            {"message", "No face detected for 3 seconds"}
        };
        network_mgr_->send_message(error_msg.dump());
        
        ring_buffer_->set_recording_active(false);
        ring_buffer_->clear(); 
        
    // In UI test mode, just reset and stay on Screen 0
    if (ui_test_mode_) {
        std::cout << "🧪 UI TEST: Face timeout - resetting to Screen 0" << std::endl;
        motion_tracker_.reset();
        set_state(SystemState::ALIGN); // Return to Screen 0
        return;
    }
        
    // Send error message before entering ERROR state
    serial_comm_->send_error("Face not detected");
    // ERROR state will handle timer and return to IDLE automatically
        set_state(SystemState::ERROR);
}

// ============================================================================
// Face Validation Functions
// ============================================================================

float SystemController::estimate_face_distance(float face_width_pixels) {
    // Distance estimation using inverse proportion:
    // distance = (reference_width * reference_distance) / current_width
    if (face_width_pixels <= 0) return 0.0f;
    return (REFERENCE_FACE_WIDTH * REFERENCE_DISTANCE) / face_width_pixels;
}

bool SystemController::is_face_orientation_extreme(const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    // Check if face is rotated too far to complete spiral motion
    // Key landmarks: nose tip (4), left eye outer (33), right eye outer (263)
    if (landmarks.size() < 264) return true;  // Not enough landmarks
    
    const auto& nose = landmarks[4];       // Nose tip (MediaPipe index 4)
    const auto& left_eye = landmarks[33];  // Left eye outer corner
    const auto& right_eye = landmarks[263]; // Right eye outer corner
    
    // Calculate eye midpoint
    float eye_mid_x = (left_eye.x + right_eye.x) / 2.0f;
    float eye_width = std::abs(right_eye.x - left_eye.x);
    
    if (eye_width < 1.0f) return true;  // Eyes too close, invalid detection
    
    // Calculate how far nose is from eye midpoint (normalized by eye width)
    float nose_offset = std::abs(nose.x - eye_mid_x) / eye_width;
    
    // If nose is more than threshold toward either eye, it's extreme
    return nose_offset > EXTREME_YAW_THRESHOLD;
}

SystemController::FaceValidationState SystemController::validate_face_position(FrameBox* frame) {
    if (!frame || !frame->metadata.face_detected) {
        return FaceValidationState::NO_FACE;
    }
    
    // Safety check: ensure we have enough landmarks
    if (frame->metadata.landmarks.size() < 468) {
        return FaceValidationState::NO_FACE;
    }
    
    // Estimate distance from face bounding box width
    float face_width = static_cast<float>(frame->metadata.face_w);
    float distance = estimate_face_distance(face_width);
    motion_tracker_.estimated_distance_cm = distance;
    
    // Check distance bounds
    if (distance < MIN_DISTANCE_CM) {
        return FaceValidationState::TOO_CLOSE;
    }
    if (distance > MAX_DISTANCE_CM) {
        return FaceValidationState::TOO_FAR;
    }
    
    // Check orientation
    if (is_face_orientation_extreme(frame->metadata.landmarks)) {
        return FaceValidationState::EXTREME_ROTATION;
    }
    
    return FaceValidationState::VALID;
}

// ============================================================================
// Spiral Motion Progress Algorithm (Cumulative Angular Displacement)
// ============================================================================

float SystemController::calculate_circular_motion_progress(const cv::Point2f& nose) {
    // Add current position to history
    motion_tracker_.nose_positions.push_back(nose);
    
    // Keep only last 60 positions for center calculation
    if (motion_tracker_.nose_positions.size() > 60) {
        motion_tracker_.nose_positions.erase(motion_tracker_.nose_positions.begin());
    }
    
    // Need at least 10 positions to start tracking
    if (motion_tracker_.nose_positions.size() < 10) {
        // Debug: log when we're collecting initial positions
        static int init_counter = 0;
        if (++init_counter % 30 == 0) {
            std::cout << "📊 Collecting initial positions: " 
                      << motion_tracker_.nose_positions.size() << "/10" << std::endl;
        }
        return motion_tracker_.progress;
    }
    
    // Calculate dynamic center (average of all positions)
    cv::Point2f center(0, 0);
    for (const auto& p : motion_tracker_.nose_positions) {
        center += p;
    }
    center.x /= motion_tracker_.nose_positions.size();
    center.y /= motion_tracker_.nose_positions.size();
    
    // Calculate current angle from center
    float dx = nose.x - center.x;
    float dy = nose.y - center.y;
    float distance_from_center = std::sqrt(dx * dx + dy * dy);
    
    // Only track if user moved away from center (not just hovering)
    if (distance_from_center < 0.02f) {
        // User is too close to center, don't update angle
        return motion_tracker_.progress;
    }
    
    float current_angle = std::atan2(dy, dx);
    
    // Initialize first angle
    if (!motion_tracker_.angle_initialized) {
        motion_tracker_.last_angle = current_angle;
        motion_tracker_.angle_initialized = true;
        return motion_tracker_.progress;
    }
    
    // Calculate angular delta (handle wraparound at ±π)
    float delta = current_angle - motion_tracker_.last_angle;
    
    // Normalize delta to [-π, π]
    while (delta > 3.14159f) delta -= 2.0f * 3.14159f;
    while (delta < -3.14159f) delta += 2.0f * 3.14159f;
    
    // Check if user is actually moving (velocity threshold)
    float velocity = std::abs(delta);
    if (velocity < MOTION_PAUSE_THRESHOLD) {
        // User stopped or moving too slowly - pause progress
        return motion_tracker_.progress;
    }
    
    // Accumulate angle (only positive direction for spiral)
    motion_tracker_.cumulative_angle += std::abs(delta);
    motion_tracker_.last_angle = current_angle;
    
    // Calculate progress (0.0 to 1.0)
    motion_tracker_.progress = motion_tracker_.cumulative_angle / SPIRAL_COMPLETE_ANGLE;
    if (motion_tracker_.progress > 1.0f) {
        motion_tracker_.progress = 1.0f;
    }
    
    // Send progress update to WebSocket periodically
    static int update_skip = 0;
    if (update_skip++ % 10 == 0) {
        nlohmann::json progress_msg = {
            {"type", "progress"},
            {"value", motion_tracker_.progress},
            {"angle", motion_tracker_.cumulative_angle},
            {"distance_cm", motion_tracker_.estimated_distance_cm}
        };
        network_mgr_->send_message(progress_msg.dump());
    }
    
    return motion_tracker_.progress;
}

void SystemController::handle_processing() {
    std::cout << "🔬 Starting Batch Processing..." << std::endl;
    
    // Switch camera to low power mode during processing (don't need live feed)
    std::cout << "📹 Switching camera to low power mode for processing..." << std::endl;
    configure_camera_for_state(SystemState::IDLE);
    
    std::vector<FrameBox*> recording = ring_buffer_->get_all_valid_frames();
    
    if (recording.empty()) {
        std::cout << "⚠ No recording data" << std::endl;
        set_state(SystemState::ERROR);
        network_mgr_->send_message("{\"type\":\"error\", \"code\":\"no_recording_data\"}");
        ring_buffer_->clear();
        std::this_thread::sleep_for(std::chrono::seconds(3));
        set_state(SystemState::IDLE);
        return;
    }

    AntiSpoofingConfig as_config;
    auto pipeline = std::make_unique<AntiSpoofingPipeline>(as_config);
    
    std::vector<bool> frame_results;
    int consecutive_passes = 0;
    int max_consecutive = 0;
    FrameBox* best_frame = nullptr;
    
    std::cout << "Processing " << recording.size() << " frames..." << std::endl;
    
    // Process ALL frames (not skipping any)
    for (size_t i = 0; i < recording.size(); i++) {
        FrameBox* frame = recording[i];
        
        // Ensure face detection is done
        if (!frame->metadata.face_detected) {
             face_detector_->detect(frame);
        }
        
        if (frame->metadata.face_detected) {
            bool is_live = pipeline->process_frame(frame);
            frame_results.push_back(is_live);
            
            if (is_live) {
                consecutive_passes++;
                if (consecutive_passes > max_consecutive) {
                    max_consecutive = consecutive_passes;
                    best_frame = frame; // Keep track of best frame
                }
            } else {
                consecutive_passes = 0;
            }
            
            // Send progress update
            int progress = (int)((float)(i + 1) / recording.size() * 100.0f);
            serial_comm_->send_progress(progress);
            network_mgr_->send_message("{\"type\":\"progress\", \"value\":" + std::to_string(progress) + "}");
        }
    }
    
    std::cout << "Max consecutive passes: " << max_consecutive << std::endl;
    
    // SUCCESS: 5 or more consecutive frames passed anti-spoofing
    if (max_consecutive >= 5 && best_frame != nullptr) {
        std::cout << "✅ Anti-spoofing PASSED (" << max_consecutive << " consecutive frames)" << std::endl;
        
        // Extract the best frame as JPEG for server submission
        // Server will convert to base64 and forward to backend API
        cv::Mat rgb_frame = best_frame->get_color_mat();
        
        // Safety check: ensure frame is valid
        if (rgb_frame.empty()) {
            std::cerr << "❌ Best frame is empty - cannot submit" << std::endl;
            serial_comm_->send_error("Camera error");
            set_state(SystemState::ERROR);
            ring_buffer_->clear();
            return;
        }
        std::vector<uchar> jpeg_buffer;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};
        cv::imencode(".jpg", rgb_frame, jpeg_buffer, compression_params);
        
        // Convert JPEG bytes to base64 for WebSocket transmission
        std::vector<uint8_t> buffer_uint8(jpeg_buffer.begin(), jpeg_buffer.end());
        std::string base64_image = CryptoUtils::base64_encode(buffer_uint8);
        
        std::cout << "📸 JPEG image size: " << jpeg_buffer.size() << " bytes" << std::endl;
        std::cout << "📦 Base64 size: " << base64_image.size() << " bytes" << std::endl;
        
        // Send to WebSocket server
        nlohmann::json api_payload;
        api_payload["type"] = "submit_result";
        api_payload["status"] = "success";
        api_payload["platform_id"] = platform_id_;
        api_payload["session_id"] = session_id_;
        api_payload["image"] = base64_image;
        api_payload["consecutive_passes"] = max_consecutive;
        api_payload["timestamp"] = std::chrono::system_clock::now().time_since_epoch().count();
        
        std::string api_message = api_payload.dump();
        std::cout << "📤 Sending result to server (" << api_message.size() << " bytes)..." << std::endl;
        
        // Set flag to wait for server ACK
        result_ack_received_ = false;
        
        // Send message and check for failure
        if (!network_mgr_->send_message(api_message)) {
            std::cerr << "❌ Failed to send result to server" << std::endl;
            serial_comm_->send_error("Failed to send result");
            set_state(SystemState::ERROR);
            ring_buffer_->clear();
            return;
        }
        
        // Wait for server ACK with 3 second timeout
        auto start_time = std::chrono::steady_clock::now();
        const int timeout_ms = 3000;
        
        while (!result_ack_received_) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time
            ).count();
            
            if (elapsed >= timeout_ms) {
                std::cerr << "⚠️  Server ACK timeout after 3 seconds" << std::endl;
                // Assume success even without ACK (data was sent)
                break;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        if (result_ack_received_) {
            std::cout << "✅ Server ACK received" << std::endl;
        }
        
        set_state(SystemState::SUCCESS);
        
    } else {
        std::cout << "❌ Anti-spoofing FAILED (only " << max_consecutive << " consecutive passes)" << std::endl;
        
        // Show error on display
        serial_comm_->send_error("Verification failed");
        serial_comm_->send_state(12);  // Screen 12: Error Animation
        
        // Send failure to WebSocket
        network_mgr_->send_message("{\"type\":\"submit_result\", \"status\":\"failed\", \"message\":\"Liveness check failed\"}");
        
        // Clean up session
        session_id_.clear();
        platform_id_.clear();
        
        // Return to IDLE after 3 seconds
        state_timer_start_ = std::chrono::steady_clock::now();
        state_timer_target_ = SystemState::IDLE;
        state_timer_duration_ms_ = 3000;
        state_timer_active_ = true;
    }
    
    ring_buffer_->clear(); 
}

void SystemController::on_websocket_message(const std::string& message) {
    std::cout << "📨 WS Message: " << message << std::endl;
    
    // Check if message is empty (connection lost)
    if (message.empty()) {
        std::cerr << "❌ WebSocket disconnected (empty message)" << std::endl;
        serial_comm_->send_error("Connection lost");
        set_state(SystemState::ERROR);
        return;
    }
    
    try {
        nlohmann::json msg_json = nlohmann::json::parse(message);
        
        std::string msg_type = msg_json.value("type", "");
        
        // Handle WebSocket connection failures (from NetworkManager retry logic)
        if (msg_type == "connection_failed") {
            int retries = msg_json.value("retries", 0);
            std::cerr << "❌ WebSocket connection failed after " << retries << " retries" << std::endl;
            serial_comm_->send_error("Connection failed");
            serial_comm_->send_state(12);  // Screen 12: Error
            
            // Clean up session
            session_id_.clear();
            platform_id_.clear();
            
            // Return to IDLE after 3 seconds
            state_timer_start_ = std::chrono::steady_clock::now();
            state_timer_target_ = SystemState::IDLE;
            state_timer_duration_ms_ = 3000;
            state_timer_active_ = true;
            return;
        }
        
        // Handle WebSocket disconnection during session
        if (msg_type == "disconnected") {
            std::cerr << "⚠️  WebSocket disconnected unexpectedly" << std::endl;
            
            // Only show error if we were in an active state
            if (current_state_ == SystemState::READY ||
                current_state_ == SystemState::COUNTDOWN ||
                current_state_ == SystemState::ALIGN ||
                current_state_ == SystemState::PROCESSING) {
                
                serial_comm_->send_error("Connection lost");
                serial_comm_->send_state(12);  // Screen 12: Error
                
                // Clean up
                session_id_.clear();
                platform_id_.clear();
                motion_tracker_.reset();
                ring_buffer_->set_recording_active(false);
                ring_buffer_->clear();
                
                // Return to IDLE after 3 seconds
                state_timer_start_ = std::chrono::steady_clock::now();
                state_timer_target_ = SystemState::IDLE;
                state_timer_duration_ms_ = 3000;
                state_timer_active_ = true;
            }
            return;
        }
        
        if (msg_type == "auth_success") {
            // WebSocket authentication successful
            std::cout << "✅ WebSocket authenticated successfully" << std::endl;
            
            // Server already notifies mobile with "device_connected"
            // No need to send redundant "device_ready" message
            
            // Go directly to READY state - no need to wait, WebSocket is connected and authenticated
            std::cout << "📍 WebSocket authenticated → READY state" << std::endl;
            set_state(SystemState::READY);
        }
        else if (msg_type == "auth_failed") {
            // Authentication failed - clear token and stop reconnecting
            std::string error_msg = msg_json.value("error", "Authentication failed");
            std::cerr << "❌ WebSocket auth failed: " << error_msg << std::endl;
            
            // Clear the pending token to stop reconnect loop
            {
                std::lock_guard<std::mutex> lock(ws_auth_mutex_);
                pending_ws_token_.clear();
            }
            
            // Disconnect and go to error state
            network_mgr_->disconnect();
            serial_comm_->send_error("Auth failed");
            serial_comm_->send_state(12); // Screen 12: Error Animation
            
            // Return to IDLE after 3 seconds
            state_timer_start_ = std::chrono::steady_clock::now();
            state_timer_target_ = SystemState::IDLE;
            state_timer_duration_ms_ = 3000;
            state_timer_active_ = true;
        }
        else if (msg_type == "mobile_disconnected") {
            // Mobile app disconnected - show error and go back to IDLE
            // NOTE: Do NOT call network_mgr_->disconnect() here - we're inside the WS callback!
            // The ERROR state handler will clean up the connection
            std::cout << "⚠️  Mobile app disconnected during session" << std::endl;
            serial_comm_->send_error("Mobile disconnected");
            // Set state to ERROR - the state handler will disconnect WebSocket asynchronously
            set_state(SystemState::ERROR);
        }
        else if (msg_type == "to_device") {
            // Server wrapped message - extract the data payload
            if (msg_json.contains("data")) {
                nlohmann::json data = msg_json["data"];
                
                if (data.contains("command")) {
                    std::string command = data.value("command", "");
                    std::cout << "📩 Received command from mobile: " << command << std::endl;
                    
                    if (command == "start_verification" || command == "start") {
                        std::cout << "▶️  Processing start verification command" << std::endl;
                        
                        if (current_state_ == SystemState::READY) {
                            if (test_mode_) {
                                // 🧪 TEST MODE: Skip verification, show success immediately
                                std::cout << "🧪 TEST MODE: Bypassing verification flow → SUCCESS" << std::endl;
                                serial_comm_->send_state(11); // Screen 11: Success Animation
                                
                                // Send success message to mobile
                                nlohmann::json success_msg = {
                                    {"type", "to_mobile"},
                                    {"data", {
                                        {"type", "result"},
                                        {"status", "success"},
                                        {"message", "TEST MODE: Verification bypassed"}
                                    }}
                                };
                                network_mgr_->send_message(success_msg.dump());
                                
                                set_state(SystemState::SUCCESS);
                            } else {
                                // REAL MODE: Start countdown, then verification
                                std::cout << "🔢 Starting COUNTDOWN phase" << std::endl;
                                set_state(SystemState::COUNTDOWN);
                            }
                        } else {
                            std::cerr << "⚠️  Received start command but not in READY state (current: " 
                                      << static_cast<int>(current_state_.load()) << ")" << std::endl;
                        }
                    }
                }
            }
        }
        // Handle start command (legacy: server forwards as plain JSON without "type" wrapper)
        else if (msg_json.contains("command")) {
            std::string command = msg_json.value("command", "");
            
            if (command == "start_verification" || command == "start") {
                std::cout << "▶️  Received start command from mobile" << std::endl;
                
                if (current_state_ == SystemState::READY) {
                    if (test_mode_) {
                        // 🧪 TEST MODE: Skip verification, show success immediately
                        std::cout << "🧪 TEST MODE: Bypassing verification flow → SUCCESS" << std::endl;
                        serial_comm_->send_state(11); // Screen 11: Success Animation
                        
                        // Send success message to mobile
                        nlohmann::json success_msg = {
                            {"type", "to_mobile"},
                            {"data", {
                                {"type", "result"},
                                {"status", "success"},
                                {"message", "TEST MODE: Verification bypassed"}
                            }}
                        };
                        network_mgr_->send_message(success_msg.dump());
                        
                        set_state(SystemState::SUCCESS);
                    } else {
                        // REAL MODE: Start countdown, then verification
                        std::cout << "🔢 Starting COUNTDOWN phase" << std::endl;
                        set_state(SystemState::COUNTDOWN);
                    }
                } else {
                    std::cerr << "⚠️  Received start command but not in READY state (current: " 
                              << static_cast<int>(current_state_.load()) << ")" << std::endl;
                }
            }
            else if (command == "retry_camera") {
                std::cout << "🔄 Received camera retry command" << std::endl;
                
                // Only allow retry if in ERROR state or camera is not running
                if (current_state_ == SystemState::ERROR || !producer_ || !producer_->is_running()) {
                    std::cout << "📹 Attempting to reinitialize camera..." << std::endl;
                    serial_comm_->send_state(2); // Show "Connecting..." screen
                    
                    // Clean up existing producer
                    if (producer_) {
                        producer_.reset();
                        std::this_thread::sleep_for(std::chrono::seconds(2));
                    }
                    
                    // Try to restart camera
                    configure_camera_for_state(SystemState::BOOT);
                    
                    if (producer_ && producer_->is_running()) {
                        std::cout << "✅ Camera reinitialized successfully!" << std::endl;
                        // Transition to IDLE (which will automatically show Screen 5)
                        set_state(SystemState::IDLE);
                    } else {
                        std::cerr << "❌ Camera retry failed" << std::endl;
                        serial_comm_->send_error("Camera retry failed - check hardware");
                        serial_comm_->send_state(12); // Screen 12: Error Animation
                        set_state(SystemState::ERROR);
                    }
                } else {
                    std::cout << "⚠️  Retry command ignored - system not in error state" << std::endl;
                }
            }
            else if (command == "restart_system") {
                std::cout << "🔄 Received system restart command" << std::endl;
                std::cout << "⚠️  Initiating controlled shutdown..." << std::endl;
                serial_comm_->send_state(2); // Show connecting screen
                running_ = false; // This will cause main loop to exit and systemd to restart
            }
        }
        else if (msg_type == "verification_result" || msg_type == "api_response" || msg_type == "image_received") {
            // Server ACK for result submission
            result_ack_received_ = true;
            
            std::string status = msg_json.value("status", "ok");
            
            if (status == "success" || status == "ok") {
                std::cout << "✅ Server acknowledged result" << std::endl;
                // Don't change state here - handle_processing will do it
            } else if (status == "failed" || status == "error") {
                std::cout << "❌ Verification FAILED" << std::endl;
                std::string reason = msg_json.value("reason", msg_json.value("message", "unknown"));
                std::cerr << "Reason: " << reason << std::endl;
                serial_comm_->send_error(reason);
                set_state(SystemState::ERROR);
            }
        }
        else if (msg_type == "mobile_disconnected") {
            // Mobile app disconnected
            std::cout << "📱 Mobile disconnected, returning to IDLE" << std::endl;
            set_state(SystemState::IDLE);
        }
        else if (msg_type == "error") {
            std::string error_msg = msg_json.value("message", "Unknown error");
            std::cerr << "⚠ Server Error: " << error_msg << std::endl;
            serial_comm_->send_error(error_msg);
        }
        else {
            std::cout << "ℹ️ Unknown message type: " << msg_type << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "⚠ Failed to parse WS message: " << e.what() << std::endl;
    }
}

// ═══════════════════════════════════════════════════════════════════
// NEW METHODS: Device Config, Boot Check, QR Handlers
// ═══════════════════════════════════════════════════════════════════

bool SystemController::load_device_config() {
    // Initialize TrustZone first
    if (!TrustZoneIdentity::initialize()) {
        std::cerr << "⚠ TrustZone initialization failed" << std::endl;
        return false;
    }
    
    // Get device ID from TrustZone (hardware-derived)
    device_id_ = TrustZoneIdentity::get_device_id();
    hardware_id_ = TrustZoneIdentity::get_hardware_serial();
    
    std::cout << "✅ Device Identity Loaded:" << std::endl;
    std::cout << "   Device ID: " << device_id_ << std::endl;
    std::cout << "   Hardware ID: " << hardware_id_ << std::endl;
    
    // Load or create device config file
    std::string config_path = "/etc/mdai/device_config.json";
    std::ifstream config_file(config_path);
    
    if (!config_file.is_open()) {
        // Create new config
        std::cout << "📝 Creating device config..." << std::endl;
        
        nlohmann::json config;
        config["device_id"] = device_id_;
        config["hardware_id"] = hardware_id_;
        config["registered_at"] = std::chrono::system_clock::now().time_since_epoch().count();
        
        std::ofstream out_file(config_path);
        out_file << config.dump(2);
        out_file.close();
        
        std::cout << "✅ Device config created: " << config_path << std::endl;
        return true;
    }
    
    // Config exists, verify it matches
    try {
        nlohmann::json config;
        config_file >> config;
        
        std::string stored_device_id = config["device_id"].get<std::string>();
        if (stored_device_id != device_id_) {
            std::cerr << "⚠ Device ID mismatch!" << std::endl;
            std::cerr << "   Stored: " << stored_device_id << std::endl;
            std::cerr << "   Current: " << device_id_ << std::endl;
            return false;
        }
        
        std::cout << "✅ Device Config Verified" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "⚠ Failed to parse device config: " << e.what() << std::endl;
        return false;
    }
}

bool SystemController::check_wifi_on_boot() {
    // Check if WiFi is already connected
    if (network_mgr_->is_connected_to_internet()) {
        std::string ip = network_mgr_->get_ip_address();
        std::cout << "✅ WiFi already connected: " << ip << std::endl;
        return true;
    }
    return false;
}

void SystemController::handle_boot() {
    // Screen 1: Logo displayed for 3 seconds (preemptive kernel-level control loop)
    serial_comm_->send_state(1);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Screen 2: "Booting into mDai" - Starting services (NOT preemptive)
    serial_comm_->send_state(2);
    
    // Service 1: Initialize camera in LOW POWER mode (persistent, never closed)
    // Retry logic: attempt up to 3 times with increasing delays
    std::cout << "🔧 Service: Initializing camera..." << std::endl;
    
    const int MAX_CAMERA_RETRIES = 3;
    bool camera_initialized = false;
    
    for (int attempt = 1; attempt <= MAX_CAMERA_RETRIES; attempt++) {
        std::cout << "📹 Camera initialization attempt " << attempt << "/" << MAX_CAMERA_RETRIES << std::endl;
        
        // Clean up previous attempt if needed
        if (producer_ && !producer_->is_running()) {
            std::cout << "🔄 Cleaning up previous camera instance..." << std::endl;
            producer_.reset();
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        
        configure_camera_for_state(SystemState::BOOT);
        
        if (producer_ && producer_->is_running()) {
            camera_initialized = true;
            std::cout << "✅ Service: Camera started successfully on attempt " << attempt << std::endl;
            break;
        }
        
        std::cerr << "❌ Camera initialization failed (attempt " << attempt << "/" << MAX_CAMERA_RETRIES << ")" << std::endl;
        
        if (attempt < MAX_CAMERA_RETRIES) {
            int wait_time = attempt * 3; // 3s, 6s, 9s
            std::cout << "⏳ Waiting " << wait_time << " seconds before retry..." << std::endl;
            serial_comm_->send_error("Camera init failed - retrying...");
            std::this_thread::sleep_for(std::chrono::seconds(wait_time));
        }
    }
    
    if (!camera_initialized) {
        std::cerr << "❌ FATAL: Camera initialization failed after " << MAX_CAMERA_RETRIES << " attempts" << std::endl;
        serial_comm_->send_error("Camera hardware error - please restart device");
        serial_comm_->send_state(12); // Screen 12: Error Animation
        
        // Enter error state - don't exit, allow system to stay alive for diagnostics
        // Mobile/Admin can send reset command or manually restart device
        std::cout << "⚠️  System entering ERROR state - camera unavailable" << std::endl;
        std::cout << "💡 Tip: Check USB connections and restart device" << std::endl;
        
        // Stay in error state indefinitely - don't kill the process
        // This allows:
        // 1. Serial communication to remain active (display shows error)
        // 2. Network stays up (can receive reset commands)
        // 3. Logs continue to be written
        // 4. Admin can SSH in to diagnose
        
        set_state(SystemState::ERROR);
        return; // Exit boot sequence but keep system running
    }
    std::cout << "✅ Camera initialization complete" << std::endl;
    
    // Service 2: Verify serial communication
    std::cout << "🔧 Service: Checking display..." << std::endl;
    if (!serial_comm_->is_connected()) {
        std::cerr << "⚠️  WARNING: Display not connected (non-fatal)" << std::endl;
        // Continue anyway - serial is optional for operation
    } else {
        std::cout << "✅ Service: Display connected" << std::endl;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Check WiFi connection status
    if (check_wifi_on_boot()) {
        // WiFi already connected - skip Screen 3 (WiFi QR scanning)
        // Go to PROVISIONED state which will show Screen 4 → Screen 5 → Screen 6
        std::string ssid = network_mgr_->get_current_ssid();
        if (ssid.empty()) ssid = "WiFi Connected";
        // Set PROVISIONED state - it will handle Screen 4 → Screen 5 → Screen 6 sequence
        set_state(SystemState::PROVISIONED);
    } else {
        // No WiFi - show Screen 3: "Looking for WiFi QR"
        // This screen only displays if WiFi is NOT connected (as per plan)
        std::cout << "📡 No WiFi connection - entering provisioning mode" << std::endl;
        set_state(SystemState::AWAIT_ADMIN_QR);
    }
}

bool SystemController::handle_wifi_qr(const std::string& qr_data) {
    try {
        // Parse QR and decrypt challenge using TrustZone
        auto [success, bson_id, decrypted_challenge, error_msg] = 
            CryptoUtils::validate_and_decrypt_qr(qr_data, TrustZoneIdentity::export_private_key_for_backup());
        
        if (!success) {
            std::cerr << "⚠ QR validation failed: " << error_msg << std::endl;
            return false;
        }
        
        std::cout << "✅ QR Challenge Decrypted: " << bson_id << std::endl;
        
        // Validate challenge with server
        std::string server_url = "https://" + middleware_host_;
        auto [server_success, wifi_ssid, wifi_password, session_type] = 
            CryptoUtils::validate_challenge_with_server(
                server_url, bson_id, decrypted_challenge);
        
        if (!server_success) {
            std::cerr << "⚠ Challenge validation failed with server" << std::endl;
            return false;
        }
        
        std::cout << "✅ WiFi Credentials Received: " << wifi_ssid << std::endl;
        
        // Try to connect
        set_state(SystemState::WIFI_CHANGE_CONNECTING);
        serial_comm_->send_state(14); // Screen 14: "Connecting to New WiFi..."
        
        if (network_mgr_->connect_wifi(wifi_ssid, wifi_password)) {
            previous_ssid_ = wifi_ssid;
            previous_password_ = wifi_password;
            
            set_state(SystemState::WIFI_CHANGE_SUCCESS);
            serial_comm_->send_state(4); // Screen 4: "WiFi Success" (show SSID)
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            // Transition to IDLE (which will automatically show Screen 5)
            set_state(SystemState::IDLE);
            return true;
        } else {
            // Failed, fallback
            set_state(SystemState::WIFI_CHANGE_FAILED);
            serial_comm_->send_state(13); // Screen 13: "WiFi Connection Failed"
            serial_comm_->send_error("WiFi Failed - Reverting to known network");
            
            // Try fallback to previous network
            if (!previous_ssid_.empty()) {
                std::cout << "Falling back to: " << previous_ssid_ << std::endl;
                network_mgr_->connect_wifi(previous_ssid_, previous_password_);
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(5));
            set_state(SystemState::AWAIT_ADMIN_QR);
            return false;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "⚠ WiFi QR processing error: " << e.what() << std::endl;
        serial_comm_->send_state(12); // Error screen
        return false;
    }
}

// ============================================================================
// HARDWARE HEALTH MONITORING & REPAIR MECHANISMS
// ============================================================================

/**
 * Check camera health
 * Returns false if camera is not responding or disconnected
 */
bool SystemController::check_camera_health() {
    if (!producer_) {
        std::cerr << "❌ Camera health check: Producer not initialized" << std::endl;
        return false;
    }
    
    // Check if producer is running
    if (!producer_->is_running()) {
        std::cerr << "❌ Camera health check: Producer not running" << std::endl;
        return false;
    }
    
    // Check if we're receiving frames (check ring buffer activity)
    FrameBox* test_frame = ring_buffer_->get_latest_frame();
    if (!test_frame) {
        std::cerr << "⚠️  Camera health check: No frames in buffer (may be starting up)" << std::endl;
        return false;  // Not necessarily a failure, could be initializing
    }
    ring_buffer_->release_frame(test_frame);
    
    std::cout << "✅ Camera health check: OK" << std::endl;
    return true;
}

/**
 * Check display (ESP32 serial) health
 * Returns false if serial connection is lost
 */
bool SystemController::check_display_health() {
    if (!serial_comm_) {
        std::cerr << "❌ Display health check: Serial not initialized" << std::endl;
        return false;
    }
    
    // Check if serial port is still open
    if (!serial_comm_->is_connected()) {
        std::cerr << "❌ Display health check: Serial connection lost" << std::endl;
        return false;
    }
    
    std::cout << "✅ Display health check: OK" << std::endl;
    return true;
}

/**
 * Check network health
 * Returns false if WiFi is disconnected
 */
bool SystemController::check_network_health() {
    if (!network_mgr_) {
        std::cerr << "❌ Network health check: NetworkManager not initialized" << std::endl;
        return false;
    }
    
    // Check WiFi connection
    if (!network_mgr_->is_connected_to_internet()) {
        std::cerr << "⚠️  Network health check: No internet connection" << std::endl;
        return false;
    }
    
    std::string ip = network_mgr_->get_ip_address();
    if (ip == "0.0.0.0") {
        std::cerr << "⚠️  Network health check: No valid IP address" << std::endl;
        return false;
    }
    
    std::cout << "✅ Network health check: OK (IP: " << ip << ")" << std::endl;
    return true;
}

/**
 * Handle hardware errors with retry mechanism
 * @param component - Component name (Camera, Display, Network)
 * @param error_msg - Error message to display
 * 
 * IMPORTANT: Hardware errors stay on screen indefinitely until repaired or manual intervention
 */
void SystemController::handle_hardware_error(const std::string& component, const std::string& error_msg) {
    std::cerr << "🔧 Hardware Error Handler: " << component << " - " << error_msg << std::endl;
    
    const int MAX_RETRIES = 3;
    int retry_count = 0;
    bool recovered = false;
    
    // Show CRITICAL hardware error on display (stays until fixed)
    if (serial_comm_ && serial_comm_->is_connected()) {
        serial_comm_->send_error("HARDWARE: " + component + " - " + error_msg);
        serial_comm_->send_state(16); // Screen 16: Critical hardware error (stays indefinitely)
    }
    
    std::cout << "⚠️  Hardware error detected - will attempt recovery but screen stays until fixed" << std::endl;
    
    // Retry mechanism based on component
    if (component == "Camera") {
        std::cout << "🔄 Attempting camera recovery..." << std::endl;
        
        // Stop current producer first
        if (producer_) {
            std::cout << "📹 Stopping current camera producer..." << std::endl;
            producer_->stop();
            producer_.reset();
        }
        
        // Wait for USB device to fully release (critical for RealSense)
        std::cout << "⏳ Waiting for USB device release (3s)..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        for (retry_count = 1; retry_count <= MAX_RETRIES; retry_count++) {
            // Exponential backoff: 3s, 5s, 8s
            int wait_seconds = (retry_count == 1) ? 3 : (retry_count == 2) ? 5 : 8;
            
            std::cout << "🔄 Camera retry attempt " << retry_count << "/" << MAX_RETRIES 
                      << " (waiting " << wait_seconds << "s for USB re-enumeration)" << std::endl;
            
            // Update display with retry status
            if (serial_comm_ && serial_comm_->is_connected()) {
                serial_comm_->send_error("Camera retry " + std::to_string(retry_count) + "/" + std::to_string(MAX_RETRIES));
            }
            
            // Wait for USB re-enumeration (RealSense needs time after disconnect)
            std::this_thread::sleep_for(std::chrono::seconds(wait_seconds));
            
            // First check if any RealSense device is available
            try {
                rs2::context ctx;
                auto devices = ctx.query_devices();
                bool device_found = false;
                std::string device_name;
                
                for (size_t i = 0; i < devices.size(); i++) {
                    auto dev = devices[i];
                    std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
                    if (name.find("RealSense") != std::string::npos) {
                        device_found = true;
                        device_name = name;
                        std::cout << "✅ Found RealSense device: " << name << std::endl;
                        break;
                    }
                }
                
                if (!device_found) {
                    std::cerr << "⚠️  No RealSense device found, waiting for reconnection..." << std::endl;
                    if (serial_comm_ && serial_comm_->is_connected()) {
                        serial_comm_->send_error("Waiting for camera...");
                    }
                    continue;  // Try again
                }
                
                // Device found - try to initialize
                std::cout << "📹 Attempting to initialize " << device_name << "..." << std::endl;
                
                CameraConfig config;
                config.enable_ir = false;
                config.enable_depth = false;
                config.color_width = 848;    // Same as FULL mode for consistency
                config.color_height = 480;
                config.color_fps = 30;
                
                producer_ = std::make_unique<Producer>(config, ring_buffer_.get());
                
                // Set up error callback for future monitoring
                producer_->set_error_callback([this](const std::string& error) {
                    std::cerr << "📹 Camera Error: " << error << std::endl;
                    camera_error_detected_.store(true);
                    camera_error_message_ = error;
                });
                
                producer_->set_status_callback([this](const std::string& status) {
                    std::cout << "📹 Camera Status: " << status << std::endl;
                });
                
                if (producer_->start()) {
                    // Wait briefly for first frame to confirm camera is working
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    
                    if (producer_->is_running()) {
                        std::cout << "✅ Camera recovered after " << retry_count << " attempt(s)" << std::endl;
                        recovered = true;
                        current_camera_mode_.store(CameraMode::RGB_ONLY);
                        break;
                    } else {
                        std::cerr << "⚠️  Camera started but stopped immediately" << std::endl;
                        producer_.reset();
                    }
                } else {
                    std::cerr << "❌ Camera start() returned false" << std::endl;
                    producer_.reset();
                }
            } catch (const rs2::error& e) {
                std::cerr << "❌ RealSense error during recovery attempt " << retry_count << ": " << e.what() << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "❌ Camera recovery attempt " << retry_count << " failed: " << e.what() << std::endl;
            }
        }
        
    } else if (component == "Display") {
        std::cout << "🔄 Attempting display recovery..." << std::endl;
        
        for (retry_count = 1; retry_count <= MAX_RETRIES; retry_count++) {
            std::cout << "🔄 Display retry attempt " << retry_count << "/" << MAX_RETRIES << std::endl;
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // Try to reconnect serial
            if (serial_comm_ && serial_comm_->connect()) {
                std::cout << "✅ Display recovered after " << retry_count << " attempt(s)" << std::endl;
                
                // Update screen to show recovery
                serial_comm_->send_error("Display recovered!");
                
                recovered = true;
                break;
            }
        }
        
    } else if (component == "Network") {
        std::cout << "🔄 Network auto-recovery handled by WiFi monitor" << std::endl;
        
        // Network recovery is handled by NetworkManager's wifi_monitor_loop
        // Wait and check periodically
        for (retry_count = 1; retry_count <= MAX_RETRIES; retry_count++) {
            std::cout << "🔄 Network check attempt " << retry_count << "/" << MAX_RETRIES << std::endl;
            
            if (serial_comm_ && serial_comm_->is_connected()) {
                serial_comm_->send_error("Network check " + std::to_string(retry_count) + "/" + std::to_string(MAX_RETRIES));
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(5));
            
            if (network_mgr_->is_connected_to_internet()) {
                std::cout << "✅ Network recovered!" << std::endl;
                recovered = true;
                break;
            }
        }
    }
    
    // Handle recovery result
    if (recovered) {
        std::cout << "✅ " << component << " recovered successfully!" << std::endl;
        std::cout << "🔄 Returning to IDLE state in 2 seconds..." << std::endl;
        
        // Show success message briefly
        if (serial_comm_ && serial_comm_->is_connected()) {
            serial_comm_->send_error(component + " RECOVERED!");
            std::this_thread::sleep_for(std::chrono::seconds(2));
            serial_comm_->send_state(5); // Return to IDLE screen
        }
        
        // Return to IDLE state
        set_state(SystemState::IDLE);
        
    } else {
        // CRITICAL ERROR - Could not recover - STAYS ON ERROR SCREEN INDEFINITELY
        std::cerr << "❌ CRITICAL: " << component << " recovery FAILED after " << MAX_RETRIES << " attempts" << std::endl;
        std::cerr << "⚠️  System requires MANUAL INTERVENTION" << std::endl;
        std::cerr << "⚠️  Error screen will remain until hardware is fixed" << std::endl;
        
        if (serial_comm_ && serial_comm_->is_connected()) {
            serial_comm_->send_error("CRITICAL: " + component + " FAILED - Manual fix needed");
            serial_comm_->send_state(16); // Screen 16: Critical hardware error - STAYS INDEFINITELY
        }
        
        // DO NOT return to IDLE - stay in error state
        // System remains running for SSH access and manual recovery
        std::cout << "⚠️  System halted in error state - awaiting manual intervention" << std::endl;
        std::cout << "⚠️  SSH access remains available for debugging" << std::endl;
        
        // Keep system alive but in error state
        // Admin can SSH in, fix hardware, and manually restart the service
    }
}

} // namespace mdai
