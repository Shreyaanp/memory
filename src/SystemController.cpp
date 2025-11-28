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
    // Ring buffer: 32 initial slots, max 2GB memory (enough for ~1700 frames)
    ring_buffer_ = std::make_unique<DynamicRingBuffer>(32, 2ULL * 1024 * 1024 * 1024);
    
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

        // WiFi connectivity check (every ~1 second)
        static int wifi_check_counter = 0;
        static int wifi_fail_count = 0;
        static bool wifi_was_connected = true;  // Assume connected initially
        
        if (++wifi_check_counter >= 200) {  // Check every ~1s (200 * 5ms)
            wifi_check_counter = 0;
            
            bool wifi_connected = network_mgr_->is_connected_to_internet();
            
            if (wifi_connected) {
                if (wifi_fail_count > 0) {
                    std::cout << "✅ WiFi connected" << std::endl;
                }
                wifi_fail_count = 0;
                wifi_was_connected = true;
                
                // If we were waiting for WiFi, transition to PROVISIONED
                if (current_state_ == SystemState::AWAIT_ADMIN_QR) {
                    std::string ip = network_mgr_->get_ip_address();
                    std::cout << "✅ WiFi connected (" << ip << ") → PROVISIONED" << std::endl;
                    set_state(SystemState::PROVISIONED);
                }
            } else {
                wifi_fail_count++;
                
                // After 3 consecutive failures (~3s), show "Looking for WiFi"
                if (wifi_fail_count >= 3 && wifi_was_connected) {
                    wifi_was_connected = false;
                    std::cout << "📡 WiFi disconnected - showing Screen 3" << std::endl;
                    
                    // Clean up any active session
                    if (current_state_ != SystemState::AWAIT_ADMIN_QR && 
                        current_state_ != SystemState::BOOT) {
                        session_id_.clear();
                        platform_id_.clear();
                        motion_tracker_.reset();
                        ring_buffer_->set_recording_active(false);
                        ring_buffer_->clear();
                        if (network_mgr_->is_connected()) {
                            network_mgr_->disconnect();
                        }
                    }
                    
                    serial_comm_->send_state(3);  // Screen 3: Looking for WiFi
                    set_state(SystemState::AWAIT_ADMIN_QR);
                }
            }
        }

        // Check state timer (for automatic transitions like ERROR -> IDLE, LOGOUT -> IDLE)
        if (state_timer_active_.load()) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - state_timer_start_
            ).count();
            
            // Debug: log timer status periodically
            static int timer_debug = 0;
            if (++timer_debug % 200 == 0) {
                std::cout << "⏱️ Timer check: elapsed=" << elapsed << "ms target=" << state_timer_duration_ms_ << "ms" << std::endl;
            }
            
            if (elapsed >= state_timer_duration_ms_) {
                state_timer_active_.store(false);
                std::cout << "🔄 Timer fired! Transitioning to IDLE" << std::endl;
                std::cout.flush();
                set_state(state_timer_target_);
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

        // =====================================================================
        // ESP32 DISPLAY HEALTH MONITORING
        // Check heartbeats from ESP32, detect crashes/disconnects
        // =====================================================================
        static int esp32_check_counter = 0;
        static int esp32_fail_count = 0;
        static bool esp32_was_alive = true;
        
        if (++esp32_check_counter >= 100) {  // Check every ~500ms (100 * 5ms)
            esp32_check_counter = 0;
            
            // Read any pending messages from ESP32 (heartbeats, status, ACKs)
            if (serial_comm_ && serial_comm_->is_connected()) {
                serial_comm_->check_esp32_status();
                
                // Check if ESP32 is alive (received heartbeat within 5 seconds)
                bool esp32_alive = serial_comm_->is_esp32_alive(5000);
                
                if (esp32_alive) {
                    if (!esp32_was_alive) {
                        std::cout << "✅ ESP32 display reconnected" << std::endl;
                    }
                    esp32_fail_count = 0;
                    esp32_was_alive = true;
                    
                    // Verify screen synchronization (only during stable states)
                    if (current_state_ == SystemState::IDLE || 
                        current_state_ == SystemState::READY ||
                        current_state_ == SystemState::AWAIT_ADMIN_QR) {
                        
                        int expected_screen = serial_comm_->get_last_state();
                        int actual_screen = serial_comm_->get_esp32_screen();
                        
                        // If there's a mismatch and we've sent a state, resync
                        if (expected_screen >= 0 && actual_screen >= 0 && 
                            expected_screen != actual_screen) {
                            std::cout << "⚠️ ESP32 screen mismatch: expected " << expected_screen 
                                      << ", got " << actual_screen << " - resyncing" << std::endl;
                            serial_comm_->send_state(expected_screen);
                        }
                    }
                    
                    // Log ESP32 memory status periodically (every ~30 seconds)
                    static int esp32_mem_log_counter = 0;
                    if (++esp32_mem_log_counter >= 60) {  // 60 * 500ms = 30s
                        esp32_mem_log_counter = 0;
                        uint32_t esp32_heap = serial_comm_->get_esp32_heap();
                        if (esp32_heap > 0) {
                            std::cout << "📊 ESP32 Heap: " << esp32_heap << " bytes free" << std::endl;
                            if (esp32_heap < 30000) {
                                std::cerr << "⚠️ ESP32 LOW MEMORY WARNING!" << std::endl;
                            }
                        }
                    }
                } else {
                    // ESP32 not responding
                    esp32_fail_count++;
                    
                    if (esp32_was_alive && esp32_fail_count >= 2) {
                        esp32_was_alive = false;
                        std::cerr << "⚠️ ESP32 display not responding (no heartbeat for 5+ seconds)" << std::endl;
                        
                        // Try to reconnect serial
                        std::cout << "🔄 Attempting ESP32 reconnection..." << std::endl;
                        if (serial_comm_->try_reconnect()) {
                            std::cout << "✅ ESP32 serial reconnected, resending state..." << std::endl;
                            // Resend current state
                            int last_state = serial_comm_->get_last_state();
                            if (last_state >= 0) {
                                serial_comm_->send_state(last_state);
                            }
                        } else {
                            std::cerr << "❌ ESP32 reconnection failed" << std::endl;
                        }
                    }
                    
                    // After 10 consecutive failures (~5 seconds), log critical error
                    if (esp32_fail_count >= 10 && esp32_fail_count % 10 == 0) {
                        std::cerr << "❌ CRITICAL: ESP32 display unresponsive for " 
                                  << (esp32_fail_count / 2) << " seconds!" << std::endl;
                    }
                }
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
            serial_comm_->send_state(2); // Screen 2: "Connecting to WiFi"
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
            // Camera remains in RGB-only mode configured in IDLE.
            // We don't reinitialize the RealSense pipeline here to avoid frame drop issues.
            // Recording stays OFF until ALIGN state (Screen 9).
            
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
            std::cout << "🔢 COUNTDOWN: Starting camera switch + countdown" << std::endl;
            serial_comm_->send_state(8);
            serial_comm_->send_state_with_text(8, "...");  // Show "preparing"
            
            // Run camera switch + countdown in thread
            std::thread([this]() {
                // STEP 1: Switch to FULL mode (Color + Depth + IR)
                std::cout << "📹 Switching to FULL mode (Color + Depth + IR)..." << std::endl;
                configure_camera_for_state(SystemState::ALIGN);
                
                // Verify camera is ready
                if (!producer_ || !producer_->is_running()) {
                    std::cerr << "❌ Camera failed to start in FULL mode" << std::endl;
                    serial_comm_->send_error("Camera error");
                    set_state(SystemState::ERROR);
                    return;
                }
                std::cout << "✅ Camera ready in FULL mode" << std::endl;
                
                // STEP 2: Countdown 5,4,3,2,1
                for (int count = 5; count >= 1; count--) {
                    if (current_state_ != SystemState::COUNTDOWN) {
                        std::cout << "⚠️ Countdown interrupted" << std::endl;
                        return;
                    }
                    serial_comm_->send_state_with_text(8, std::to_string(count));
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                
                // STEP 3: Transition to ALIGN
                if (current_state_ == SystemState::COUNTDOWN) {
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
            std::cout << "📍 ALIGN: Recording enabled (Color + Depth + IR)" << std::endl;
            serial_comm_->send_state(9); // Screen 9: Nose tracking
            serial_comm_->send_tracking_data(233, 233, 0, true);
            
            // ROBUST RECORDING FIX: Stop → Clear → Start
            // Prevents race condition where frames written during clear
            ring_buffer_->set_recording_active(false);
            ring_buffer_->clear();
            motion_tracker_.reset();
            ring_buffer_->set_recording_active(true);
            std::cout << "📼 Recording active - frames being stored" << std::endl;
            break;
            
        case SystemState::PROCESSING:
            std::cout << "⚙️ PROCESSING: Recording stopped, sending to server" << std::endl;
            ring_buffer_->set_recording_active(false);
            serial_comm_->send_state(10); // Screen 10: "Processing" screen
            std::thread([this]() { handle_processing(); }).detach();
            break;
            
        case SystemState::SUCCESS:
            std::cout << "✅ SUCCESS: Switching back to RGB_ONLY mode" << std::endl;
            serial_comm_->send_state(11); // Screen 11: "Success Animation"
            
            // Switch back to low-power RGB-only mode
            configure_camera_for_state(SystemState::IDLE);
            
            // Wait for "delete" message from mobile via WebSocket
            break;
            
        case SystemState::ERROR:
            serial_comm_->send_state(12); // Screen 12: "Error Animation"
            std::cout << "❌ ERROR state - cleaning up..." << std::endl;
            
            // Clean up session
            session_id_.clear();
            platform_id_.clear();
            motion_tracker_.reset();
            {
                std::lock_guard<std::mutex> lock(ws_auth_mutex_);
                pending_ws_token_.clear();
            }
            
            // Switch camera back to RGB_ONLY
            configure_camera_for_state(SystemState::IDLE);
            
            // Disconnect WebSocket in a separate thread (avoids deadlock if called from WS callback)
            if (network_mgr_ && network_mgr_->is_connected()) {
                std::thread([this]() {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    if (network_mgr_) {
                        network_mgr_->disconnect();
                        std::cout << "✅ WebSocket disconnected" << std::endl;
                    }
                }).detach();
            }
            
            // Start timer to return to IDLE after 3 seconds
            state_timer_start_ = std::chrono::steady_clock::now();
            state_timer_target_ = SystemState::IDLE;
            state_timer_duration_ms_ = 3000;
            state_timer_active_ = true;
            std::cout << "⏰ Returning to IDLE in 3 seconds..." << std::endl;
            break;
            
        case SystemState::LOGOUT:
            serial_comm_->send_state(15); // Screen 15: "Deleting Data / Thank You"
            std::cout << "🗑️ DELETE: Showing Screen 15 for 3 seconds..." << std::endl;
            std::cout.flush();
            
            // Clean up session data
            {
                std::string saved_session = session_id_;  // Save for delete_ack
                session_id_.clear();
                platform_id_.clear();
                motion_tracker_.reset();
                
                // Use a thread to handle the 3 second animation then send ack
                std::thread([this, saved_session]() {
                    // Wait 3 seconds for delete animation
                    std::cout << "⏳ Waiting 3 seconds for delete animation..." << std::endl;
                    std::cout.flush();
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    
                    // AFTER 3 seconds, send delete_ack to server
                    std::cout << "📤 Sending delete_ack to server..." << std::endl;
                    std::cout.flush();
                    
                    bool ack_sent = false;
                    if (network_mgr_ && network_mgr_->is_connected()) {
                        nlohmann::json delete_ack = {
                            {"type", "delete_ack"},
                            {"session_id", saved_session},
                            {"status", "complete"}
                        };
                        ack_sent = network_mgr_->send_message(delete_ack.dump());
                        std::cout << "✅ delete_ack sent to server (success=" << ack_sent << ")" << std::endl;
                        std::cout.flush();
                    } else {
                        std::cout << "⚠️ WebSocket not connected, skipping delete_ack" << std::endl;
                        std::cout.flush();
                    }
                    
                    std::cout << "🔄 Cleaning up auth token..." << std::endl;
                    std::cout.flush();
                    
                    // Clear auth token
                    {
                        std::lock_guard<std::mutex> lock(ws_auth_mutex_);
                        pending_ws_token_.clear();
                    }
                    
                    // Stop WebSocket reconnection (non-blocking)
                    network_mgr_->stop_reconnect();
                    std::cout.flush();
                    
                    std::cout << "✅ Delete complete → Going to IDLE..." << std::endl;
                    std::cout.flush();
                    set_state(SystemState::IDLE);
                    std::cout << "🔄 IDLE state set - main loop restarted (Screen 6)" << std::endl;
                    std::cout.flush();
                }).detach();
            }
            break;
            
        default:
            break;
    }
}

void SystemController::configure_camera_for_state(SystemState state) {
    CameraConfig config;
    CameraMode target_mode;
    
    // Determine target camera mode based on state
    // LOW POWER (RGB-only): Boot, WiFi setup, Idle/Ready (QR + tracking without depth)
    // FULL MODE (RGB+Depth+IR): Reserved for future anti-spoofing / depth-enabled flows
    // 
    // Resolution: ALL streams use 848x480 for consistency
    // This matches RealSense default and ensures proper alignment between RGB/Depth/IR
    
    if (state == SystemState::BOOT ||
        state == SystemState::AWAIT_ADMIN_QR || 
        state == SystemState::IDLE ||
        state == SystemState::READY ||  // Keep READY in low-power RGB-only mode (no re-init)
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
        // FULL MODE: Color + Depth + IR (for anti-spoofing)
        target_mode = CameraMode::FULL;
        config.enable_ir = true;
        config.enable_depth = true;
        config.depth_width = 848;
        config.depth_height = 480;
        config.color_width = 848;
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
        config.auto_exposure = true;
    }
    
    
    // Only reconfigure if changing modes (low power → high power or vice versa)
    if (current_camera_mode_.load() == target_mode && producer_) {
        std::cout << "📹 Camera already in correct mode - no reconfiguration needed" << std::endl;
        return;
    }
    
    // **CRITICAL FIX**: Only stop camera when switching between modes
    // In current flow we keep READY/ALIGN in RGB-only, so mode changes are rare
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
    
    // Reset frame counter when not in ALIGN
    static int align_frame_count = 0;
    if (state != SystemState::ALIGN) {
        align_frame_count = 0;
    } else {
        align_frame_count++;
    }
    
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
    // Re-check state before processing
    SystemState current = current_state_.load();
    if (current != SystemState::IDLE && current != SystemState::READY) {
        return;
    }
    
    // Static scanner - reuse across calls (much more efficient)
    static zbar::ImageScanner scanner;
    static bool scanner_initialized = false;
    if (!scanner_initialized) {
        scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);  // Disable all
        scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);  // Enable QR only
        scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_X_DENSITY, 1);
        scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_Y_DENSITY, 1);
        scanner_initialized = true;
        std::cout << "📷 ZBar scanner initialized" << std::endl;
    }
    
    // Debounce
    static auto last_qr_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_qr_time).count();
    
    // Process every 3rd frame for better detection
    static int frame_count = 0;
    if (++frame_count % 3 != 0) return;

    cv::Mat color_img = frame->get_color_mat();
    if (color_img.empty()) {
        return;
    }
    
    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(color_img, gray, cv::COLOR_BGR2GRAY);
    
    // Create ZBar image from raw grayscale data
    zbar::Image zbar_image(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);
    
    // Scan for QR codes
    int n = scanner.scan(zbar_image);
    
    std::string decoded_info;
    if (n > 0) {
        for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
             symbol != zbar_image.symbol_end(); ++symbol) {
            decoded_info = symbol->get_data();
            
            // Log detected QR
            std::string preview = decoded_info.length() > 40 
                ? decoded_info.substr(0, 40) + "..." 
                : decoded_info;
            std::cout << "📱 QR[" << decoded_info.length() << "]: " << preview << std::endl;
            break;
        }
    }
    
    // Clean up zbar image
    zbar_image.set_data(NULL, 0);
    
    if (!decoded_info.empty()) {
        // Debounce: 2 second cooldown after valid QR
        if (elapsed < 2000) {
            return;
        }
        last_qr_time = std::chrono::steady_clock::now();
        
        // Filter noise (QR codes should be at least 10 chars)
        if (decoded_info.length() < 10) {
            return;
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
                qr_key = "fallback";
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
    
    static int frame_count = 0;
    frame_count++;
    
    // Run face detection
    if (!face_detector_->detect(frame)) {
        motion_tracker_.no_face_counter++;
        motion_tracker_.validation_state = FaceValidationState::NO_FACE;
        
        // Log no-face periodically
        static int no_face_log = 0;
        if (++no_face_log % 60 == 0) {
            std::cout << "👤 No face detected (count: " << motion_tracker_.no_face_counter << ")" << std::endl;
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
        
        
        serial_comm_->send_tracking_data(233, 233, 
            static_cast<int>(motion_tracker_.progress * 100), false);
        return;
    }
    
    static int face_detect_count = 0;
    face_detect_count++;
    
    // Validate face position - simplified (gates disabled for testing)
    motion_tracker_.validation_state = validate_face_position(frame);
    bool is_valid = (motion_tracker_.validation_state == FaceValidationState::VALID);
    
    // Reset no-face counter since we have a face
        if (frame->metadata.face_detected) {
            motion_tracker_.no_face_counter = 0;
    } else {
        motion_tracker_.no_face_counter++;
    }
    
    // Extract nose position (frame is portrait 480x848, rotated at source)
    float nose_x_rot = landmarks[4].x;  // X in rotated space (0-480)
    float nose_y_rot = landmarks[4].y;  // Y in rotated space (0-848)
    
    // Normalize to 0.0-1.0 using rotated dimensions
    float nose_x_norm = nose_x_rot / frame->metadata.rotated_width;   // /480
    float nose_y_norm = nose_y_rot / frame->metadata.rotated_height;  // /848
    
    // Map to 466x466 display with ASPECT RATIO CORRECTION
    // The Y axis (848px) has more range than X (480px), so we use center crop logic
    // to make vertical movement feel as responsive as horizontal
    
    // For X: full range 0-480 maps to 0-466 (with mirror)
    int screen_x = static_cast<int>(nose_x_norm * 466.0f);
    
    // For Y: use center 480px of the 848px range for 1:1 feel
    // Center of 848 = 424, so usable range is 424±240 = 184 to 664
    float y_center = 0.5f;  // Normalized center
    float y_range = 480.0f / 848.0f;  // ~0.566 of the full range
    float y_adjusted = (nose_y_norm - (y_center - y_range/2)) / y_range;  // Remap to 0-1
    y_adjusted = std::max(0.0f, std::min(1.0f, y_adjusted));  // Clamp
    int screen_y = static_cast<int>(y_adjusted * 466.0f);
    
    // Mirror X-axis (camera sees mirror image of user)
    screen_x = 465 - screen_x;
    
    // Clamp to display bounds
    screen_x = std::max(0, std::min(465, screen_x));
    screen_y = std::max(0, std::min(465, screen_y));
    
    // Calculate progress only if face position is valid
    float progress = motion_tracker_.progress;
    if (is_valid) {
        progress = calculate_circular_motion_progress(cv::Point2f(nose_x_norm, nose_y_norm));
    }
    
    // Send combined tracking data to display
    serial_comm_->send_tracking_data(screen_x, screen_y, 
        static_cast<int>(progress * 100), is_valid);
    
    
                
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
                
    // Check for spiral completion (real spiral motion required)
    if (progress >= 1.0f) {
        // Send 100% progress to display to ensure ring is fully filled
        serial_comm_->send_tracking_data(screen_x, screen_y, 100, true);
        
        std::cout << "🎉 Spiral complete! Celebrating for 1 second..." << std::endl;
        std::cout.flush();
        
        // Wait 1 second to let user see the completed ring
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        std::cout << "✅ Celebration done → PROCESSING" << std::endl;
        set_state(SystemState::PROCESSING);
        return;
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

bool SystemController::is_face_orientation_extreme(const std::vector<FrameBoxMetadata::Landmark>& /*landmarks*/) {
    // DISABLED FOR TESTING - always allow face
    return false;
}

SystemController::FaceValidationState SystemController::validate_face_position(FrameBox* frame) {
    // SIMPLIFIED FOR TESTING - just check if face is detected
    if (!frame || !frame->metadata.face_detected) {
        return FaceValidationState::NO_FACE;
    }
    
    if (frame->metadata.landmarks.size() < 468) {
        return FaceValidationState::NO_FACE;
    }
    
    // Always valid if face detected - no distance/orientation gates
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
    
    // Filter out noise: ignore very large jumps (face rotation/landmark shift)
    float velocity = std::abs(delta);
    if (velocity > 0.8f) {  // Jump > ~45 degrees in one frame = noise
        return motion_tracker_.progress;
    }
    
    // Minimum movement threshold to count as progress
    if (velocity < 0.01f) {
        return motion_tracker_.progress;
    }
    
    // Accumulate angular displacement (no boost, no assist - real movement only)
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
    std::cout << "🔬 Starting Processing..." << std::endl;
    
    // ============================================
    // STEP 1: Retrieve recorded frames
    // ============================================
    serial_comm_->send_batch_progress(10);
    
    std::cout << "📊 Retrieving recorded frames from ring buffer..." << std::endl;
    std::vector<FrameBox*> recording = ring_buffer_->get_all_valid_frames();
    std::cout << "📊 Retrieved " << recording.size() << " frames for processing" << std::endl;
    
    if (recording.empty()) {
        std::cerr << "⚠ No recording data" << std::endl;
        serial_comm_->send_error("No recording data");
        network_mgr_->send_message("{\"type\":\"error\", \"code\":\"no_recording_data\"}");
        ring_buffer_->clear();
        set_state(SystemState::ERROR);
        return;
    }
    
    serial_comm_->send_batch_progress(20);

    // ============================================
    // STEP 2: Find best frame with face
    // ============================================
    FrameBox* best_frame = nullptr;
    for (FrameBox* frame : recording) {
        if (frame->metadata.face_detected &&
            frame->metadata.landmarks.size() >= 468) {
            best_frame = frame;
        }
    }
    
    if (!best_frame) {
        best_frame = recording.back();
    }
    
    serial_comm_->send_batch_progress(40);

    // ============================================
    // STEP 3: Encode image
    // ============================================
    cv::Mat rgb_frame = best_frame->get_color_mat();
    if (rgb_frame.empty()) {
        std::cerr << "❌ Best frame is empty - cannot submit" << std::endl;
        serial_comm_->send_error("Camera error");
        ring_buffer_->clear();
        set_state(SystemState::ERROR);
        return;
    }
    
    std::vector<uchar> jpeg_buffer;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};
    cv::imencode(".jpg", rgb_frame, jpeg_buffer, compression_params);
    
    serial_comm_->send_batch_progress(60);
    
    // Save captured frame for debugging
    std::string debug_path = "/home/mercleDev/mdai_logs/captured_face.jpg";
    cv::imwrite(debug_path, rgb_frame);
    std::cout << "📸 Face image saved to: " << debug_path << std::endl;
    
    std::vector<uint8_t> buffer_uint8(jpeg_buffer.begin(), jpeg_buffer.end());
    std::string base64_image = CryptoUtils::base64_encode(buffer_uint8);
    
    std::cout << "📸 JPEG size: " << jpeg_buffer.size() << " bytes, Base64: " << base64_image.size() << " bytes" << std::endl;
    
    // Clear ring buffer now that we have the image
    ring_buffer_->clear();
    
    serial_comm_->send_batch_progress(80);
    
    // ============================================
    // STEP 4: Send to API server
    // ============================================
    std::cout << "📤 Sending result to API server..." << std::endl;
    
    nlohmann::json api_payload;
    api_payload["type"] = "submit_result";
    api_payload["status"] = "success";
    api_payload["platform_id"] = platform_id_;
    api_payload["session_id"] = session_id_;
    api_payload["image"] = base64_image;
    api_payload["consecutive_passes"] = 1;
    api_payload["timestamp"] = std::chrono::system_clock::now().time_since_epoch().count();
        
    std::string api_message = api_payload.dump();
    std::cout << "📤 Sending " << api_message.size() << " bytes to server..." << std::endl;
    
    result_ack_received_ = false;
    
    if (!network_mgr_->send_message(api_message)) {
        std::cerr << "❌ Failed to send result to server" << std::endl;
        serial_comm_->send_error("Failed to send result");
        set_state(SystemState::ERROR);
        return;
    }
    
    // Wait for server ACK (up to 10 seconds)
    auto send_time = std::chrono::steady_clock::now();
    const int ack_timeout_ms = 10000;
    
    while (!result_ack_received_) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - send_time
        ).count();
        
        if (elapsed >= ack_timeout_ms) {
            std::cerr << "⚠️  Server ACK timeout after 10 seconds" << std::endl;
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    if (result_ack_received_) {
        std::cout << "✅ Server ACK received!" << std::endl;
    }
    
    serial_comm_->send_batch_progress(100);
    
    // ============================================
    // STEP 5: Go to SUCCESS
    // ============================================
    if (current_state_ == SystemState::PROCESSING) {
        set_state(SystemState::SUCCESS);
    } else {
        std::cout << "⚠️ State changed during processing - not overriding to SUCCESS" << std::endl;
    }
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
                            std::cout << "🔢 Starting COUNTDOWN phase" << std::endl;
                            set_state(SystemState::COUNTDOWN);
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
                    std::cout << "🔢 Starting COUNTDOWN phase" << std::endl;
                    set_state(SystemState::COUNTDOWN);
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
        else if (msg_type == "ping") {
            // Server keepalive ping - respond with pong
            nlohmann::json pong_msg = {{"type", "pong"}};
            network_mgr_->send_message(pong_msg.dump());
        }
        else if (msg_type == "session_closed") {
            // Server is closing the session (e.g., expired, server shutdown)
            std::string reason = msg_json.value("reason", "Session closed");
            std::cout << "⚠️ Session closed by server: " << reason << std::endl;
            serial_comm_->send_error(reason);
            set_state(SystemState::ERROR);
        }
        else if (msg_type == "delete") {
            // Delete/cleanup request from mobile via server
            std::cout << "🗑️ Delete request received from mobile" << std::endl;
            
            // Go to LOGOUT if we're in SUCCESS state (waiting for delete)
            // Also handle if we're in other active states
            if (current_state_ == SystemState::SUCCESS) {
                std::cout << "📱 SUCCESS → LOGOUT (delete received)" << std::endl;
                set_state(SystemState::LOGOUT);
            }
            else if (current_state_ != SystemState::LOGOUT &&
                     current_state_ != SystemState::IDLE) {
                std::cout << "📱 Active state → LOGOUT (delete received)" << std::endl;
                set_state(SystemState::LOGOUT);
            }
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
