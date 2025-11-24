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

namespace mdai {

SystemController::SystemController() {
    ring_buffer_ = std::make_unique<DynamicRingBuffer>(32, 6ULL * 1024 * 1024 * 1024);
    
    SerialCommunicator::Config serial_cfg;
    // Auto-detect ttyACM device (ESP32 can be on ACM0 or ACM1)
    std::ifstream acm0_check("/dev/ttyACM0");
    std::ifstream acm1_check("/dev/ttyACM1");
    
    if (acm0_check.good()) {
        serial_cfg.port_name = "/dev/ttyACM0";
    } else if (acm1_check.good()) {
        serial_cfg.port_name = "/dev/ttyACM1";
    } else {
        serial_cfg.port_name = "/dev/ttyACM0";  // Default fallback
    }
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
    
    // Boot sequence: Show logo, then check WiFi
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
            
        case SystemState::PROVISIONED:
            serial_comm_->send_state(4); // Screen 4: "WiFi Connected" screen (show SSID, 3s)
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(3));
                serial_comm_->send_state(5); // Screen 5: "mDai Ready" screen (2s preemptive)
                std::this_thread::sleep_for(std::chrono::seconds(2));
                set_state(SystemState::IDLE);
            }).detach();
            break;
            
        case SystemState::IDLE:
            serial_comm_->send_state(5); // Screen 5: "Waiting for User QR" (IDLE phase)
            configure_camera_for_state(SystemState::IDLE);
            break;
            
        case SystemState::PLACEHOLDER_SCREEN_7:
            // ═══════════════════════════════════════════════════════════════════
            // SCREEN 7: PLACEHOLDER - TO BE IMPLEMENTED LATER
            // ═══════════════════════════════════════════════════════════════════
            // This is a dummy placeholder as requested. No complex logic here.
            // Just displays a screen and waits for 3 seconds before continuing.
            serial_comm_->send_state(7); // Screen 7: Placeholder screen
            std::cout << "📍 [PLACEHOLDER] Screen 7 displayed (dummy implementation)" << std::endl;
            
            // Auto-transition after 3 seconds (dummy behavior)
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(3));
                if (current_state_ == SystemState::PLACEHOLDER_SCREEN_7) {
                    std::cout << "📍 [PLACEHOLDER] Auto-transitioning from Screen 7 → READY" << std::endl;
                    set_state(SystemState::READY);
                }
            }).detach();
            break;
            
        case SystemState::READY:
            serial_comm_->send_state(6); // Screen 6: "Ready?" (waiting for Start button)
            configure_camera_for_state(SystemState::READY);
            
            // Safety Timeout: If user doesn't start within 60 seconds, go back to IDLE
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(60));
                if (current_state_ == SystemState::READY) {
                    std::cout << "⚠️  Session Timeout (User didn't start)" << std::endl;
                    network_mgr_->send_message("{\"type\":\"error\", \"code\":\"timeout_ready\", \"message\":\"User did not start verification\"}");
                    network_mgr_->disconnect();
                    set_state(SystemState::IDLE);
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
            serial_comm_->send_state(7); // Screen 7: "Nose Tracking" with target
            motion_tracker_.reset();
            ring_buffer_->clear();
            ring_buffer_->set_recording_active(true); 
            break;
            
        case SystemState::PROCESSING:
            ring_buffer_->set_recording_active(false); 
            serial_comm_->send_state(10); // Screen 10: "Processing" screen
            std::thread([this]() { handle_processing(); }).detach();
            break;
            
        case SystemState::SUCCESS:
            serial_comm_->send_state(8); // Screen 8: "Success" screen
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(5));
                set_state(SystemState::LOGOUT);
            }).detach();
            break;
            
        case SystemState::ERROR:
            serial_comm_->send_state(9); // Screen 9: "Error" screen
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(5));
                set_state(SystemState::IDLE);
            }).detach();
            break;
            
        case SystemState::LOGOUT:
            serial_comm_->send_state(13); // Screen 13: "Thank You" screen
            // Switch camera back: Full → Low Power (camera stays on, just changes mode)
            configure_camera_for_state(SystemState::IDLE);
            std::cout << "🔄 Camera switching: High Power → Low Power (persistent)" << std::endl;
            
            // Close WebSocket connection
            network_mgr_->disconnect();
            
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(3));
                set_state(SystemState::IDLE);
            }).detach();
            break;
            
        default:
            break;
    }
}

void SystemController::configure_camera_for_state(SystemState state) {
    CameraConfig config;
    CameraMode target_mode;
    
    // Determine target camera mode based on state
    if (state == SystemState::BOOT ||
        state == SystemState::AWAIT_ADMIN_QR || 
        state == SystemState::IDLE ||
        state == SystemState::READY ||  // READY state stays in RGB_ONLY (QR scanning only)
        state == SystemState::WIFI_CHANGE_CONNECTING ||
        state == SystemState::WIFI_CHANGE_SUCCESS ||
        state == SystemState::WIFI_CHANGE_FAILED) {
        
        target_mode = CameraMode::RGB_ONLY;
        config.enable_ir = false;
        config.enable_depth = false;
        config.emitter_enabled = 0;
        config.laser_power = 0.0f;
        config.align_to_color = false;
        config.color_width = 1280;  // Increased from 640 for better QR detection
        config.color_height = 720;   // Increased from 480 for better QR detection
        config.color_fps = 30;
        config.depth_width = 0;
        config.depth_height = 0;
        config.enable_spatial_filter = false;
        config.enable_temporal_filter = false;
        config.enable_hole_filling = false;
        config.auto_exposure = true;  // Let the camera adapt lighting for QR readability
        
    } else {
        target_mode = CameraMode::FULL;
        config.enable_ir = true;
        config.depth_width = 848;
        config.depth_height = 480;
        config.color_width = 1280;
        config.color_height = 720;
        config.color_fps = 30;
        config.align_to_color = true;
        config.laser_power = 300.0f;
    }
    
    // **CRITICAL**: Camera should NEVER be stopped once started
    // Only reconfigure if changing modes (low power → high power or vice versa)
    if (current_camera_mode_.load() == target_mode && producer_) {
        std::cout << "📹 Camera already in correct mode - no reconfiguration needed" << std::endl;
        return;
    }
    
    // **CRITICAL FIX**: Only stop camera when switching from LOW to HIGH power
    // LOW POWER mode should NEVER be closed once started
    if (current_camera_mode_.load() == CameraMode::RGB_ONLY && target_mode == CameraMode::FULL) {
        // Switching from low power to high power - safe to stop and restart
        std::cout << "📹 Camera: Upgrading from Low Power → High Power" << std::endl;
        if (producer_) {
            producer_->stop();
            // CRITICAL: Wait for hardware to fully release resources
            std::cout << "⏳ Waiting 2 seconds for camera hardware to release..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    } else if (current_camera_mode_.load() == CameraMode::FULL && target_mode == CameraMode::RGB_ONLY) {
        // Switching from high power to low power - stop high power mode
        std::cout << "📹 Camera: Downgrading from High Power → Low Power" << std::endl;
        if (producer_) {
            producer_->stop();
            // CRITICAL: Wait for hardware to fully release resources
            std::cout << "⏳ Waiting 2 seconds for camera hardware to release..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    } else if (current_camera_mode_.load() == CameraMode::UNINITIALIZED) {
        // First time initialization
        std::cout << "📹 Camera: Initial startup in " << (target_mode == CameraMode::RGB_ONLY ? "Low Power" : "High Power") << " mode" << std::endl;
    }
    
    // Log mode change
    if (target_mode == CameraMode::RGB_ONLY) {
        std::cout << "📹 Camera: RGB-only mode (" << config.color_width << "x" 
                  << config.color_height << "@" << config.color_fps << "fps) - Low Power - PERSISTENT" << std::endl;
    } else {
        std::cout << "📹 Camera: Full mode (1280x720 RGB + 848x480 Depth + IR + Laser)" << std::endl;
    }
    
    producer_ = std::make_unique<Producer>(config, ring_buffer_.get());
    producer_->start();
    
    // Update current mode tracker
    current_camera_mode_.store(target_mode);
}

void SystemController::process_frame(FrameBox* frame) {
    SystemState state = current_state_.load();
    switch (state) {
        case SystemState::AWAIT_ADMIN_QR: handle_await_admin_qr(frame); break;
        case SystemState::IDLE: handle_idle(frame); break;
        case SystemState::PLACEHOLDER_SCREEN_7: break;  // Placeholder - no processing
        case SystemState::READY: handle_idle(frame); break;  // READY also scans for session QR
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
        std::cout << "📱 QR Code Detected (Idle State)" << std::endl;
        
        // STEP 1: Detect QR type by size and structure
        // Session QR: Raw encrypted 8-char token = ~44-88 base64 chars
        // WiFi QR: Much longer (~150+ chars)
        bool is_session_qr = false;
        bool is_wifi_qr = false;
        std::string encrypted_data;
        
        // First, check if it's JSON with qr_encrypted field (legacy format)
        try {
            nlohmann::json qr_json = nlohmann::json::parse(decoded_info);
            
            if (qr_json.contains("qr_encrypted")) {
                is_session_qr = true;
                encrypted_data = qr_json["qr_encrypted"].get<std::string>();
                std::cout << "🔐 Detected: SESSION QR (JSON format)" << std::endl;
            } else {
                // JSON but no qr_encrypted = WiFi QR
                is_wifi_qr = true;
                encrypted_data = decoded_info;
                std::cout << "🔐 Detected: WIFI QR (JSON format)" << std::endl;
            }
        } catch (...) {
            // Not JSON - check for Session QR prefix "S:" or size
            if (decoded_info.length() > 2 && decoded_info.substr(0, 2) == "S:") {
                // Session QR with "S:" prefix (remove prefix before decryption)
                is_session_qr = true;
                encrypted_data = decoded_info.substr(2);  // Remove "S:" prefix
                std::cout << "🔐 Detected: SESSION QR (S: prefix, " << encrypted_data.length() << " chars)" << std::endl;
            } else if (decoded_info.length() < 100) {
                // Legacy Session QR without prefix
                is_session_qr = true;
                encrypted_data = decoded_info;
                std::cout << "🔐 Detected: SESSION QR (legacy format, " << decoded_info.length() << " chars)" << std::endl;
            } else {
                is_wifi_qr = true;
                encrypted_data = decoded_info;
                std::cout << "🔐 Detected: WIFI QR (raw format, " << decoded_info.length() << " chars)" << std::endl;
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
                std::string ws_token = CryptoUtils::aes256_decrypt(encrypted_data, qr_key);
                std::cout << "✅ QR decrypted successfully" << std::endl;
                
                // Decrypted QR contains just the HMAC token (8 chars after stripping "S:" prefix)
                if (ws_token.empty() || ws_token.length() != 8) {
                    std::cerr << "⚠ Invalid token in QR (expected 8 chars)" << std::endl;
                    return;
                }
                
                std::cout << "🔐 Token (HMAC): " << ws_token << std::endl;
                
                // Connect to WebSocket (async - wait for connection)
                std::cout << "🔌 Connecting to WebSocket..." << std::endl;
                
                // No session_id in path - server will lookup by ws_token
                std::string ws_path = "/ws/device";
                
                // Start connection (spawns background thread)
                network_mgr_->connect_to_middleware(middleware_host_, 443, ws_path, device_id_);
                
                // Wait for connection to be established (max 10 seconds)
                int wait_count = 0;
                while (!network_mgr_->is_connected() && wait_count < 100) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    wait_count++;
                }
                
                if (network_mgr_->is_connected()) {
                    std::cout << "✅ WebSocket connected" << std::endl;
                    
                    // Send authentication message (no timestamp - server uses DB created_at)
                    nlohmann::json auth_msg = {
                        {"type", "auth"},
                        {"bearer_token", ws_token},
                        {"device_id", device_id_}
                    };
                    
                    network_mgr_->send_message(auth_msg.dump());
                    std::cout << "📤 Sent auth message with 8-char HMAC token" << std::endl;
                    
                    // Note: session_id will be determined by server from ws_token
                    // State will change to PLACEHOLDER_SCREEN_7 after receiving auth_success from server
                } else {
                    std::cerr << "❌ WebSocket connection failed (timeout)" << std::endl;
                    serial_comm_->send_state(9); // Error screen
                }
                
            } catch (const std::exception& e) {
                std::cerr << "⚠ Session QR Decryption Error: " << e.what() << std::endl;
                serial_comm_->send_state(9); // Error screen
            }
        }
        // STEP 3: Handle WiFi QR (from admin portal)
        else if (is_wifi_qr) {
            // User scanned WiFi QR in IDLE state - wrong QR type!
            std::cerr << "⚠ WiFi QR scanned in IDLE state - need Session QR!" << std::endl;
            serial_comm_->send_state(9); // Show error screen
            std::this_thread::sleep_for(std::chrono::seconds(3));
            serial_comm_->send_state(5); // Back to "Scan QR" screen
            return;
        }
    }
#endif
}

void SystemController::handle_warmup(FrameBox* frame) {
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
    if (face_detector_->detect(frame)) {
        if (frame->metadata.face_detected) {
            motion_tracker_.no_face_counter = 0;
            
            auto& landmarks = frame->metadata.landmarks;
            if (landmarks.size() > 4) {
                float nose_x = landmarks[4].x / frame->color_width;
                float nose_y = landmarks[4].y / frame->color_height;
                
                // Send to LilyGo screen
                serial_comm_->send_nose_position(nose_x, nose_y);
                
                float progress = calculate_circular_motion_progress(cv::Point2f(nose_x, nose_y));
                serial_comm_->send_progress(static_cast<int>(progress * 100));
                
                // Send detailed progress to mobile app via WebSocket
                static int mobile_update_skip = 0;
                if (mobile_update_skip++ % 10 == 0) {
                    nlohmann::json progress_msg = {
                        {"type", "progress"},
                        {"state", "align"},
                        {"progress", static_cast<int>(progress * 100)},
                        {"nose_position", {
                            {"x", nose_x},
                            {"y", nose_y}
                        }}
                    };
                    network_mgr_->send_message(progress_msg.dump());
                }
                
                if (progress >= 1.0f) {
                    set_state(SystemState::PROCESSING);
                }
            }
        } else {
             motion_tracker_.no_face_counter++;
        }
    } else {
        motion_tracker_.no_face_counter++;
    }
    
    // Timeout: 3 seconds (90 frames)
    if (motion_tracker_.no_face_counter > 90) {
        std::cout << "Face Timeout!" << std::endl;
        nlohmann::json error_msg = {
            {"type", "error"},
            {"code", "timeout_no_face"},
            {"message", "No face detected for 3 seconds"}
        };
        network_mgr_->send_message(error_msg.dump());
        
        ring_buffer_->set_recording_active(false);
        ring_buffer_->clear(); 
        
        set_state(SystemState::ERROR);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        set_state(SystemState::IDLE);
    }
}

float SystemController::calculate_circular_motion_progress(const cv::Point2f& nose) {
    motion_tracker_.nose_positions.push_back(nose);
    if (motion_tracker_.nose_positions.size() > 150) {
        motion_tracker_.nose_positions.erase(motion_tracker_.nose_positions.begin());
    }
    if (motion_tracker_.nose_positions.size() < 30) return 0.0f;
    
    // Quadrant Logic
    cv::Point2f center(0,0);
    for(const auto& p : motion_tracker_.nose_positions) center += p;
    center.x /= motion_tracker_.nose_positions.size();
    center.y /= motion_tracker_.nose_positions.size();
    
    bool q1=false, q2=false, q3=false, q4=false;
    for(const auto& p : motion_tracker_.nose_positions) {
        float dx = p.x - center.x;
        float dy = p.y - center.y;
        if(dx > 0.05 && dy > 0.05) q1 = true;
        if(dx < -0.05 && dy > 0.05) q2 = true;
        if(dx < -0.05 && dy < -0.05) q3 = true;
        if(dx > 0.05 && dy < -0.05) q4 = true;
    }
    
    if (q1 && q2 && q3 && q4) {
        motion_tracker_.progress += 0.02f; 
    }
    
    // Send progress update to WS periodically
    static int update_skip = 0;
    if (update_skip++ % 10 == 0) {
        std::string msg = "{\"type\":\"progress\", \"value\":" + std::to_string(motion_tracker_.progress) + "}";
        network_mgr_->send_message(msg);
    }
    
    return (motion_tracker_.progress > 1.0f) ? 1.0f : motion_tracker_.progress;
}

void SystemController::handle_processing() {
    std::cout << "🔬 Starting Batch Processing..." << std::endl;
    
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
        
        // Extract the best frame as base64 for API submission
        cv::Mat rgb_frame = best_frame->get_color_mat();
        std::vector<uchar> buffer;
        cv::imencode(".jpg", rgb_frame, buffer);
        std::vector<uint8_t> buffer_uint8(buffer.begin(), buffer.end());
        std::string base64_image = CryptoUtils::base64_encode(buffer_uint8);
        
        // Send to WebSocket API
        nlohmann::json api_payload;
        api_payload["type"] = "submit_result";
        api_payload["platform_id"] = platform_id_;
        api_payload["session_id"] = session_id_;
        api_payload["image"] = base64_image;
        api_payload["consecutive_passes"] = max_consecutive;
        api_payload["timestamp"] = std::chrono::system_clock::now().time_since_epoch().count();
        
        std::string api_message = api_payload.dump();
        std::cout << "📤 Sending result to API..." << std::endl;
        network_mgr_->send_message(api_message);
        
        // Wait for server response (handled in on_websocket_message)
        // For now, assume success after 2 seconds
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        set_state(SystemState::SUCCESS);
        
    } else {
        std::cout << "❌ Anti-spoofing FAILED (only " << max_consecutive << " consecutive passes)" << std::endl;
        set_state(SystemState::ERROR);
        network_mgr_->send_message("{\"type\":\"result\", \"status\":\"failed\", \"reason\":\"insufficient_liveness\"}");
    }
    
    ring_buffer_->clear(); 
}

void SystemController::on_websocket_message(const std::string& message) {
    std::cout << "📨 WS Message: " << message << std::endl;
    
    try {
        nlohmann::json msg_json = nlohmann::json::parse(message);
        
        std::string msg_type = msg_json.value("type", "");
        
        if (msg_type == "auth_success") {
            // WebSocket authentication successful
            std::cout << "✅ WebSocket authenticated successfully" << std::endl;
            
            // Server already notifies mobile with "device_connected"
            // No need to send redundant "device_ready" message
            
            // Enter PLACEHOLDER_SCREEN_7 first (dummy screen)
            std::cout << "📍 Entering PLACEHOLDER Screen 7 (will auto-transition)" << std::endl;
            set_state(SystemState::PLACEHOLDER_SCREEN_7);
        }
        // Handle start command (server forwards as plain JSON without "type" wrapper)
        else if (msg_json.contains("command")) {
            std::string command = msg_json.value("command", "");
            
            if (command == "start_verification" || command == "start") {
                std::cout << "▶️  Received start command from mobile" << std::endl;
                
                if (current_state_ == SystemState::READY) {
                    if (test_mode_) {
                        // 🧪 TEST MODE: Skip verification, show success immediately
                        std::cout << "🧪 TEST MODE: Bypassing verification flow → SUCCESS" << std::endl;
                        serial_comm_->send_state(8); // Screen 8: Success
                        
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
                        // REAL MODE: Do actual verification
                        std::cout << "🔥 Starting WARMUP phase" << std::endl;
                        set_state(SystemState::WARMUP);
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
                        serial_comm_->send_state(5); // Show "mDai Ready"
                        std::this_thread::sleep_for(std::chrono::seconds(2));
                        set_state(SystemState::IDLE);
                    } else {
                        std::cerr << "❌ Camera retry failed" << std::endl;
                        serial_comm_->send_error("Camera retry failed - check hardware");
                        serial_comm_->send_state(9); // Error screen
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
        else if (msg_type == "verification_result" || msg_type == "api_response") {
            // Server forwarded API response
            std::string status = msg_json.value("status", "");
            
            if (status == "success") {
                std::cout << "✅ Verification SUCCESS" << std::endl;
                set_state(SystemState::SUCCESS);
            } else {
                std::cout << "❌ Verification FAILED" << std::endl;
                std::string reason = msg_json.value("reason", "unknown");
                std::cerr << "Reason: " << reason << std::endl;
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
        serial_comm_->send_state(9); // Screen 9: Error screen (was 11, but 9 is correct per state mapping)
        
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
        // Go directly to Screen 4: WiFi Connected
        serial_comm_->send_state(4); // Screen 4: "WiFi Connected" (show SSID)
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // Screen 5: "mDai Ready" (2 seconds preemptive)
        serial_comm_->send_state(5);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        set_state(SystemState::IDLE);
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
        serial_comm_->send_state(2); // Reuse Screen 2 for "Connecting to WiFi"
        
        if (network_mgr_->connect_wifi(wifi_ssid, wifi_password)) {
            previous_ssid_ = wifi_ssid;
            previous_password_ = wifi_password;
            
            set_state(SystemState::WIFI_CHANGE_SUCCESS);
            serial_comm_->send_state(4); // Screen 4: "WiFi Success" (show SSID)
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            serial_comm_->send_state(5); // Screen 5: "mDai Ready"
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            set_state(SystemState::IDLE);
            return true;
        } else {
            // Failed, fallback
            set_state(SystemState::WIFI_CHANGE_FAILED);
            serial_comm_->send_state(13); // Screen 13: "WiFi Failed" error screen
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

} // namespace mdai
