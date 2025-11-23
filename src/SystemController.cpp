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
            serial_comm_->send_state(3); // "Scan WiFi QR" screen
            configure_camera_for_state(SystemState::AWAIT_ADMIN_QR);
            break;
            
        case SystemState::PROVISIONING:
            serial_comm_->send_state(2); // "Connecting to WiFi" screen
            break;
            
        case SystemState::PROVISIONED:
            serial_comm_->send_state(3); // "WiFi Connected" screen (3s)
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(3));
                serial_comm_->send_state(4); // "mDai Ready" screen (3s)
                std::this_thread::sleep_for(std::chrono::seconds(3));
                set_state(SystemState::IDLE);
            }).detach();
            break;
            
        case SystemState::IDLE:
            serial_comm_->send_state(5); // "Scan QR Code" screen
            configure_camera_for_state(SystemState::IDLE);
            break;
            
        case SystemState::READY:
            serial_comm_->send_state(6); // "Connected - Waiting for Start" screen
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
            serial_comm_->send_state(6); // Keep showing "Ready?" screen (no change)
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
            serial_comm_->send_state(7); // "Nose Tracking" screen
            motion_tracker_.reset();
            ring_buffer_->clear();
            ring_buffer_->set_recording_active(true); 
            break;
            
        case SystemState::PROCESSING:
            ring_buffer_->set_recording_active(false); 
            serial_comm_->send_state(2); // "Processing" screen (reuse Screen 2 with text update)
            // Update screen 2 text to "Processing..." via serial command
            std::thread([this]() { handle_processing(); }).detach();
            break;
            
        case SystemState::SUCCESS:
            serial_comm_->send_state(8); // "Success" screen
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(5));
                set_state(SystemState::LOGOUT);
            }).detach();
            break;
            
        case SystemState::ERROR:
            serial_comm_->send_state(9); // "Error" screen
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(5));
                set_state(SystemState::IDLE);
            }).detach();
            break;
            
        case SystemState::LOGOUT:
            serial_comm_->send_state(10); // "Thank You" screen
            // Switch camera back: Full → RGB-only (happens in background)
            configure_camera_for_state(SystemState::IDLE);
            std::cout << "🔄 Camera switching: Full → RGB-only (cooling down)" << std::endl;
            
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
        state == SystemState::WIFI_CHANGE_CONNECTING ||
        state == SystemState::WIFI_CHANGE_SUCCESS ||
        state == SystemState::WIFI_CHANGE_FAILED) {
        
        target_mode = CameraMode::RGB_ONLY;
        config.enable_ir = false;
        config.color_width = 640;
        config.color_height = 480;
        config.color_fps = 30;
        
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
    
    // **OPTIMIZATION**: Skip reconfiguration if already in correct mode
    if (current_camera_mode_.load() == target_mode && producer_) {
        std::cout << "📹 Camera already in correct mode - skipping reconfiguration" << std::endl;
        return;
    }
    
    // Stop existing producer
    if (producer_) producer_->stop();
    
    // Log mode change
    if (target_mode == CameraMode::RGB_ONLY) {
        std::cout << "📹 Camera: RGB-only mode (640x480@30fps) - Low Power" << std::endl;
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
    
    // Use ZBar instead of OpenCV QRCodeDetector
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
    
    zbar::Image zbar_image(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);
    int n = scanner.scan(zbar_image);
    
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
            break; // Take first QR code
        }
    }
    
    if (!decoded_info.empty()) {
        std::cout << "📱 QR Code Detected (Idle State)" << std::endl;
        
        // STEP 1: Detect QR type by structure (JSON wrapper check)
        bool is_session_qr = false;
        bool is_wifi_qr = false;
        std::string encrypted_data;
        
        try {
            nlohmann::json qr_json = nlohmann::json::parse(decoded_info);
            
            // Session QR has "qr_encrypted" field (JSON wrapper)
            if (qr_json.contains("qr_encrypted")) {
                is_session_qr = true;
                encrypted_data = qr_json["qr_encrypted"].get<std::string>();
                std::cout << "🔐 Detected: SESSION QR (has qr_encrypted field)" << std::endl;
            } 
            // If no qr_encrypted field, could be raw WiFi QR or needs device_key
            else {
                is_wifi_qr = true;
                encrypted_data = decoded_info;  // Entire QR is encrypted data
                std::cout << "🔐 Detected: WIFI QR (no qr_encrypted field)" << std::endl;
            }
        } catch (...) {
            // Not JSON, treat as raw encrypted WiFi QR
            is_wifi_qr = true;
            encrypted_data = decoded_info;
            std::cout << "🔐 Detected: WIFI QR (not JSON format)" << std::endl;
        }
        
        // STEP 2: Handle Session QR (from mobile app)
        if (is_session_qr) {
            std::string qr_key = "mdai_qr_encryption_key_32byte!";  // Shared key
            
            try {
                std::string decrypted_json = CryptoUtils::aes256_decrypt(encrypted_data, qr_key);
                std::cout << "✅ QR decrypted successfully" << std::endl;
                
                nlohmann::json session_data = nlohmann::json::parse(decrypted_json);
                
                // Extract session_id and token
                std::string session_id = session_data.value("session_id", "");
                std::string ws_token = session_data.value("token", "");
                int64_t timestamp = session_data.value("timestamp", 0);
                
                if (session_id.empty() || ws_token.empty()) {
                    std::cerr << "⚠ Invalid session data in QR" << std::endl;
                    return;
                }
                
                std::cout << "🔑 Session ID: " << session_id << std::endl;
                std::cout << "⏰ QR Timestamp: " << timestamp << std::endl;
                
                // Check if QR is too old (> 5 minutes)
                int64_t current_time = std::time(nullptr);
                if (current_time - timestamp > 300) {
                    std::cerr << "⚠ QR code expired (>5 minutes old)" << std::endl;
                    serial_comm_->send_state(9); // Error screen
                    return;
                }
                
                // Connect to WebSocket
                std::cout << "🔌 Connecting to WebSocket..." << std::endl;
                
                std::string ws_path = "/ws/device?session_id=" + session_id_;
                
                if (network_mgr_->connect_to_middleware(middleware_host_, 443, ws_path, device_id_)) {
                    std::cout << "✅ WebSocket connected" << std::endl;
                    
                    // Send authentication message
                    nlohmann::json auth_msg = {
                        {"type", "auth"},
                        {"token", ws_token},
                        {"device_id", device_id_},
                        {"session_id", session_id}
                    };
                    
                    network_mgr_->send_message(auth_msg.dump());
                    std::cout << "📤 Sent auth message" << std::endl;
                    
                    // Store session info
                    session_id_ = session_id;
                    
                    // State will change to READY after receiving auth_success from server
                } else {
                    std::cerr << "❌ WebSocket connection failed" << std::endl;
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
            
            // Send "device_ready" signal to mobile
            nlohmann::json ready_msg = {
                {"type", "device_ready"},
                {"device_id", device_id_},
                {"message", "Device is ready for face verification"}
            };
            network_mgr_->send_message(ready_msg.dump());
            std::cout << "📤 Sent 'device_ready' signal to mobile" << std::endl;
            
            // Enter READY state and wait for user to click "Start" on mobile
            std::cout << "⏸️  Waiting for user to start verification..." << std::endl;
            set_state(SystemState::READY);
        }
        else if (msg_type == "start_verification") {
            // Mobile sent start command after user clicked "Start"
            std::cout << "▶️  Received 'start_verification' command from mobile" << std::endl;
            
            if (current_state_ == SystemState::READY) {
                std::cout << "🔥 Starting WARMUP phase" << std::endl;
                set_state(SystemState::WARMUP);
            } else {
                std::cerr << "⚠️  Received start_verification but not in READY state" << std::endl;
            }
        }
        else if (msg_type == "to_device") {
            // Mobile sent message to device (forwarded by server)
            if (msg_json.contains("data")) {
                auto data = msg_json["data"];
                std::string command = data.value("command", "");
                
                if (command == "start_verification" || command == "start") {
                    std::cout << "▶️  Received start command from mobile" << std::endl;
                    
                    if (current_state_ == SystemState::READY) {
                        std::cout << "🔥 Starting WARMUP phase" << std::endl;
                        set_state(SystemState::WARMUP);
                    } else {
                        std::cerr << "⚠️  Received start command but not in READY state (current: " 
                                  << static_cast<int>(current_state_.load()) << ")" << std::endl;
                    }
                }
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
    // Screen 1: Logo displayed for 3 seconds
    serial_comm_->send_state(1);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Screen 2: "Booting into mDai" - initializing camera
    serial_comm_->send_state(2);
    
    // Initialize camera in RGB-only mode for QR scanning
    configure_camera_for_state(SystemState::BOOT);
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Check WiFi
    if (check_wifi_on_boot()) {
        // WiFi already connected
        serial_comm_->send_state(3); // "WiFi Connected" screen
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // Show "mDai Ready" screen
        serial_comm_->send_state(4);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        set_state(SystemState::IDLE);
    } else {
        // No WiFi, need provisioning
        configure_camera_for_state(SystemState::AWAIT_ADMIN_QR);
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
        serial_comm_->send_state(11); // "Connecting to WiFi" screen
        
        if (network_mgr_->connect_wifi(wifi_ssid, wifi_password)) {
            previous_ssid_ = wifi_ssid;
            previous_password_ = wifi_password;
            
            set_state(SystemState::WIFI_CHANGE_SUCCESS);
            serial_comm_->send_state(12); // "WiFi Success" screen
            std::this_thread::sleep_for(std::chrono::seconds(5));
            
            set_state(SystemState::IDLE);
            return true;
        } else {
            // Failed, fallback
            set_state(SystemState::WIFI_CHANGE_FAILED);
            serial_comm_->send_state(13); // "WiFi Failed" screen
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
        return false;
    }
}

bool SystemController::handle_session_qr(const std::string& qr_data) {
    try {
        // Parse session QR from Flutter app
        // Expected format: {"platform_id":"xxx", "session_id":"xxx", "timestamp":"xxx"}
        nlohmann::json qr_json = nlohmann::json::parse(qr_data);
        
        platform_id_ = qr_json["platform_id"].get<std::string>();
        session_id_ = qr_json["session_id"].get<std::string>();
        
        std::cout << "✅ Session QR Scanned: platform=" << platform_id_ 
                  << ", session=" << session_id_ << std::endl;
        
        // Connect to WebSocket with session_id
        std::string ws_path = "/ws/device?session_id=" + session_id_;
        bool connected = network_mgr_->connect_to_middleware(
            middleware_host_, 
            443, 
            ws_path, 
            device_id_
        );
        
        if (connected) {
            std::cout << "✅ WebSocket Connected" << std::endl;
            
            // Send authentication message with session_id
            nlohmann::json auth_msg;
            auth_msg["type"] = "auth";
            auth_msg["session_id"] = session_id_;
            auth_msg["device_id"] = device_id_;
            
            network_mgr_->send_message(auth_msg.dump());
            
            // Wait a moment for auth response
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            set_state(SystemState::READY);
            return true;
        } else {
            std::cerr << "⚠ WebSocket connection failed" << std::endl;
            serial_comm_->send_error("Connection failed");
            return false;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "⚠ Session QR processing error: " << e.what() << std::endl;
        return false;
    }
}

} // namespace mdai
