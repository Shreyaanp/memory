/**
 * @file SystemController.cpp
 * @brief Simplified SystemController for IR-only mode (no anti-spoofing)
 */

#include "SystemController.hpp"
#include "TrustZoneIdentity.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <nlohmann/json.hpp>
#include <quirc.h>
#include <librealsense2/rs.hpp>

namespace mdai {

SystemController::SystemController() {
    // Ring buffer: 32 initial slots, 3GB max memory
    ring_buffer_ = std::make_unique<DynamicRingBuffer>(32, 3ULL * 1024 * 1024 * 1024);
    
    SerialCommunicator::Config serial_cfg;
    serial_cfg.port_name = "/dev/ttyACM0";
    serial_comm_ = std::make_unique<SerialCommunicator>(serial_cfg);
    
    network_mgr_ = std::make_unique<NetworkManager>();
}

SystemController::~SystemController() {
    stop();
}

std::string SystemController::load_qr_shared_key() {
    std::ifstream config_file("/opt/mdai/device_config.json");
    if (config_file.is_open()) {
        nlohmann::json config;
        try {
            config_file >> config;
            if (config.contains("qr_shared_key")) {
                return config["qr_shared_key"].get<std::string>();
            }
        } catch (...) {}
        config_file.close();
    }
    
    const char* env_key = std::getenv("QR_SHARED_KEY");
    if (env_key) return std::string(env_key);
    
    return "";
}

bool SystemController::initialize() {
    if (!load_device_config()) {
        std::cerr << "⚠ Device not registered. Run deploy-complete-system.sh first!" << std::endl;
        return false;
    }
    
    if (serial_comm_) {
        if (!serial_comm_->connect()) {
            std::cerr << "⚠ Serial connection failed" << std::endl;
        }
        serial_comm_->start_async();
    } else {
        std::cerr << "⚠ Serial communicator not initialized" << std::endl;
    }
    
    face_detector_ = create_face_detector();
    if (!face_detector_) return false;

    if (network_mgr_) {
        network_mgr_->set_message_callback([this](const std::string& msg) {
            this->on_websocket_message(msg);
        });
        
        network_mgr_->set_connect_callback([this]() {
            std::lock_guard<std::mutex> lock(ws_auth_mutex_);
            if (!pending_ws_token_.empty() && network_mgr_) {
                nlohmann::json auth_msg = {
                    {"type", "auth"},
                    {"bearer_token", pending_ws_token_},
                    {"device_id", device_id_}
                };
                network_mgr_->send_message(auth_msg.dump());
            }
        });
    } else {
        std::cerr << "⚠ Network manager not initialized" << std::endl;
        return false;
    }
    
    set_state(SystemState::BOOT);
    return true;
}

void SystemController::run() {
    running_ = true;
    while (running_) {
        // Simple check - if no producer, wait (camera is initialized elsewhere)
        if (!producer_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        // If producer exists but stopped (e.g., after PROCESSING), restart it
        // Only in states that need camera for QR scanning
        if (!producer_->is_running()) {
            SystemState current = current_state_.load();
            if (current == SystemState::IDLE || current == SystemState::AWAIT_ADMIN_QR) {
                std::cout << "📹 Camera stopped - restarting for QR scanning..." << std::endl;
                
                // Stop old producer properly first
                producer_->stop();
                producer_.reset();
                
                // Small delay for camera to release resources
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                
                // Create new producer
                initialize_camera();
                
                if (producer_ && producer_->is_running()) {
                    std::cout << "✅ Camera restarted successfully" << std::endl;
                } else {
                    std::cerr << "❌ Camera restart failed" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
            }
            continue;
        }

        // WiFi connectivity check
        static int wifi_check_counter = 0;
        static int wifi_fail_count = 0;
        const int WIFI_FAIL_THRESHOLD = 3;
        
        if (++wifi_check_counter >= 100) {
            wifi_check_counter = 0;
            
            if (network_mgr_ && network_mgr_->is_connected_to_internet()) {
                if (wifi_fail_count > 0) wifi_fail_count = 0;
                
                if (current_state_ == SystemState::AWAIT_ADMIN_QR) {
                    set_state(SystemState::PROVISIONED);
                }
            } else {
                wifi_fail_count++;
                if (wifi_fail_count >= WIFI_FAIL_THRESHOLD &&
                    current_state_ != SystemState::AWAIT_ADMIN_QR && 
                    current_state_ != SystemState::BOOT &&
                    current_state_ != SystemState::ERROR) {
                    
                    session_id_.clear();
                    
                    motion_tracker_.reset();
                    if (ring_buffer_) {
                        ring_buffer_->set_recording_active(false);
                    }
                    // NOTE: Don't clear ring_buffer - causes race condition crash
                    
                    if (network_mgr_ && network_mgr_->is_connected()) {
                        network_mgr_->disconnect();
                    }
                    
                    if (serial_comm_) serial_comm_->queue_state(3);
                    set_state(SystemState::AWAIT_ADMIN_QR);
                    wifi_fail_count = 0;
                }
            }
        }

        // State timer
        if (state_timer_active_) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - state_timer_start_
            ).count();
            
            if (elapsed >= state_timer_duration_ms_) {
                state_timer_active_ = false;
                set_state(state_timer_target_);
            }
        }

        // Camera health
        static int camera_health_counter = 0;
        if (++camera_health_counter >= 400) {
            camera_health_counter = 0;
            if (camera_error_detected_.load()) {
                camera_error_detected_.store(false);
                handle_hardware_error("Camera", camera_error_message_);
            }
        }

        // ============================================
        // HEALTH CHECKS (run every ~10 seconds)
        // ============================================
        static int health_check_counter = 0;
        if (++health_check_counter >= 3000) {  // ~30fps * 10 seconds = ~300, but main loop is faster
            health_check_counter = 0;
            
            // 1. Heartbeat logging (for debugging)
            last_heartbeat_ = std::chrono::steady_clock::now();
            
            // 2. Global state watchdog - reset to IDLE if stuck too long
            SystemState current = current_state_.load();
            if (current != SystemState::IDLE && 
                current != SystemState::BOOT && 
                current != SystemState::AWAIT_ADMIN_QR) {
                
                auto state_duration = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - state_entry_time_).count();
                
                // Check for stuck state (> 60 seconds in non-idle state)
                if (state_duration > STATE_STUCK_TIMEOUT_SEC) {
                    std::cerr << "⚠️ STATE WATCHDOG: Stuck in state for " << state_duration 
                              << "s - forcing reset to IDLE" << std::endl;
                    if (serial_comm_) serial_comm_->queue_error("Session timeout");
                    set_state(SystemState::ERROR);
                }
                
                // Check for alignment timeout (> 45 seconds)
                if (current == SystemState::ALIGN && state_duration > ALIGN_TIMEOUT_SEC) {
                    std::cerr << "⚠️ ALIGN TIMEOUT: No progress for " << state_duration 
                              << "s - resetting" << std::endl;
                    if (serial_comm_) serial_comm_->queue_error("Alignment timeout");
                    set_state(SystemState::ERROR);
                }
            }
            
            // 3. Serial health check
            if (serial_comm_ && !serial_comm_->is_connected()) {
                serial_fail_count_++;
                std::cerr << "⚠️ Serial disconnected (fail count: " << serial_fail_count_ << ")" << std::endl;
                
                if (serial_fail_count_ >= SERIAL_MAX_FAILS) {
                    std::cerr << "🔄 Attempting serial reconnect..." << std::endl;
                    if (serial_comm_->try_reconnect()) {
                        std::cout << "✅ Serial reconnected" << std::endl;
                    } else {
                        std::cerr << "❌ Serial reconnect failed" << std::endl;
                    }
                    serial_fail_count_ = 0;
                }
            } else {
                serial_fail_count_ = 0;
            }
            
            // 4. Memory check - only log during active recording
            // The ring buffer self-manages memory limits; this is just for monitoring
            if (ring_buffer_ && ring_buffer_->is_recording_active()) {
                size_t mem_usage = ring_buffer_->get_memory_usage();
                size_t frame_count = ring_buffer_->get_usage();
                std::cout << "📊 Recording buffer: " << frame_count << " frames, " 
                          << mem_usage / (1024*1024) << "MB" << std::endl;
            }
        }
        // ============================================

        try {
            if (!ring_buffer_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            FrameBox* frame = ring_buffer_->get_latest_frame();
            if (frame) {
                process_frame(frame);
                ring_buffer_->release_frame(frame);
            } else {
                // Track no-frame count for stuck detection
                static int no_frame_count = 0;
                no_frame_count++;
                
                // If no frames for extended period, try camera recovery
                if (no_frame_count > 3000) {  // ~30 seconds at 10ms sleep
                    std::cerr << "⚠️ No frames for extended period - attempting camera recovery" << std::endl;
                    no_frame_count = 0;
                    
                    if (producer_ && !producer_->is_running()) {
                        handle_hardware_error("Camera", "No frames received");
                    }
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        } catch (const std::exception& e) {
            std::cerr << "❌ Exception in main loop: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } catch (...) {
            std::cerr << "❌ Unknown exception in main loop - recovering..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    std::cout << "⚠️ Main loop exited (running=" << running_.load() << ")" << std::endl;
}

void SystemController::request_restart(const std::string& reason) {
    std::cout << "🔄 Restart requested: " << (reason.empty() ? "No reason specified" : reason) << std::endl;
    restart_requested_.store(true);
    restart_reason_ = reason;
    running_ = false;  // This will cause run() to exit
}

void SystemController::stop() {
    std::cout << "🛑 SystemController::stop() called" << std::endl;
    
    // Set running flag first to stop main loop
    running_ = false;
    
    // Notify ESP32 display of shutdown (state 16 = shutdown screen)
    // Do this BEFORE stopping serial to ensure the message gets through
    if (serial_comm_) {
        std::cout << "   📺 Sending shutdown state to display..." << std::endl;
        serial_comm_->send_state_with_text(16, "Shutting down...");
        // Give serial a moment to send the message
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Stop producer first to release camera resources
    if (producer_) {
        std::cout << "   📹 Stopping camera..." << std::endl;
        producer_->stop();
    }
    
    // Disconnect network
    if (network_mgr_) {
        std::cout << "   🌐 Disconnecting network..." << std::endl;
        network_mgr_->disconnect();
    }
    
    // Stop serial async thread last
    if (serial_comm_) {
        std::cout << "   📟 Stopping serial communication..." << std::endl;
        serial_comm_->stop_async();
    }
    
    // Brief delay to allow threads to clean up
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    std::cout << "   ✅ SystemController stopped" << std::endl;
}

void SystemController::clear_session() {
    // Clear all session data - call this when returning to IDLE
    bool had_session = !session_id_.empty();
    bool was_connected = network_mgr_ && network_mgr_->is_connected();
    
    if (had_session || was_connected) {
        std::cout << "🧹 Clearing session data..." << std::endl;
    }
    
    // Clear session identifiers
    session_id_.clear();
    {
        std::lock_guard<std::mutex> lock(ws_auth_mutex_);
        pending_ws_token_.clear();
    }
    
    // Stop recording but DON'T clear the ring buffer here
    // The ring buffer is still being accessed by frame processing threads
    // Just disable recording - frames will naturally be overwritten
    if (ring_buffer_) {
        ring_buffer_->set_recording_active(false);
        // NOTE: Don't call ring_buffer_->clear() - causes crash due to race condition
        // with frame processing. Buffer will be reused naturally.
    }
    
    // Reset motion tracker
    motion_tracker_.reset();
    
    // Just signal to stop - don't call disconnect() here
    // The WebSocket thread handles its own cleanup when the connection closes
    if (network_mgr_) {
        network_mgr_->stop_reconnect();
    }
    
    // Reset result ACK flag
    result_ack_received_ = false;
    
    if (had_session || was_connected) {
        std::cout << "   ✓ Session cleared" << std::endl;
    }
}

void SystemController::set_state(SystemState new_state) {
    current_state_ = new_state;
    state_entry_time_ = std::chrono::steady_clock::now();  // Track when we entered this state
    
    switch (new_state) {
        case SystemState::BOOT:
            std::thread([this]() {
                try {
                    handle_boot();
                } catch (const std::exception& e) {
                    std::cerr << "❌ Exception in BOOT thread: " << e.what() << std::endl;
                    set_state(SystemState::ERROR);
                } catch (...) {
                    std::cerr << "❌ Unknown exception in BOOT thread" << std::endl;
                    set_state(SystemState::ERROR);
                }
            }).detach();
            break;
            
        case SystemState::AWAIT_ADMIN_QR:
            if (serial_comm_) serial_comm_->queue_state(3);
            break;
            
        case SystemState::PROVISIONING:
            if (serial_comm_) serial_comm_->queue_state(2);
            break;
            
        case SystemState::PROVISIONED: {
            std::string ssid = network_mgr_ ? network_mgr_->get_current_ssid() : "";
            if (ssid.empty()) ssid = "WiFi Connected";
            if (serial_comm_) serial_comm_->send_state_with_text(4, ssid);
            std::thread([this]() {
                try {
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    if (serial_comm_) serial_comm_->queue_state(5);
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    set_state(SystemState::IDLE);
                } catch (const std::exception& e) {
                    std::cerr << "❌ Exception in PROVISIONED thread: " << e.what() << std::endl;
                    set_state(SystemState::ERROR);
                } catch (...) {
                    std::cerr << "❌ Unknown exception in PROVISIONED thread" << std::endl;
                    set_state(SystemState::ERROR);
                }
            }).detach();
            break;
        }
            
        case SystemState::IDLE:
            std::cout << "🔄 Entering IDLE state - ready for new QR" << std::endl;
            if (serial_comm_) serial_comm_->queue_state(6);
            clear_session();  // Clear all old session data
            // Camera restart is handled in main loop to avoid race conditions
            break;
            
        case SystemState::READY:
            if (serial_comm_) serial_comm_->queue_state(7);
            // Timeout: If no start command in 30 seconds, go to ERROR
            std::thread([this]() {
                try {
                    std::this_thread::sleep_for(std::chrono::seconds(30));
                    if (current_state_ == SystemState::READY) {
                        std::cout << "⚠️ READY timeout - no start command" << std::endl;
                        if (serial_comm_) serial_comm_->queue_error("Session timeout");
                        set_state(SystemState::ERROR);
                    }
                } catch (const std::exception& e) {
                    std::cerr << "❌ Exception in READY timeout thread: " << e.what() << std::endl;
                } catch (...) {
                    std::cerr << "❌ Unknown exception in READY timeout thread" << std::endl;
                }
            }).detach();
            break;
            
        case SystemState::COUNTDOWN:
            if (serial_comm_) {
                serial_comm_->queue_state(8);
                serial_comm_->send_state_with_text(8, "...");
            }
            std::thread([this]() {
                try {
                    for (int count = 5; count >= 1; count--) {
                        if (current_state_ != SystemState::COUNTDOWN) return;
                        if (serial_comm_) serial_comm_->send_state_with_text(8, std::to_string(count));
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                    if (current_state_ == SystemState::COUNTDOWN) {
                        set_state(SystemState::ALIGN);
                    }
                } catch (const std::exception& e) {
                    std::cerr << "❌ Exception in COUNTDOWN thread: " << e.what() << std::endl;
                    set_state(SystemState::ERROR);
                } catch (...) {
                    std::cerr << "❌ Unknown exception in COUNTDOWN thread" << std::endl;
                    set_state(SystemState::ERROR);
                }
            }).detach();
            break;
            
        case SystemState::WARMUP:
            if (serial_comm_) serial_comm_->queue_state(7);
            std::thread([this]() {
                try {
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    if (current_state_ == SystemState::WARMUP) {
                        set_state(SystemState::ALIGN);
                    }
                } catch (const std::exception& e) {
                    std::cerr << "❌ Exception in WARMUP thread: " << e.what() << std::endl;
                    set_state(SystemState::ERROR);
                } catch (...) {
                    std::cerr << "❌ Unknown exception in WARMUP thread" << std::endl;
                    set_state(SystemState::ERROR);
                }
            }).detach();
            break;
            
        case SystemState::ALIGN:
            if (serial_comm_) {
                serial_comm_->queue_state(9);
                serial_comm_->queue_tracking(233, 233, 0, true);
            }
            if (ring_buffer_) {
                ring_buffer_->set_recording_active(false);
                // NOTE: Don't clear ring_buffer here - causes race condition with producer
                motion_tracker_.reset();
                ring_buffer_->set_recording_active(true);
            }
            break;
            
        case SystemState::PROCESSING:
            if (ring_buffer_) ring_buffer_->set_recording_active(false);
            // Stop producer to prevent overwriting frames during processing
            if (producer_) {
                producer_->stop();
            }
            if (serial_comm_) serial_comm_->queue_state(10);
            std::thread([this]() {
                try {
                    handle_processing();
                } catch (const std::exception& e) {
                    std::cerr << "❌ Exception in PROCESSING thread: " << e.what() << std::endl;
                    if (serial_comm_) serial_comm_->queue_error("Processing error");
                    set_state(SystemState::ERROR);
                } catch (...) {
                    std::cerr << "❌ Unknown exception in PROCESSING thread" << std::endl;
                    if (serial_comm_) serial_comm_->queue_error("Processing error");
                    set_state(SystemState::ERROR);
                }
            }).detach();
            break;
            
        case SystemState::SUCCESS:
            if (serial_comm_) serial_comm_->queue_state(11);
            // Timeout: If no delete message in 30 seconds, go to DELETE_SCREEN anyway
            std::thread([this]() {
                try {
                    std::this_thread::sleep_for(std::chrono::seconds(30));
                    if (current_state_ == SystemState::SUCCESS) {
                        std::cout << "⚠️ SUCCESS timeout (no delete from server)" << std::endl;
                        set_state(SystemState::DELETE_SCREEN);
                    }
                } catch (const std::exception& e) {
                    std::cerr << "❌ Exception in SUCCESS timeout thread: " << e.what() << std::endl;
                    set_state(SystemState::IDLE);
                } catch (...) {
                    std::cerr << "❌ Unknown exception in SUCCESS timeout thread" << std::endl;
                    set_state(SystemState::IDLE);
                }
            }).detach();
            break;
            
        case SystemState::ERROR:
            std::cout << "❌ Entering ERROR state..." << std::endl;
            if (serial_comm_) serial_comm_->queue_state(12);
            // Stop recording but DON'T fully clear session yet - that happens in IDLE
            if (ring_buffer_) {
                ring_buffer_->set_recording_active(false);
            }
            // Go to IDLE after 3 seconds (cleanup happens there)
            std::thread([this]() {
                try {
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    if (current_state_ == SystemState::ERROR) {
                        std::cout << "   ⏰ ERROR timeout → IDLE" << std::endl;
                        set_state(SystemState::IDLE);
                    }
                } catch (const std::exception& e) {
                    std::cerr << "❌ Exception in ERROR timeout thread: " << e.what() << std::endl;
                    // Force transition to IDLE to recover
                    current_state_ = SystemState::IDLE;
                } catch (...) {
                    std::cerr << "❌ Unknown exception in ERROR timeout thread" << std::endl;
                    current_state_ = SystemState::IDLE;
                }
            }).detach();
            break;
            
        case SystemState::DELETE_SCREEN: {
            std::cout << "🗑️ Entering DELETE_SCREEN state..." << std::endl;
            if (serial_comm_) serial_comm_->queue_state(15);
            
            // Save session ID for delete_ack, then start cleanup thread
            std::string saved_session = session_id_;
            
            std::thread([this, saved_session]() {
                try {
                    // Send delete_ack to server
                    if (network_mgr_ && network_mgr_->is_connected()) {
                        nlohmann::json delete_ack = {
                            {"type", "delete_ack"},
                            {"session_id", saved_session},
                            {"status", "complete"}
                        };
                        network_mgr_->send_message(delete_ack.dump());
                        std::cout << "   ✓ delete_ack sent" << std::endl;
                    }
                    
                    // Show delete screen for 3 seconds
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    
                    // Transition to IDLE (clear_session will be called there)
                    std::cout << "   🔄 DELETE_SCREEN → IDLE" << std::endl;
                    set_state(SystemState::IDLE);
                } catch (const std::exception& e) {
                    std::cerr << "❌ Exception in DELETE_SCREEN: " << e.what() << std::endl;
                    set_state(SystemState::IDLE);
                }
            }).detach();
            break;
        }
            
        default:
            break;
    }
}

void SystemController::initialize_camera() {
    CameraConfig config;
    // Standard resolution for IR camera with higher FPS
    config.ir_width = 848;
    config.ir_height = 480;
    config.ir_fps = 30;  // Higher FPS at this resolution
    config.auto_exposure = true;
    
    producer_ = std::make_unique<Producer>(config, ring_buffer_.get());
    
    producer_->set_error_callback([this](const std::string& error) {
        camera_error_detected_.store(true);
        camera_error_message_ = error;
    });
    
    producer_->set_status_callback([this](const std::string& status) {
        std::cout << "📹 Camera: " << status << std::endl;
    });
    
    producer_->start();
}

void SystemController::process_frame(FrameBox* frame) {
    SystemState state = current_state_.load();
    
    // Frame counter for IDLE state (no logging - too verbose)
    
    switch (state) {
        case SystemState::AWAIT_ADMIN_QR: handle_await_admin_qr(frame); break;
        case SystemState::IDLE: handle_idle(frame); break;
        case SystemState::READY: break;  // Don't scan QR in READY - waiting for start command
        case SystemState::WARMUP: handle_warmup(frame); break;
        case SystemState::ALIGN: handle_align(frame); break;
        default: break;
    }
}

void SystemController::handle_await_admin_qr(FrameBox* frame) {
#ifdef HAVE_OPENCV
    if (!frame) return;
    
    static int frame_skip = 0;
    if (frame_skip++ % 5 != 0) return;
    
    // Get raw IR grayscale directly for better QR detection
    cv::Mat gray = frame->get_ir_mat();
    if (gray.empty()) return;
    
    // Apply CLAHE for better contrast (critical for IR-based QR scanning)
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    cv::Mat enhanced;
    clahe->apply(gray, enhanced);
    
    cv::QRCodeDetector qr_decoder;
    std::string decoded_info = qr_decoder.detectAndDecode(enhanced);
    
    if (!decoded_info.empty()) {
        if (handle_wifi_qr(decoded_info)) {
            std::cout << "✅ WiFi Provisioning Complete" << std::endl;
        }
    }
#endif
}

void SystemController::handle_idle(FrameBox* frame) {
#ifdef HAVE_OPENCV
    if (!frame) return;
    
    try {
        // ONLY scan QR in IDLE state - NOT in READY or any other state
        SystemState current = current_state_.load();
        if (current != SystemState::IDLE) return;
        
        // Frame skip for performance (scan every 5th frame)
        static int frame_skip = 0;
        if (frame_skip++ % 5 != 0) return;
        
        // QR scan counter (no logging - too verbose)
        static int scan_count = 0;
        scan_count++;

    // Get raw IR grayscale directly for better QR detection
    cv::Mat gray = frame->get_ir_mat();
    if (gray.empty()) {
        std::cerr << "⚠️ IR frame empty!" << std::endl;
        return;
    }
    
    // Build multiple preprocessing candidates for robust QR detection
    std::vector<cv::Mat> candidates;
    
    // Method 0: CLAHE enhanced (higher clip limit for IR)
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(4.0, cv::Size(8, 8));
    cv::Mat enhanced;
    clahe->apply(gray, enhanced);
    candidates.push_back(enhanced);
    
    // Method 1: Gaussian blur + Otsu threshold (good for noisy IR)
    cv::Mat blurred, otsu;
    cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);
    cv::threshold(blurred, otsu, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    candidates.push_back(otsu);
    
    // Method 2: Adaptive threshold on CLAHE (best for phone screens)
    cv::Mat adaptive_on_clahe;
    cv::adaptiveThreshold(enhanced, adaptive_on_clahe, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, 
                          cv::THRESH_BINARY, 51, 10);
    candidates.push_back(adaptive_on_clahe);
    
    // Method 3: Sharpened + threshold
    cv::Mat sharpened;
    cv::GaussianBlur(gray, sharpened, cv::Size(0, 0), 3);
    cv::addWeighted(gray, 1.5, sharpened, -0.5, 0, sharpened);
    cv::Mat sharp_thresh;
    cv::threshold(sharpened, sharp_thresh, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    candidates.push_back(sharp_thresh);
    
    // Method 4: Original grayscale (fallback)
    candidates.push_back(gray);
    
    std::string decoded_info;
    static int ecc_fail_count = 0;  // Track ECC failures to reduce log spam
    
    for (size_t i = 0; i < candidates.size() && decoded_info.empty(); i++) {
        cv::Mat& img = candidates[i];
        
        struct quirc *qr = quirc_new();
        if (!qr) continue;
        
        if (quirc_resize(qr, img.cols, img.rows) < 0) {
            quirc_destroy(qr);
            continue;
        }
        
        uint8_t *image = quirc_begin(qr, nullptr, nullptr);
        if (!image) {
            quirc_destroy(qr);
            continue;
        }
        memcpy(image, img.data, img.cols * img.rows);
        quirc_end(qr);
        
        int num_codes = quirc_count(qr);
        
        // Log scan progress periodically (every 500 scans, reduced verbosity)
        static int total_scans = 0;
        if (++total_scans % 500 == 0) {
            std::cout << "📷 QR scans: " << total_scans << ", ECC fails: " << ecc_fail_count << std::endl;
        }
        
        if (num_codes > 0) {
            struct quirc_code code;
            struct quirc_data data;
            
            quirc_extract(qr, 0, &code);
            quirc_decode_error_t err = quirc_decode(&code, &data);
            
            if (err == QUIRC_SUCCESS) {
                decoded_info = std::string(reinterpret_cast<char*>(data.payload), data.payload_len);
                std::cout << "📱 QR DETECTED! Method " << i << ", Length: " << decoded_info.length() << " bytes" << std::endl;
                ecc_fail_count = 0;  // Reset on success
            } else {
                ecc_fail_count++;
                // Only log ECC failures every 50 occurrences to reduce spam
                if (ecc_fail_count % 50 == 1) {
                    std::cout << "⚠️ QR visible but ECC errors (count: " << ecc_fail_count << ") - hold phone steady" << std::endl;
                }
            }
        }
        
        quirc_destroy(qr);
    }
    
    if (!decoded_info.empty()) {
        std::cout << "🔍 QR content preview: " << decoded_info.substr(0, std::min(size_t(50), decoded_info.length())) << "..." << std::endl;
        
        // Minimum length check
        if (decoded_info.length() < 20) {
            std::cout << "⚠️ QR too short (" << decoded_info.length() << " chars), ignoring" << std::endl;
            return;
        }
        
        bool is_session_qr = false;
        std::string encrypted_data;
        
        if (decoded_info.length() > 2 && decoded_info.substr(0, 2) == "S:") {
            is_session_qr = true;
            encrypted_data = decoded_info.substr(2);
            std::cout << "📦 Detected S: prefix format" << std::endl;
        } else {
            try {
                nlohmann::json qr_json = nlohmann::json::parse(decoded_info);
                if (qr_json.contains("qr_encrypted")) {
                    is_session_qr = true;
                    encrypted_data = qr_json["qr_encrypted"].get<std::string>();
                    std::cout << "📦 Detected JSON format with qr_encrypted field" << std::endl;
                }
            } catch (...) {
                if (decoded_info.length() < 100 && decoded_info.length() > 20) {
                    is_session_qr = true;
                    encrypted_data = decoded_info;
                    std::cout << "📦 Treating as raw encrypted data" << std::endl;
                }
            }
        }
        
        if (is_session_qr) {
            std::cout << "🔐 Attempting decryption..." << std::endl;
            
            std::string qr_key = load_qr_shared_key();
            if (qr_key.empty()) {
                std::cerr << "❌ QR decryption key not configured!" << std::endl;
                if (serial_comm_) serial_comm_->queue_error("Device not configured");
                set_state(SystemState::ERROR);
                return;
            }
            
            try {
                std::string decrypted_data = CryptoUtils::aes256_decrypt(encrypted_data, qr_key);
                std::cout << "✅ Decryption successful! Token length: " << decrypted_data.length() << std::endl;
                
                std::string ws_token;
                if (decrypted_data.length() > 2 && decrypted_data.substr(0, 2) == "S:") {
                    ws_token = decrypted_data.substr(2);
                } else {
                    ws_token = decrypted_data;
                }
                
                if (ws_token.empty() || ws_token.length() != 8) {
                    std::cout << "⚠️ Invalid token length: " << ws_token.length() << " (expected 8)" << std::endl;
                    return;
                }
                
                std::cout << "🔗 Connecting to middleware with token..." << std::endl;
                
                {
                    std::lock_guard<std::mutex> lock(ws_auth_mutex_);
                    pending_ws_token_ = ws_token;
                }
                
                if (!network_mgr_) {
                    std::cerr << "❌ Network manager not available" << std::endl;
                    if (serial_comm_) serial_comm_->queue_error("Internal error");
                    set_state(SystemState::ERROR);
                    return;
                }
                
                std::string ws_path = "/ws/device";
                network_mgr_->connect_to_middleware(middleware_host_, 443, ws_path, device_id_);
                
                int wait_count = 0;
                while (network_mgr_ && !network_mgr_->is_connected() && wait_count < 100) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    wait_count++;
                }
                
                if (!network_mgr_ || !network_mgr_->is_connected()) {
                    std::cerr << "❌ WebSocket connection failed after " << wait_count * 100 << "ms" << std::endl;
                    if (serial_comm_) {
                        serial_comm_->queue_error("Connection failed");
                        serial_comm_->queue_state(12);
                    }
                    if (network_mgr_) network_mgr_->disconnect();
                    state_timer_start_ = std::chrono::steady_clock::now();
                    state_timer_target_ = SystemState::IDLE;
                    state_timer_duration_ms_ = 3000;
                    state_timer_active_ = true;
                } else {
                    std::cout << "✅ WebSocket connected!" << std::endl;
                }
                
            } catch (const std::exception& e) {
                std::cerr << "❌ Decryption failed: " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "❌ Decryption failed (unknown error)" << std::endl;
            }
        } else {
            std::cout << "⚠️ QR not recognized as session QR" << std::endl;
        }
    }
    } catch (const std::exception& e) {
        std::cerr << "❌ Exception in handle_idle: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "❌ Unknown exception in handle_idle" << std::endl;
    }
#endif
}

void SystemController::handle_warmup(FrameBox* /* frame */) {
    static int warmup_frames = 0;
    if (warmup_frames++ > 60) {
        if (warmup_frames == 62 && network_mgr_) {
            network_mgr_->send_message("{\"type\":\"status\", \"payload\":\"warmed_up\"}");
        }
    }
}

void SystemController::handle_align(FrameBox* frame) {
    if (!frame) {
        motion_tracker_.no_face_counter++;
        if (motion_tracker_.no_face_counter > 150) {
            if (serial_comm_) serial_comm_->queue_error("Camera disconnected");
            set_state(SystemState::ERROR);
        }
        return;
    }
    
    static int startup_frames = 0;
    static const int STARTUP_GRACE_PERIOD = 450;
    static const int FACE_TIMEOUT_FRAMES = 150;
    
    startup_frames++;
    bool in_grace_period = (startup_frames < STARTUP_GRACE_PERIOD);
    
    if (!face_detector_ || !face_detector_->detect(frame)) {
        motion_tracker_.no_face_counter++;
        motion_tracker_.validation_state = FaceValidationState::NO_FACE;
        
        if (serial_comm_) serial_comm_->queue_tracking(233, 233, 
            static_cast<int>(motion_tracker_.progress * 100), false);
        
        if (!in_grace_period && motion_tracker_.no_face_counter > FACE_TIMEOUT_FRAMES) {
            handle_face_timeout();
            startup_frames = 0;
        }
        return;
    }
    
    auto& landmarks = frame->metadata.landmarks;
    if (landmarks.size() < 468) {
        motion_tracker_.no_face_counter++;
        motion_tracker_.validation_state = FaceValidationState::NO_FACE;
        if (serial_comm_) serial_comm_->queue_tracking(233, 233, 
            static_cast<int>(motion_tracker_.progress * 100), false);
        return;
    }
    
    // Relaxed validation - just check if face is detected, no extreme yaw check
    bool is_valid = frame->metadata.face_detected && landmarks.size() >= 468;
    
    if (frame->metadata.face_detected) {
        motion_tracker_.no_face_counter = 0;
    } else {
        motion_tracker_.no_face_counter++;
    }
    
    float nose_x_rot = landmarks[4].x;
    float nose_y_rot = landmarks[4].y;
    
    float nose_x_norm = nose_x_rot / frame->metadata.frame_width;
    float nose_y_norm = nose_y_rot / frame->metadata.frame_height;
    
    // Camera is rotated 90° - swap X and Y axes
    // Camera Y → Screen X, Camera X → Screen Y
    float raw_x = nose_y_norm * 465.0f;
    float raw_y = nose_x_norm * 465.0f;
    // No mirroring needed
    
    constexpr float BASE_SMOOTHING = 0.15f;
    constexpr float DEAD_ZONE = 3.0f;
    constexpr float VELOCITY_THRESHOLD = 20.0f;
    
    if (!motion_tracker_.smoothing_initialized) {
        motion_tracker_.smoothed_x = raw_x;
        motion_tracker_.smoothed_y = raw_y;
        motion_tracker_.smoothing_initialized = true;
    } else {
        float dx = raw_x - motion_tracker_.smoothed_x;
        float dy = raw_y - motion_tracker_.smoothed_y;
        float distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance > DEAD_ZONE) {
            float velocity_factor = std::min(distance / VELOCITY_THRESHOLD, 1.0f);
            float adaptive_smoothing = BASE_SMOOTHING * (1.0f - velocity_factor * 0.8f);
            
            motion_tracker_.smoothed_x = adaptive_smoothing * motion_tracker_.smoothed_x 
                                       + (1.0f - adaptive_smoothing) * raw_x;
            motion_tracker_.smoothed_y = adaptive_smoothing * motion_tracker_.smoothed_y 
                                       + (1.0f - adaptive_smoothing) * raw_y;
        }
    }
    
    int screen_x = std::max(0, std::min(465, static_cast<int>(motion_tracker_.smoothed_x)));
    int screen_y = std::max(0, std::min(465, static_cast<int>(motion_tracker_.smoothed_y)));
    
    float progress = motion_tracker_.progress;
    if (is_valid) {
        progress = calculate_circular_motion_progress(cv::Point2f(nose_x_norm, nose_y_norm));
    }
    
    if (serial_comm_) serial_comm_->queue_tracking(screen_x, screen_y, 
        static_cast<int>(progress * 100), is_valid);
    
    // Progress messages to mobile removed - not needed
    
    if (progress >= 1.0f) {
        if (serial_comm_) serial_comm_->queue_tracking(screen_x, screen_y, 100, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        set_state(SystemState::PROCESSING);
        return;
    }
}

void SystemController::handle_face_timeout() {
    nlohmann::json error_msg = {
        {"type", "error"},
        {"code", "timeout_no_face"},
        {"message", "No face detected"}
    };
    if (network_mgr_) network_mgr_->send_message(error_msg.dump());
    
    if (ring_buffer_) ring_buffer_->set_recording_active(false);
    // NOTE: Don't clear ring_buffer - causes race condition
    
    if (serial_comm_) serial_comm_->queue_error("Face not detected");
    set_state(SystemState::ERROR);
}

SystemController::FaceValidationState SystemController::validate_face_position(FrameBox* frame) {
    if (!frame || !frame->metadata.face_detected) {
        return FaceValidationState::NO_FACE;
    }
    
    if (frame->metadata.landmarks.size() < 468) {
        return FaceValidationState::NO_FACE;
    }
    
    // Check orientation only (no depth-based distance in IR-only mode)
    if (is_face_orientation_extreme(frame->metadata.landmarks)) {
        return FaceValidationState::EXTREME_ROTATION;
    }
    
    return FaceValidationState::VALID;
}

bool SystemController::is_face_orientation_extreme(const std::vector<FrameBoxMetadata::Landmark>& landmarks) {
    if (landmarks.size() < 264) return true;
    
    const auto& nose = landmarks[4];
    const auto& left_eye = landmarks[33];
    const auto& right_eye = landmarks[263];
    
    float eye_mid_y = (left_eye.y + right_eye.y) / 2.0f;
    float eye_width = std::abs(right_eye.y - left_eye.y);
    
    if (eye_width < 20.0f) return true;
    
    float nose_offset = std::abs(nose.y - eye_mid_y) / eye_width;
    return nose_offset > EXTREME_YAW_THRESHOLD;
}

float SystemController::calculate_circular_motion_progress(const cv::Point2f& nose) {
    motion_tracker_.smoothed_nose_x = 
        NOSE_SMOOTHING_FACTOR * motion_tracker_.smoothed_nose_x + 
        (1.0f - NOSE_SMOOTHING_FACTOR) * nose.x;
    motion_tracker_.smoothed_nose_y = 
        NOSE_SMOOTHING_FACTOR * motion_tracker_.smoothed_nose_y + 
        (1.0f - NOSE_SMOOTHING_FACTOR) * nose.y;
    
    float smooth_x = motion_tracker_.smoothed_nose_x;
    float smooth_y = motion_tracker_.smoothed_nose_y;
    
    float dx = smooth_x - SPIRAL_CENTER_X;
    float dy = smooth_y - SPIRAL_CENTER_Y;
    float current_angle = std::atan2(dy, dx);
    
    float ideal_x = SPIRAL_CENTER_X + SPIRAL_RADIUS * std::cos(current_angle);
    float ideal_y = SPIRAL_CENTER_Y + SPIRAL_RADIUS * std::sin(current_angle);
    
    float display_norm_x = (1.0f - MAGNETIC_STRENGTH) * smooth_x + MAGNETIC_STRENGTH * ideal_x;
    float display_norm_y = (1.0f - MAGNETIC_STRENGTH) * smooth_y + MAGNETIC_STRENGTH * ideal_y;
    
    motion_tracker_.display_x = display_norm_x * 465.0f;
    motion_tracker_.display_y = display_norm_y * 465.0f;
    motion_tracker_.smoothed_x = motion_tracker_.display_x;
    motion_tracker_.smoothed_y = motion_tracker_.display_y;
    
    motion_tracker_.nose_positions.push_back(nose);
    if (motion_tracker_.nose_positions.size() > 60) {
        motion_tracker_.nose_positions.erase(motion_tracker_.nose_positions.begin());
    }
    
    if (!motion_tracker_.angle_initialized) {
        motion_tracker_.last_angle = current_angle;
        motion_tracker_.angle_initialized = true;
        return motion_tracker_.progress;
    }
    
    float delta = current_angle - motion_tracker_.last_angle;
    while (delta > 3.14159f) delta -= 2.0f * 3.14159f;
    while (delta < -3.14159f) delta += 2.0f * 3.14159f;
    
    float velocity = std::abs(delta);
    if (velocity > MOTION_PAUSE_THRESHOLD && velocity < 0.8f) {
        float boosted_delta = velocity * PROGRESS_BOOST;
        motion_tracker_.total_rotation += boosted_delta;
        motion_tracker_.last_angle = current_angle;
    }
    
    float new_progress = motion_tracker_.total_rotation / SPIRAL_COMPLETE_ANGLE;
    if (new_progress > 1.0f) new_progress = 1.0f;
    
    if (new_progress > motion_tracker_.progress) {
        motion_tracker_.progress = new_progress;
    }
    
    return motion_tracker_.progress;
}

void SystemController::handle_processing() {
    std::cout << "🔬 Processing IR frames..." << std::endl;
    
    std::string current_session_id = session_id_;
    std::vector<FrameBox*> recording;  // Declare at top for cleanup in catch
    
    // Helper lambda to release all acquired frames (moved to top for exception safety)
    auto release_all_frames = [this, &recording]() {
        if (!ring_buffer_) return;
        for (FrameBox* frame : recording) {
            if (frame) ring_buffer_->release_frame(frame);
        }
        recording.clear();
    };
    
    try {
        if (serial_comm_) serial_comm_->queue_batch_progress(10);
        
        if (!ring_buffer_) {
            std::cerr << "❌ Ring buffer not available" << std::endl;
            if (serial_comm_) serial_comm_->queue_error("Internal error");
            set_state(SystemState::ERROR);
            return;
        }
        
        recording = ring_buffer_->get_all_valid_frames();
        std::cout << "📊 Retrieved " << recording.size() << " frames" << std::endl;
        
        if (recording.empty()) {
            if (serial_comm_) serial_comm_->queue_error("No recording data");
            if (network_mgr_) network_mgr_->send_message("{\"type\":\"error\", \"code\":\"no_recording_data\"}");
            set_state(SystemState::ERROR);
            return;
        }
        
        if (serial_comm_) serial_comm_->queue_batch_progress(30);
        
        // Select best frame based on face detection confidence
        FrameBox* best_frame = nullptr;
        float best_confidence = -1.0f;
        
        for (FrameBox* frame : recording) {
            if (frame && frame->metadata.face_detected && 
                frame->metadata.landmarks.size() >= 468 &&
                frame->metadata.face_detection_confidence > best_confidence) {
                best_confidence = frame->metadata.face_detection_confidence;
                best_frame = frame;
            }
        }
        
        if (!best_frame && !recording.empty()) {
            best_frame = recording.back();
        }
        
        if (!best_frame) {
            if (serial_comm_) serial_comm_->queue_error("No valid frames");
            release_all_frames();
            set_state(SystemState::ERROR);
            return;
        }
        
        if (serial_comm_) serial_comm_->queue_batch_progress(50);
        
        // Convert IR to BGR for image encoding
        cv::Mat ir_bgr = best_frame->get_ir_as_bgr();
        if (ir_bgr.empty()) {
            if (serial_comm_) serial_comm_->queue_error("Camera error");
            release_all_frames();
            set_state(SystemState::ERROR);
            return;
        }
        
        // Rotate image 90° counter-clockwise (-90°) to correct camera orientation
        cv::Mat rotated;
        cv::rotate(ir_bgr, rotated, cv::ROTATE_90_CLOCKWISE);
        
        std::vector<uchar> jpeg_buffer;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};
        cv::imencode(".jpg", rotated, jpeg_buffer, compression_params);
        
        if (serial_comm_) serial_comm_->queue_batch_progress(70);
        
        std::string save_path = "/home/mercleDev/mdai_logs/captured_face_" + current_session_id + ".jpg";
        cv::imwrite(save_path, rotated);
        std::cout << "📸 IR image saved: " << save_path << " (rotated for portrait)" << std::endl;
        
        // Release all acquired frames - we've extracted the image data we need
        release_all_frames();
        
        if (serial_comm_) serial_comm_->queue_batch_progress(75);
        
        // Upload image
        std::vector<uint8_t> buffer_uint8(jpeg_buffer.begin(), jpeg_buffer.end());
        if (!network_mgr_) {
            if (serial_comm_) serial_comm_->queue_error("Network not available");
            set_state(SystemState::ERROR);
            return;
        }
        std::string image_id = network_mgr_->upload_image(middleware_host_, buffer_uint8, 
                                                           current_session_id, device_id_);
        
        if (image_id.empty()) {
            if (serial_comm_) serial_comm_->queue_error("Image upload failed");
            set_state(SystemState::ERROR);
            return;
        }
        
        if (serial_comm_) serial_comm_->queue_batch_progress(85);
        
        // Send result (platform_id handled server-side, not needed here)
        nlohmann::json api_payload;
        api_payload["type"] = "submit_result";
        api_payload["status"] = "success";
        api_payload["session_id"] = current_session_id;
        api_payload["image_id"] = image_id;
        api_payload["mode"] = "ir_only";
        api_payload["timestamp"] = std::chrono::system_clock::now().time_since_epoch().count();
        
        result_ack_received_ = false;
        
        if (!network_mgr_ || !network_mgr_->send_message(api_payload.dump())) {
            if (serial_comm_) serial_comm_->queue_error("Failed to send result");
            set_state(SystemState::ERROR);
            return;
        }
        
        // Wait for ACK
        auto send_time = std::chrono::steady_clock::now();
        while (!result_ack_received_) {
            if (!network_mgr_ || !network_mgr_->is_connected()) {
                if (serial_comm_) serial_comm_->queue_error("Connection lost");
                set_state(SystemState::ERROR);
                return;
            }
            
            if (current_state_ != SystemState::PROCESSING) return;
            
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - send_time
            ).count();
            
            if (elapsed >= 10000) {
                if (serial_comm_) serial_comm_->queue_error("Server timeout");
                set_state(SystemState::ERROR);
                return;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        if (serial_comm_) serial_comm_->queue_batch_progress(100);
        
        if (current_state_ == SystemState::PROCESSING) {
            set_state(SystemState::SUCCESS);
        }
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Exception in handle_processing(): " << e.what() << std::endl;
        release_all_frames();
        if (serial_comm_) serial_comm_->queue_error("Processing failed");
        set_state(SystemState::ERROR);
    } catch (...) {
        std::cerr << "❌ Unknown exception in handle_processing()" << std::endl;
        release_all_frames();
        if (serial_comm_) serial_comm_->queue_error("Processing failed");
        set_state(SystemState::ERROR);
    }
}

void SystemController::on_websocket_message(const std::string& message) {
    if (message.empty()) {
        if (serial_comm_) serial_comm_->queue_error("Connection lost");
        set_state(SystemState::ERROR);
        return;
    }
    
    try {
        nlohmann::json msg_json = nlohmann::json::parse(message);
        std::string msg_type = msg_json.value("type", "");
        
        if (msg_type == "connection_failed") {
            if (serial_comm_) serial_comm_->queue_error("Connection failed");
            set_state(SystemState::ERROR);
            return;
        }
        
        if (msg_type == "disconnected") {
            // WebSocket disconnected - if in active session, go to ERROR
            if (current_state_ != SystemState::IDLE && 
                current_state_ != SystemState::DELETE_SCREEN &&
                current_state_ != SystemState::ERROR) {
                if (serial_comm_) serial_comm_->queue_error("Connection lost");
                set_state(SystemState::ERROR);
            }
            return;
        }
        
        if (msg_type == "auth_success") {
            if (msg_json.contains("session_id")) {
                session_id_ = msg_json["session_id"].get<std::string>();
            }
            set_state(SystemState::READY);
        }
        else if (msg_type == "auth_failed") {
            if (serial_comm_) serial_comm_->queue_error("Auth failed");
            set_state(SystemState::ERROR);
        }
        else if (msg_type == "mobile_disconnected") {
            // Ignore if in DELETE_SCREEN (disconnect is expected after delete_ack)
            if (current_state_ != SystemState::DELETE_SCREEN && 
                current_state_ != SystemState::IDLE &&
                current_state_ != SystemState::ERROR) {
                if (serial_comm_) serial_comm_->queue_error("Mobile disconnected");
                set_state(SystemState::ERROR);
            }
        }
        else if (msg_type == "to_device") {
            if (msg_json.contains("data")) {
                nlohmann::json data = msg_json["data"];
                if (data.contains("command")) {
                    std::string command = data.value("command", "");
                    if ((command == "start_verification" || command == "start") &&
                        current_state_ == SystemState::READY) {
                        set_state(SystemState::COUNTDOWN);
                    }
                }
            }
        }
        else if (msg_json.contains("command")) {
            std::string command = msg_json.value("command", "");
            if ((command == "start_verification" || command == "start") &&
                current_state_ == SystemState::READY) {
                set_state(SystemState::COUNTDOWN);
            }
        }
        else if (msg_type == "verification_result" || msg_type == "api_response" || msg_type == "image_received") {
            result_ack_received_ = true;
            std::string status = msg_json.value("status", "ok");
            
            if (status == "success" || status == "ok") {
                std::cout << "✅ Verification SUCCESS from server!" << std::endl;
                // SUCCESS state transition happens in handle_processing() after result_ack_received_ is set
            }
            else if (status == "failed" || status == "error") {
                std::string reason = msg_json.value("reason", "unknown");
                std::string message = msg_json.value("message", "");
                
                std::cout << "❌ Verification FAILED: " << reason << " - " << message << std::endl;
                
                // Show user-friendly error message
                if (serial_comm_) {
                    if (reason == "spoof_detected") {
                        std::cout << "🚨 SPOOF DETECTED by server!" << std::endl;
                        serial_comm_->queue_error("Spoof detected");
                    } else if (!message.empty()) {
                        serial_comm_->queue_error(message);
                    } else {
                        serial_comm_->queue_error(reason);
                    }
                }
                set_state(SystemState::ERROR);
            }
        }
        else if (msg_type == "delete") {
            std::cout << "📩 Received DELETE message from server" << std::endl;
            if (current_state_ == SystemState::SUCCESS ||
                (current_state_ != SystemState::DELETE_SCREEN && current_state_ != SystemState::IDLE)) {
                std::cout << "   → Transitioning to DELETE_SCREEN" << std::endl;
                set_state(SystemState::DELETE_SCREEN);
            } else {
                std::cout << "   → Ignored (current state doesn't allow)" << std::endl;
            }
        }
        else if (msg_type == "session_closed") {
            // Server closed the session (timeout, etc.)
            std::string reason = msg_json.value("reason", "Session closed");
            std::cout << "⚠️ Session closed by server: " << reason << std::endl;
            if (current_state_ != SystemState::IDLE && current_state_ != SystemState::ERROR) {
                if (serial_comm_) serial_comm_->queue_error("Session expired");
                set_state(SystemState::ERROR);
            }
        }
        else if (msg_type == "error") {
            // Generic error from server
            std::string error = msg_json.value("error", "Unknown error");
            std::cout << "❌ Server error: " << error << std::endl;
            if (current_state_ != SystemState::IDLE && current_state_ != SystemState::ERROR) {
                if (serial_comm_) serial_comm_->queue_error(error);
                set_state(SystemState::ERROR);
            }
        }
        else if (msg_type == "ping") {
            // Server ping - respond with pong
            if (network_mgr_ && network_mgr_->is_connected()) {
                nlohmann::json pong = {{"type", "pong"}};
                network_mgr_->send_message(pong.dump());
            }
        }
        else if (msg_type == "pong" || msg_type == "ack") {
            // Server acknowledgments - ignore silently
        }
        
    } catch (...) {}
}

bool SystemController::load_device_config() {
    if (!TrustZoneIdentity::initialize()) {
        return false;
    }
    
    device_id_ = TrustZoneIdentity::get_device_id();
    hardware_id_ = TrustZoneIdentity::get_hardware_serial();
    
    std::cout << "✅ Device ID: " << device_id_ << std::endl;
    
    std::string config_path = "/etc/mdai/device_config.json";
    std::ifstream config_file(config_path);
    
    if (!config_file.is_open()) {
        nlohmann::json config;
        config["device_id"] = device_id_;
        config["hardware_id"] = hardware_id_;
        
        std::ofstream out_file(config_path);
        out_file << config.dump(2);
        out_file.close();
        return true;
    }
    
    try {
        nlohmann::json config;
        config_file >> config;
        std::string stored_device_id = config["device_id"].get<std::string>();
        return stored_device_id == device_id_;
    } catch (...) {
        return false;
    }
}

bool SystemController::check_wifi_on_boot() {
    return network_mgr_ && network_mgr_->is_connected_to_internet();
}

void SystemController::handle_boot() {
    std::cout << "🚀 handle_boot() started" << std::endl;
    
    try {
        if (serial_comm_) serial_comm_->queue_state(1);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        if (serial_comm_) serial_comm_->queue_state(2);
        
        std::cout << "🔧 Initializing IR camera..." << std::endl;
        
        const int MAX_RETRIES = 3;
        bool camera_ok = false;
        
        for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
            try {
                if (producer_ && !producer_->is_running()) {
                    producer_.reset();
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }
                
                initialize_camera();
                
                if (producer_ && producer_->is_running()) {
                    camera_ok = true;
                    std::cout << "✅ IR camera started (attempt " << attempt << ")" << std::endl;
                    break;
                }
            } catch (const std::exception& e) {
                std::cerr << "⚠️ Camera init attempt " << attempt << " failed: " << e.what() << std::endl;
            }
            
            if (attempt < MAX_RETRIES) {
                std::this_thread::sleep_for(std::chrono::seconds(attempt * 3));
            }
        }
        
        if (!camera_ok) {
            std::cerr << "❌ Camera failed after " << MAX_RETRIES << " attempts" << std::endl;
            if (serial_comm_) {
                serial_comm_->queue_error("Camera hardware error");
                serial_comm_->queue_state(12);
            }
            set_state(SystemState::ERROR);
            return;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        if (check_wifi_on_boot()) {
            set_state(SystemState::PROVISIONED);
        } else {
            set_state(SystemState::AWAIT_ADMIN_QR);
        }
        
        std::cout << "✅ handle_boot() completed successfully" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ FATAL in handle_boot(): " << e.what() << std::endl;
        if (serial_comm_) serial_comm_->queue_error("Boot failed");
        set_state(SystemState::ERROR);
    } catch (...) {
        std::cerr << "❌ FATAL unknown exception in handle_boot()" << std::endl;
        if (serial_comm_) serial_comm_->queue_error("Boot failed");
        set_state(SystemState::ERROR);
    }
}

bool SystemController::handle_wifi_qr(const std::string& qr_data) {
    try {
        auto [success, bson_id, decrypted_challenge, error_msg] = 
            CryptoUtils::validate_and_decrypt_qr(qr_data, TrustZoneIdentity::export_private_key_for_backup());
        
        if (!success) return false;
        
        std::string server_url = "https://" + middleware_host_;
        auto [server_success, wifi_ssid, wifi_password, session_type] = 
            CryptoUtils::validate_challenge_with_server(
                server_url, bson_id, decrypted_challenge);
        
        if (!server_success) return false;
        
        set_state(SystemState::WIFI_CHANGE_CONNECTING);
        if (serial_comm_) serial_comm_->queue_state(14);
        
        if (network_mgr_ && network_mgr_->connect_wifi(wifi_ssid, wifi_password)) {
            previous_ssid_ = wifi_ssid;
            previous_password_ = wifi_password;
            set_state(SystemState::WIFI_CHANGE_SUCCESS);
            if (serial_comm_) serial_comm_->queue_state(4);
            std::this_thread::sleep_for(std::chrono::seconds(3));
            set_state(SystemState::IDLE);
            return true;
        } else {
            set_state(SystemState::WIFI_CHANGE_FAILED);
            if (serial_comm_) serial_comm_->queue_state(13);
            if (!previous_ssid_.empty() && network_mgr_) {
                network_mgr_->connect_wifi(previous_ssid_, previous_password_);
            }
            std::this_thread::sleep_for(std::chrono::seconds(5));
            set_state(SystemState::AWAIT_ADMIN_QR);
            return false;
        }
        
    } catch (...) {
        if (serial_comm_) serial_comm_->queue_state(12);
        return false;
    }
}

bool SystemController::check_camera_health() {
    if (!producer_) return false;
    return producer_->is_camera_healthy(3000);
}

void SystemController::handle_hardware_error(const std::string& component, const std::string& error_msg) {
    std::cerr << "❌ Hardware FAILURE [" << component << "]: " << error_msg << std::endl;
    
    // Track failure count and time
    if (hardware_failure_count_ == 0) {
        first_failure_time_ = std::chrono::steady_clock::now();
    }
    hardware_failure_count_++;
    
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - first_failure_time_).count();
    
    // Stage 1: After MAX_RETRIES failures, restart the service
    if (hardware_failure_count_ == MAX_HARDWARE_RETRIES) {
        std::cerr << "🔄 Max retries reached - RESTARTING SERVICE..." << std::endl;
        if (serial_comm_) {
            serial_comm_->queue_error("Restarting service...");
            serial_comm_->queue_state(12);
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Kill this process - systemd will restart it
        running_ = false;
        std::exit(1);  // Exit with error code, systemd will restart
        return;
    }
    
    // Stage 2: If still failing after timeout, reboot device
    if (elapsed > HARDWARE_FAILURE_REBOOT_TIMEOUT_SEC && hardware_failure_count_ > MAX_HARDWARE_RETRIES) {
        std::cerr << "🔄 Hardware failure persists for " << elapsed << "s - REBOOTING DEVICE..." << std::endl;
        if (serial_comm_) {
            serial_comm_->queue_error("Hardware failure - rebooting");
            serial_comm_->queue_state(12);
        }
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // Reboot the device
        (void)std::system("sudo reboot");
        return;
    }
    
    // Try to recover the component
    if (component == "Camera") {
        if (serial_comm_) serial_comm_->queue_error("Camera error - reconnecting...");
        
        // Stop and reset camera
        if (producer_) {
            try {
                producer_->stop();
            } catch (...) {}
            producer_.reset();
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        
        // Try to reinitialize
        try {
            initialize_camera();
            
            if (producer_ && producer_->is_running()) {
                std::cout << "✅ Camera recovered (attempt " << hardware_failure_count_ << ")" << std::endl;
                hardware_failure_count_ = 0;  // Reset on success
            } else {
                std::cerr << "⚠️ Camera recovery failed (attempt " << hardware_failure_count_ << ")" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "❌ Camera init exception: " << e.what() << std::endl;
        }
    }
}

} // namespace mdai

