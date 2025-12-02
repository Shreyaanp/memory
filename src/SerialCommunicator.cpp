#include "SerialCommunicator.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

namespace mdai {

SerialCommunicator::SerialCommunicator(const Config& config) : config_(config) {
}

SerialCommunicator::~SerialCommunicator() {
    disconnect();
}

bool SerialCommunicator::connect() {
    try {
        std::lock_guard<std::mutex> lock(write_mutex_);
        
        if (connected_) return true;

        // Open serial port
        fd_ = open(config_.port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ == -1) {
            std::cerr << "❌ Failed to open serial port " << config_.port_name << ": " << strerror(errno) << std::endl;
            return false;
        }

        if (!configure_port()) {
            close(fd_);
            fd_ = -1;
            return false;
        }

        connected_ = true;
        std::cout << "✓ Serial connected to " << config_.port_name << " at " << config_.baud_rate << " baud" << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "❌ SerialCommunicator::connect exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ SerialCommunicator::connect unknown exception" << std::endl;
        return false;
    }
}

void SerialCommunicator::disconnect() {
    try {
        std::lock_guard<std::mutex> lock(write_mutex_);
        
        if (fd_ != -1) {
            close(fd_);
            fd_ = -1;
        }
        connected_ = false;
    } catch (const std::exception& e) {
        std::cerr << "❌ SerialCommunicator::disconnect exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "❌ SerialCommunicator::disconnect unknown exception" << std::endl;
    }
}

bool SerialCommunicator::configure_port() {
    try {
        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
            return false;
        }

        // Set baud rate
        speed_t speed;
        switch (config_.baud_rate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            default: speed = B115200;
        }
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        // 8N1 mode (8 bits, no parity, 1 stop bit)
        tty.c_cflag &= ~PARENB; // No parity
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;     // 8 bits
        
        // No hardware flow control
        tty.c_cflag &= ~CRTSCTS;
        
        // Enable receiver, ignore modem control lines
        tty.c_cflag |= CREAD | CLOCAL;
        
        // Disable software flow control
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        
        // Raw input mode
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        
        // Raw output mode
        tty.c_oflag &= ~OPOST;

        // Timeouts
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 10; // 1 second read timeout

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "❌ configure_port exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ configure_port unknown exception" << std::endl;
        return false;
    }
}

bool SerialCommunicator::send_packet(SerialCommand cmd, const std::vector<uint8_t>& payload) {
    try {
        std::lock_guard<std::mutex> lock(write_mutex_);
        
        if (!connected_ || fd_ == -1) return false;

        std::vector<uint8_t> packet;
        packet.reserve(payload.size() + 5); // Start + Length + Cmd + Checksum + End

        packet.push_back(SERIAL_START_BYTE);
        packet.push_back(static_cast<uint8_t>(payload.size()));
        packet.push_back(static_cast<uint8_t>(cmd));
        
        // Add payload
        packet.insert(packet.end(), payload.begin(), payload.end());
        
        // Calculate checksum (XOR of Length + Cmd + Payload)
        uint8_t checksum = calculate_checksum(packet);
        packet.push_back(checksum);
        
        packet.push_back(SERIAL_END_BYTE);

        ssize_t written = write(fd_, packet.data(), packet.size());
        if (written != static_cast<ssize_t>(packet.size())) {
            std::cerr << "Error writing to serial port" << std::endl;
            return false;
        }
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "❌ send_packet exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ send_packet unknown exception" << std::endl;
        return false;
    }
}

uint8_t SerialCommunicator::calculate_checksum(const std::vector<uint8_t>& data) {
    // XOR checksum of everything after START_BYTE up to checksum position
    // data format so far: START | LEN | CMD | PAYLOAD... 
    // We start XORing from index 1 (LEN)
    uint8_t checksum = 0;
    for (size_t i = 1; i < data.size(); i++) {
        checksum ^= data[i];
    }
    return checksum;
}

bool SerialCommunicator::send_state(int state_id) {
    try {
        std::lock_guard<std::mutex> lock(write_mutex_);
        
        if (!connected_ || fd_ == -1) {
            // Try to reconnect if disconnected
            if (!try_reconnect()) {
                return false;
            }
        }
        
        // Send simple number format that ESP32 expects: "5\n"
        std::string message = std::to_string(state_id) + "\n";
        ssize_t written = write(fd_, message.c_str(), message.length());
        
        if (written == static_cast<ssize_t>(message.length())) {
            last_state_ = state_id;  // Store for recovery
            std::cout << "[Serial] Sent state: " << state_id << " to ESP32" << std::endl;
            return true;
        } else {
            std::cerr << "Error writing to serial port" << std::endl;
            // Mark as disconnected and try reconnect next time
            connected_ = false;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << "❌ send_state exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ send_state unknown exception" << std::endl;
        return false;
    }
}

bool SerialCommunicator::send_state_with_text(int state_id, const std::string& text) {
    try {
        std::lock_guard<std::mutex> lock(write_mutex_);
        
        if (!connected_ || fd_ == -1) {
            // Try to reconnect if disconnected
            if (!try_reconnect()) {
                return false;
            }
        }
        
        // Send JSON format: {"screen": 4, "text": "WiFi Name"}
        std::string message = "{\"screen\": " + std::to_string(state_id) + ", \"text\": \"" + text + "\"}\n";
        ssize_t written = write(fd_, message.c_str(), message.length());
        
        if (written == static_cast<ssize_t>(message.length())) {
            last_state_ = state_id;  // Store for recovery
            std::cout << "[Serial] Sent state: " << state_id << " with text: " << text << " to ESP32" << std::endl;
            return true;
        } else {
            std::cerr << "Error writing to serial port" << std::endl;
            // Mark as disconnected and try reconnect next time
            connected_ = false;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << "❌ send_state_with_text exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ send_state_with_text unknown exception" << std::endl;
        return false;
    }
}

bool SerialCommunicator::send_nose_position(float x, float y) {
    try {
        std::lock_guard<std::mutex> lock(write_mutex_);
        
        if (!connected_ || fd_ == -1) return false;
        
        // Send text format that ESP32 expects: "X:233,Y:180\n"
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "X:%d,Y:%d\n", static_cast<int>(x), static_cast<int>(y));
        ssize_t written = write(fd_, buffer, strlen(buffer));
        
        if (written != static_cast<ssize_t>(strlen(buffer))) {
            std::cerr << "Error writing to serial port" << std::endl;
            return false;
        }
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "❌ send_nose_position exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ send_nose_position unknown exception" << std::endl;
        return false;
    }
}

bool SerialCommunicator::send_progress(int progress_percent) {
    try {
        if (progress_percent < 0) progress_percent = 0;
        if (progress_percent > 100) progress_percent = 100;
        std::vector<uint8_t> payload = {static_cast<uint8_t>(progress_percent)};
        return send_packet(SerialCommand::CMD_UPDATE_PROGRESS, payload);
    } catch (const std::exception& e) {
        std::cerr << "❌ send_progress exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ send_progress unknown exception" << std::endl;
        return false;
    }
}

bool SerialCommunicator::send_batch_progress(int progress_percent) {
    try {
        // Check connection without lock first (try_reconnect has its own locking)
        if (!connected_ || fd_ == -1) {
            if (!try_reconnect()) {
                std::cerr << "[Serial] Batch progress failed: not connected" << std::endl;
                return false;
            }
        }
        
        // Now lock for the actual write
        std::lock_guard<std::mutex> lock(write_mutex_);
        
        // Re-check connection after acquiring lock
        if (!connected_ || fd_ == -1) {
            std::cerr << "[Serial] Batch progress failed: connection lost" << std::endl;
            return false;
        }
        
        // Clamp values
        if (progress_percent < 0) progress_percent = 0;
        if (progress_percent > 100) progress_percent = 100;
        
        // Send text format: "B:50\n"
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "B:%d\n", progress_percent);
        ssize_t written = write(fd_, buffer, strlen(buffer));
        
        if (written != static_cast<ssize_t>(strlen(buffer))) {
            std::cerr << "[Serial] Error writing batch progress to serial port" << std::endl;
            connected_ = false;
            return false;
        }
        
        std::cout << "[Serial] Batch Progress: " << progress_percent << "%" << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "❌ send_batch_progress exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ send_batch_progress unknown exception" << std::endl;
        return false;
    }
}

bool SerialCommunicator::send_tracking_data(int x, int y, int progress_percent, bool target_valid) {
    try {
        std::lock_guard<std::mutex> lock(write_mutex_);
        
        if (!connected_ || fd_ == -1) {
            if (!try_reconnect()) {
                return false;
            }
        }
        
        // Clamp values
        if (progress_percent < 0) progress_percent = 0;
        if (progress_percent > 100) progress_percent = 100;
        
        // Send combined format: "X:233,Y:180,P:75,C:1\n"
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "X:%d,Y:%d,P:%d,C:%d\n", 
                 x, y, progress_percent, target_valid ? 1 : 0);
        ssize_t written = write(fd_, buffer, strlen(buffer));
        
        if (written != static_cast<ssize_t>(strlen(buffer))) {
            std::cerr << "Error writing tracking data to serial port" << std::endl;
            connected_ = false;
            return false;
        }
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "❌ send_tracking_data exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ send_tracking_data unknown exception" << std::endl;
        return false;
    }
}

bool SerialCommunicator::send_error(const std::string& error_msg) {
    try {
        std::lock_guard<std::mutex> lock(write_mutex_);
        
        if (!connected_ || fd_ == -1) {
            if (!try_reconnect()) {
                return false;
            }
        }
        
        // Send JSON format that ESP32 expects: {"error": "message"}
        // Truncate message if too long
        std::string msg = error_msg;
        if (msg.length() > 200) {
            msg = msg.substr(0, 200);
        }
        
        // Escape any quotes in the message
        std::string escaped_msg;
        for (char c : msg) {
            if (c == '"') {
                escaped_msg += "\\\"";
            } else if (c == '\\') {
                escaped_msg += "\\\\";
            } else {
                escaped_msg += c;
            }
        }
        
        std::string message = "{\"error\": \"" + escaped_msg + "\"}\n";
        ssize_t written = write(fd_, message.c_str(), message.length());
        
        if (written == static_cast<ssize_t>(message.length())) {
            std::cout << "[Serial] Sent error: " << error_msg << std::endl;
            return true;
        } else {
            std::cerr << "Error writing to serial port" << std::endl;
            connected_ = false;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << "❌ send_error exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ send_error unknown exception" << std::endl;
        return false;
    }
}

bool SerialCommunicator::auto_detect_port() {
    try {
        // Try ttyACM0 first (most common for ESP32)
        std::ifstream acm0("/dev/ttyACM0");
        if (acm0.good()) {
            config_.port_name = "/dev/ttyACM0";
            std::cout << "[Serial] Auto-detected ESP32 on /dev/ttyACM0" << std::endl;
            return true;
        }
        
        // Try ttyACM1
        std::ifstream acm1("/dev/ttyACM1");
        if (acm1.good()) {
            config_.port_name = "/dev/ttyACM1";
            std::cout << "[Serial] Auto-detected ESP32 on /dev/ttyACM1" << std::endl;
            return true;
        }
        
        // Try ttyACM2 (rare but possible)
        std::ifstream acm2("/dev/ttyACM2");
        if (acm2.good()) {
            config_.port_name = "/dev/ttyACM2";
            std::cout << "[Serial] Auto-detected ESP32 on /dev/ttyACM2" << std::endl;
            return true;
        }
        
        // Try ttyUSB devices (alternative ESP32 connection method)
        for (int i = 0; i < 5; i++) {
            std::string usb_port = "/dev/ttyUSB" + std::to_string(i);
            std::ifstream usb_check(usb_port);
            if (usb_check.good()) {
                config_.port_name = usb_port;
                std::cout << "[Serial] Auto-detected ESP32 on " << usb_port << std::endl;
                return true;
            }
        }
        
        std::cerr << "[Serial] ❌ ESP32 not detected!" << std::endl;
        std::cerr << "[Serial]    Checked: /dev/ttyACM0, /dev/ttyACM1, /dev/ttyACM2, /dev/ttyUSB0-4" << std::endl;
        std::cerr << "[Serial]    Troubleshooting:" << std::endl;
        std::cerr << "[Serial]      1. Ensure ESP32 is plugged in and powered on" << std::endl;
        std::cerr << "[Serial]      2. Check USB cable (must support data, not power-only)" << std::endl;
        std::cerr << "[Serial]      3. Run: lsusb (should show ESP32/CP210x/CH340 device)" << std::endl;
        std::cerr << "[Serial]      4. Run: ls -la /dev/ttyACM* /dev/ttyUSB*" << std::endl;
        std::cerr << "[Serial]      5. Check user is in dialout group: groups | grep dialout" << std::endl;
        
        return false;
    } catch (const std::exception& e) {
        std::cerr << "❌ auto_detect_port exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ auto_detect_port unknown exception" << std::endl;
        return false;
    }
}

bool SerialCommunicator::try_reconnect() {
    try {
        // Already connected
        if (connected_ && fd_ != -1) return true;
        
        std::cout << "[Serial] Attempting to reconnect..." << std::endl;
        
        // Close old connection if any
        if (fd_ != -1) {
            close(fd_);
            fd_ = -1;
        }
        connected_ = false;
        
        // Auto-detect port
        if (!auto_detect_port()) {
            std::cerr << "[Serial] ❌ No ESP32 device found. Auto-detection failed." << std::endl;
            std::cerr << "[Serial]    The application will continue without display control." << std::endl;
            std::cerr << "[Serial]    To fix: Connect ESP32 and restart the application." << std::endl;
            return false;
        }
        
        // Try to reconnect
        if (connect()) {
            std::cout << "[Serial] ✅ Reconnected to ESP32 on " << config_.port_name << std::endl;
            
            // Restore last state if we had one
            if (last_state_ >= 0) {
                std::cout << "[Serial] Restoring last state: " << last_state_ << std::endl;
                std::string message = std::to_string(last_state_) + "\n";
                ssize_t written = write(fd_, message.c_str(), message.length());
                (void)written; // Suppress unused result warning - reconnect is best effort
            }
            
            return true;
        }
        
        return false;
    } catch (const std::exception& e) {
        std::cerr << "❌ try_reconnect exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ try_reconnect unknown exception" << std::endl;
        return false;
    }
}

// ============================================
// Async Queue Implementation
// ============================================

void SerialCommunicator::start_async() {
    try {
        if (async_running_.load()) return;
        
        async_running_.store(true);
        async_thread_ = std::thread(&SerialCommunicator::async_worker, this);
        std::cout << "📤 Serial async thread started" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "❌ start_async exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "❌ start_async unknown exception" << std::endl;
    }
}

void SerialCommunicator::stop_async() {
    try {
        if (!async_running_.load()) return;
        
        async_running_.store(false);
        queue_cv_.notify_all();
        
        if (async_thread_.joinable()) {
            async_thread_.join();
        }
        std::cout << "📤 Serial async thread stopped" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "❌ stop_async exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "❌ stop_async unknown exception" << std::endl;
    }
}

void SerialCommunicator::queue_state(int state_id, const std::string& text) {
    try {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        if (message_queue_.size() >= MAX_QUEUE_SIZE) {
            message_queue_.pop();
            queue_dropped_++;
        }
        
        message_queue_.push(SerialMsg_State{state_id, text});
        queue_cv_.notify_one();
    } catch (const std::exception& e) {
        std::cerr << "❌ queue_state exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "❌ queue_state unknown exception" << std::endl;
    }
}

void SerialCommunicator::queue_tracking(int x, int y, int progress_percent, bool target_valid) {
    try {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        // For tracking, replace existing tracking messages (only latest matters)
        // First, drain old tracking messages
        std::queue<SerialMessage> temp_queue;
        while (!message_queue_.empty()) {
            auto& msg = message_queue_.front();
            if (!std::holds_alternative<SerialMsg_Tracking>(msg)) {
                temp_queue.push(std::move(msg));
            }
            message_queue_.pop();
        }
        message_queue_ = std::move(temp_queue);
        
        if (message_queue_.size() >= MAX_QUEUE_SIZE) {
            message_queue_.pop();
            queue_dropped_++;
        }
        
        message_queue_.push(SerialMsg_Tracking{x, y, progress_percent, target_valid});
        queue_cv_.notify_one();
    } catch (const std::exception& e) {
        std::cerr << "❌ queue_tracking exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "❌ queue_tracking unknown exception" << std::endl;
    }
}

void SerialCommunicator::queue_batch_progress(int progress_percent) {
    try {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        if (message_queue_.size() >= MAX_QUEUE_SIZE) {
            message_queue_.pop();
            queue_dropped_++;
        }
        
        message_queue_.push(SerialMsg_BatchProgress{progress_percent});
        queue_cv_.notify_one();
    } catch (const std::exception& e) {
        std::cerr << "❌ queue_batch_progress exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "❌ queue_batch_progress unknown exception" << std::endl;
    }
}

void SerialCommunicator::queue_error(const std::string& error_msg) {
    try {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        if (message_queue_.size() >= MAX_QUEUE_SIZE) {
            message_queue_.pop();
            queue_dropped_++;
        }
        
        message_queue_.push(SerialMsg_Error{error_msg});
        queue_cv_.notify_one();
    } catch (const std::exception& e) {
        std::cerr << "❌ queue_error exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "❌ queue_error unknown exception" << std::endl;
    }
}

size_t SerialCommunicator::get_queue_size() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return message_queue_.size();
}

void SerialCommunicator::async_worker() {
    while (async_running_.load()) {
        try {
            SerialMessage msg;
            
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait_for(lock, std::chrono::milliseconds(50), [this] {
                    return !message_queue_.empty() || !async_running_.load();
                });
                
                if (!async_running_.load()) break;
                if (message_queue_.empty()) {
                    // Read feedback from ESP32 when idle
                    lock.unlock();
                    read_serial_feedback();
                    continue;
                }
                
                msg = std::move(message_queue_.front());
                message_queue_.pop();
            }
            
            process_message(msg);
        } catch (const std::exception& e) {
            std::cerr << "❌ async_worker exception: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "❌ async_worker unknown exception" << std::endl;
        }
    }
}

void SerialCommunicator::process_message(const SerialMessage& msg) {
    try {
        std::visit([this](auto&& arg) {
            using T = std::decay_t<decltype(arg)>;
            
            if constexpr (std::is_same_v<T, SerialMsg_State>) {
                if (arg.text.empty()) {
                    send_state(arg.state_id);
                } else {
                    send_state_with_text(arg.state_id, arg.text);
                }
            }
            else if constexpr (std::is_same_v<T, SerialMsg_Tracking>) {
                send_tracking_binary(arg.x, arg.y, arg.progress, arg.valid);
            }
            else if constexpr (std::is_same_v<T, SerialMsg_BatchProgress>) {
                send_batch_progress(arg.progress);
            }
            else if constexpr (std::is_same_v<T, SerialMsg_Error>) {
                send_error(arg.message);
            }
        }, msg);
    } catch (const std::exception& e) {
        std::cerr << "❌ process_message exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "❌ process_message unknown exception" << std::endl;
    }
}

// ============================================
// Binary Protocol Implementation
// ============================================

bool SerialCommunicator::send_tracking_binary(int x, int y, int progress_percent, bool target_valid) {
    try {
        // For now, use text format (binary can be added later for performance)
        return send_tracking_data(x, y, progress_percent, target_valid);
    } catch (const std::exception& e) {
        std::cerr << "❌ send_tracking_binary exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ send_tracking_binary unknown exception" << std::endl;
        return false;
    }
}

bool SerialCommunicator::send_binary_packet(uint8_t cmd, const uint8_t* payload, uint8_t len) {
    try {
        std::lock_guard<std::mutex> lock(write_mutex_);
        
        if (!connected_ || fd_ == -1) return false;
        
        uint8_t seq = sequence_number_++;
        
        // Build packet: START + LEN + SEQ + CMD + PAYLOAD + CHECKSUM + END
        std::vector<uint8_t> packet;
        packet.push_back(SERIAL_START_BYTE);
        packet.push_back(len);
        packet.push_back(seq);
        packet.push_back(cmd);
        for (uint8_t i = 0; i < len; i++) {
            packet.push_back(payload[i]);
        }
        
        // Calculate checksum (XOR of LEN, SEQ, CMD, PAYLOAD)
        uint8_t checksum = len ^ seq ^ cmd;
        for (uint8_t i = 0; i < len; i++) {
            checksum ^= payload[i];
        }
        packet.push_back(checksum);
        packet.push_back(SERIAL_END_BYTE);
        
        ssize_t written = write(fd_, packet.data(), packet.size());
        return written == static_cast<ssize_t>(packet.size());
    } catch (const std::exception& e) {
        std::cerr << "❌ send_binary_packet exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ send_binary_packet unknown exception" << std::endl;
        return false;
    }
}

bool SerialCommunicator::wait_for_ack(uint8_t seq, int timeout_ms) {
    try {
        std::unique_lock<std::mutex> lock(ack_mutex_);
        ack_received_ = false;
        
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        
        while (!ack_received_ && std::chrono::steady_clock::now() < deadline) {
            ack_cv_.wait_until(lock, deadline);
            if (last_acked_sequence_ == seq) {
                return true;
            }
        }
        
        return false;
    } catch (const std::exception& e) {
        std::cerr << "❌ wait_for_ack exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ wait_for_ack unknown exception" << std::endl;
        return false;
    }
}

void SerialCommunicator::read_serial_feedback() {
    try {
        if (!connected_ || fd_ == -1) return;
        
        char buffer[256];
        ssize_t bytes_read = read(fd_, buffer, sizeof(buffer) - 1);
        
        if (bytes_read <= 0) return;
        
        buffer[bytes_read] = '\0';
        serial_read_buffer_ += buffer;
        
        // Process complete lines
        size_t pos;
        while ((pos = serial_read_buffer_.find('\n')) != std::string::npos) {
            std::string line = serial_read_buffer_.substr(0, pos);
            serial_read_buffer_.erase(0, pos + 1);
            
            // Trim CR if present
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            
            if (line.empty()) continue;
            
            // Parse heartbeat: "H"
            if (line == "H") {
                last_heartbeat_ = std::chrono::steady_clock::now();
                continue;
            }
            
            // Parse status: "S:6"
            if (line.rfind("S:", 0) == 0) {
                try {
                    int screen = std::stoi(line.substr(2));
                    esp32_reported_screen_.store(screen);
                } catch (...) {}
                continue;
            }
            
            // Parse ACK: "ACK:5"
            if (line.rfind("ACK:", 0) == 0) {
                try {
                    std::lock_guard<std::mutex> lock(ack_mutex_);
                    last_acked_sequence_ = std::stoi(line.substr(4));
                    ack_received_ = true;
                    ack_cv_.notify_one();
                } catch (...) {}
                continue;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "❌ read_serial_feedback exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "❌ read_serial_feedback unknown exception" << std::endl;
    }
}

bool SerialCommunicator::is_esp32_responsive() const {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_heartbeat_).count();
    return elapsed < 10;  // Responsive if heartbeat received within 10 seconds
}

} // namespace mdai
