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
    // Initialize heartbeat time to now (will timeout quickly if ESP32 doesn't respond)
    last_heartbeat_time_ = std::chrono::steady_clock::now();
    memset(read_buffer_, 0, sizeof(read_buffer_));
}

SerialCommunicator::~SerialCommunicator() {
    disconnect();
}

bool SerialCommunicator::connect() {
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
}

void SerialCommunicator::disconnect() {
    std::lock_guard<std::mutex> lock(write_mutex_);
    
    if (fd_ != -1) {
        close(fd_);
        fd_ = -1;
    }
    connected_ = false;
}

bool SerialCommunicator::configure_port() {
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
}

bool SerialCommunicator::send_packet(SerialCommand cmd, const std::vector<uint8_t>& payload) {
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
}

bool SerialCommunicator::send_state_with_text(int state_id, const std::string& text) {
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
}

bool SerialCommunicator::send_nose_position(float x, float y) {
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
}

bool SerialCommunicator::send_progress(int progress_percent) {
    if (progress_percent < 0) progress_percent = 0;
    if (progress_percent > 100) progress_percent = 100;
    std::vector<uint8_t> payload = {static_cast<uint8_t>(progress_percent)};
    return send_packet(SerialCommand::CMD_UPDATE_PROGRESS, payload);
}

bool SerialCommunicator::send_batch_progress(int progress_percent) {
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
}

bool SerialCommunicator::send_tracking_data(int x, int y, int progress_percent, bool target_valid) {
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
}

bool SerialCommunicator::send_error(const std::string& error_msg) {
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
}

bool SerialCommunicator::auto_detect_port() {
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
}

bool SerialCommunicator::try_reconnect() {
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
}

// ============================================================================
// BINARY PROTOCOL IMPLEMENTATION
// ============================================================================

uint8_t SerialCommunicator::get_next_sequence() {
    return sequence_number_.fetch_add(1);
}

bool SerialCommunicator::send_binary_packet(uint8_t cmd, const uint8_t* payload, uint8_t len) {
    if (!connected_ || fd_ == -1) return false;
    if (len > 32) return false;  // Max payload size
    
    std::vector<uint8_t> packet;
    packet.reserve(len + 6);  // START + SEQ + CMD + LEN + PAYLOAD + CHECKSUM + END
    
    uint8_t seq = get_next_sequence();
    
    packet.push_back(SERIAL_START_BYTE);  // START
    packet.push_back(seq);                 // SEQ
    packet.push_back(cmd);                 // CMD
    packet.push_back(len);                 // LEN
    
    // PAYLOAD
    for (uint8_t i = 0; i < len; i++) {
        packet.push_back(payload[i]);
    }
    
    // CHECKSUM (XOR of SEQ, CMD, LEN, and payload)
    uint8_t checksum = seq ^ cmd ^ len;
    for (uint8_t i = 0; i < len; i++) {
        checksum ^= payload[i];
    }
    packet.push_back(checksum);
    
    packet.push_back(SERIAL_END_BYTE);    // END
    
    std::lock_guard<std::mutex> lock(write_mutex_);
    ssize_t written = write(fd_, packet.data(), packet.size());
    
    return written == static_cast<ssize_t>(packet.size());
}

bool SerialCommunicator::wait_for_ack(uint8_t seq, int timeout_ms) {
    auto start = std::chrono::steady_clock::now();
    
    while (true) {
        // Check if we received the ACK
        if (last_ack_seq_.load() == seq) {
            return true;
        }
        
        // Check timeout
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= timeout_ms) {
            return false;
        }
        
        // Read and process incoming data
        check_esp32_status();
        
        // Small delay to avoid busy-waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

bool SerialCommunicator::send_tracking_binary(int x, int y, int progress_percent, bool target_valid) {
    // Clamp values
    if (x < 0) x = 0;
    if (x > 465) x = 465;
    if (y < 0) y = 0;
    if (y > 465) y = 465;
    if (progress_percent < 0) progress_percent = 0;
    if (progress_percent > 100) progress_percent = 100;
    
    // Build binary tracking packet
    BinaryTrackingData data;
    data.x = static_cast<uint16_t>(x);
    data.y = static_cast<uint16_t>(y);
    data.progress = static_cast<uint8_t>(progress_percent);
    data.flags = target_valid ? 0x01 : 0x00;
    
    // Send without waiting for ACK (tracking is high-frequency, losing one packet is OK)
    return send_binary_packet(BIN_CMD_TRACKING, reinterpret_cast<uint8_t*>(&data), sizeof(data));
}

bool SerialCommunicator::send_state_reliable(int state_id, int max_retries, int timeout_ms) {
    uint8_t payload[1] = { static_cast<uint8_t>(state_id) };
    
    for (int attempt = 0; attempt < max_retries; attempt++) {
        uint8_t seq = sequence_number_.load();  // Get current seq before sending
        
        if (send_binary_packet(BIN_CMD_STATE, payload, 1)) {
            // Wait for ACK
            if (wait_for_ack(seq, timeout_ms)) {
                last_state_ = state_id;
                std::cout << "[Serial] State " << state_id << " ACKed (attempt " << (attempt + 1) << ")" << std::endl;
                return true;
            } else {
                std::cout << "[Serial] State " << state_id << " timeout (attempt " << (attempt + 1) << "/" << max_retries << ")" << std::endl;
            }
        }
    }
    
    std::cerr << "[Serial] State " << state_id << " FAILED after " << max_retries << " attempts" << std::endl;
    
    // Fall back to text protocol
    std::cout << "[Serial] Falling back to text protocol..." << std::endl;
    return send_state(state_id);
}

// ============================================================================
// ESP32 STATUS MONITORING
// ============================================================================

bool SerialCommunicator::check_esp32_status() {
    if (!connected_ || fd_ == -1) return false;
    
    // Static buffer for binary packet reception
    static uint8_t bin_buf[40];
    static int bin_buf_pos = 0;
    static bool in_binary_packet = false;
    
    // Read available data from serial port (non-blocking)
    char temp_buf[128];
    ssize_t bytes_read = read(fd_, temp_buf, sizeof(temp_buf) - 1);
    
    if (bytes_read <= 0) {
        // No data available (non-blocking mode)
        return is_esp32_alive();
    }
    
    // Process received bytes
    for (ssize_t i = 0; i < bytes_read; i++) {
        uint8_t c = static_cast<uint8_t>(temp_buf[i]);
        
        // Check for binary packet start
        if (c == SERIAL_START_BYTE && !in_binary_packet) {
            in_binary_packet = true;
            bin_buf_pos = 0;
            bin_buf[bin_buf_pos++] = c;
            continue;
        }
        
        // If in binary packet mode, collect bytes
        if (in_binary_packet) {
            bin_buf[bin_buf_pos++] = c;
            
            // Binary ACK/NAK format: [START][SEQ][CMD][CHECKSUM][END] = 5 bytes
            if (bin_buf_pos >= 5) {
                // Check if we have a complete ACK/NAK packet
                if (bin_buf[4] == SERIAL_END_BYTE && 
                    (bin_buf[2] == BIN_CMD_ACK || bin_buf[2] == BIN_CMD_NAK)) {
                    
                    uint8_t seq = bin_buf[1];
                    uint8_t cmd = bin_buf[2];
                    uint8_t checksum = bin_buf[3];
                    uint8_t expected_checksum = seq ^ cmd;
                    
                    if (checksum == expected_checksum) {
                        if (cmd == BIN_CMD_ACK) {
                            // ACK received!
                            last_ack_seq_.store(seq);
                            
                            // Update heartbeat since ESP32 is responding
                            {
                                std::lock_guard<std::mutex> lock(heartbeat_mutex_);
                                last_heartbeat_time_ = std::chrono::steady_clock::now();
                            }
                        } else {
                            // NAK received
                            std::cerr << "[Serial] NAK received for seq=" << (int)seq << std::endl;
                        }
                    } else {
                        std::cerr << "[Serial] Binary checksum mismatch" << std::endl;
                    }
                    
                    in_binary_packet = false;
                    bin_buf_pos = 0;
                    continue;
                }
            }
            
            // Overflow protection or invalid packet
            if (bin_buf_pos >= 10) {
                in_binary_packet = false;
                bin_buf_pos = 0;
            }
            continue;
        }
        
        // Text protocol handling
        if (c == '\n' || c == '\r') {
            if (read_buffer_pos_ > 0) {
                read_buffer_[read_buffer_pos_] = '\0';
                
                // Parse the message
                std::string msg(read_buffer_);
                
                // Heartbeat format: "HB:<screen>"
                if (msg.rfind("HB:", 0) == 0) {
                    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
                    last_heartbeat_time_ = std::chrono::steady_clock::now();
                    
                    // Extract screen number
                    try {
                        esp32_current_screen_ = std::stoi(msg.substr(3));
                    } catch (...) {
                        // Ignore parse errors
                    }
                }
                // Status format: "STATUS:<screen>:<heap>:<error>"
                else if (msg.rfind("STATUS:", 0) == 0) {
                    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
                    last_heartbeat_time_ = std::chrono::steady_clock::now();
                    
                    // Parse: STATUS:6:150000:0
                    size_t pos1 = msg.find(':', 7);
                    size_t pos2 = msg.find(':', pos1 + 1);
                    
                    if (pos1 != std::string::npos && pos2 != std::string::npos) {
                        try {
                            esp32_current_screen_ = std::stoi(msg.substr(7, pos1 - 7));
                            esp32_heap_ = std::stoul(msg.substr(pos1 + 1, pos2 - pos1 - 1));
                            esp32_has_error_ = (msg.substr(pos2 + 1) == "1");
                        } catch (...) {
                            // Ignore parse errors
                        }
                    }
                    
                    // Log status
                    std::cout << "[Serial] ESP32 Status: Screen=" << esp32_current_screen_ 
                              << ", Heap=" << esp32_heap_ 
                              << ", Error=" << (esp32_has_error_ ? "YES" : "NO") << std::endl;
                }
                // ACK format: "ACK:<cmd>" (text ACK from old protocol)
                else if (msg.rfind("ACK:", 0) == 0) {
                    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
                    last_heartbeat_time_ = std::chrono::steady_clock::now();
                }
                // Binary state confirmation: "BIN:STATE:<id>"
                else if (msg.rfind("BIN:STATE:", 0) == 0) {
                    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
                    last_heartbeat_time_ = std::chrono::steady_clock::now();
                    try {
                        esp32_current_screen_ = std::stoi(msg.substr(10));
                    } catch (...) {}
                }
                // Error format: "ERR:<code>:<message>"
                else if (msg.rfind("ERR:", 0) == 0) {
                    std::cerr << "[Serial] ESP32 Error: " << msg << std::endl;
                }
                // Binary errors
                else if (msg.rfind("BIN:ERR:", 0) == 0) {
                    std::cerr << "[Serial] ESP32 Binary Error: " << msg << std::endl;
                }
                // OK responses
                else if (msg.rfind("OK:", 0) == 0) {
                    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
                    last_heartbeat_time_ = std::chrono::steady_clock::now();
                }
                
                read_buffer_pos_ = 0;
            }
        } else {
            if (read_buffer_pos_ < (int)sizeof(read_buffer_) - 1) {
                read_buffer_[read_buffer_pos_++] = static_cast<char>(c);
            } else {
                // Buffer overflow, reset
                read_buffer_pos_ = 0;
            }
        }
    }
    
    return is_esp32_alive();
}

bool SerialCommunicator::is_esp32_alive(int timeout_ms) const {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_heartbeat_time_).count();
    
    return elapsed < timeout_ms;
}

} // namespace mdai
