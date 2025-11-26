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

} // namespace mdai
