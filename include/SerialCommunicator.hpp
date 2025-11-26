#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <cstdint>

namespace mdai {

// Serial protocol constants
constexpr uint8_t SERIAL_START_BYTE = 0xAA;
constexpr uint8_t SERIAL_END_BYTE = 0x55;

// Serial commands (matching LilyGo ESP32 protocol)
enum class SerialCommand : uint8_t {
    CMD_SET_STATE = 0x01,       // Switch to a specific screen/state
    CMD_UPDATE_NOSE = 0x02,     // Update nose tracking dot position
    CMD_UPDATE_PROGRESS = 0x03, // Update circular progress ring
    CMD_SHOW_ERROR = 0x04,      // Display error message
    CMD_HEARTBEAT = 0x05        // Keepalive
};

/**
 * @brief Configuration for serial communication
 */
struct SerialConfig {
    std::string port_name = "/dev/ttyACM0";  // LilyGo ESP32
    int baud_rate = 115200;
    int timeout_ms = 1000;
};

/**
 * @brief Serial communicator for LilyGo AMOLED display
 * 
 * Sends commands to ESP32-based display over UART.
 * Protocol: START(0xAA) | LEN | CMD | PAYLOAD... | CHECKSUM | END(0x55)
 */
class SerialCommunicator {
public:
    using Config = SerialConfig;
    
    explicit SerialCommunicator(const Config& config);
    ~SerialCommunicator();
    
    /**
     * @brief Connect to serial port
     */
    bool connect();
    
    /**
     * @brief Disconnect from serial port
     */
    void disconnect();
    
    /**
     * @brief Check if connected
     */
    bool is_connected() const { return connected_; }
    
    /**
     * @brief Send state change command to LilyGo
     * @param state_id State ID (0-10 corresponding to screens)
     */
    bool send_state(int state_id);
    
    /**
     * @brief Send state change with optional text
     * @param state_id State/Screen ID (1-13)
     * @param text Optional text to display on the screen
     * @return true if sent successfully
     */
    bool send_state_with_text(int state_id, const std::string& text);
    
    /**
     * @brief Send nose tracking position update
     * @param x X coordinate (0.0-1.0, normalized)
     * @param y Y coordinate (0.0-1.0, normalized)
     */
    bool send_nose_position(float x, float y);
    
    /**
     * @brief Send progress update for circular motion
     * @param progress_percent Progress (0-100)
     */
    bool send_progress(int progress_percent);
    
    /**
     * @brief Send combined tracking data (position, progress, target color)
     * @param x X coordinate (screen pixels)
     * @param y Y coordinate (screen pixels)
     * @param progress_percent Progress (0-100)
     * @param target_valid true = green target, false = grey target
     * @return true if sent successfully
     * 
     * Format: "X:233,Y:180,P:75,C:1\n"
     */
    bool send_tracking_data(int x, int y, int progress_percent, bool target_valid);
    
    /**
     * @brief Send error message to display
     * @param error_msg Error message string (max 250 chars)
     */
    bool send_error(const std::string& error_msg);
    
    /**
     * @brief Try to reconnect if disconnected
     * @return true if reconnected successfully
     */
    bool try_reconnect();
    
    /**
     * @brief Get last sent state (for recovery)
     */
    int get_last_state() const { return last_state_; }
    
private:
    bool configure_port();
    bool send_packet(SerialCommand cmd, const std::vector<uint8_t>& payload);
    uint8_t calculate_checksum(const std::vector<uint8_t>& data);
    bool auto_detect_port();  // Auto-detect ttyACM0 or ttyACM1
    
    Config config_;
    int fd_ = -1;
    bool connected_ = false;
    std::mutex write_mutex_;
    int last_state_ = -1;  // Track last sent state for recovery
};

}  // namespace mdai


