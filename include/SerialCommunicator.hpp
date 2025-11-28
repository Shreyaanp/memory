#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <cstdint>
#include <chrono>
#include <atomic>
#include <condition_variable>

namespace mdai {

// Serial protocol constants
constexpr uint8_t SERIAL_START_BYTE = 0xAA;
constexpr uint8_t SERIAL_END_BYTE = 0x55;

// Binary protocol commands
constexpr uint8_t BIN_CMD_STATE = 0x01;     // Set screen state
constexpr uint8_t BIN_CMD_TRACKING = 0x02;  // Tracking data (X, Y, Progress, Valid)
constexpr uint8_t BIN_CMD_BATCH = 0x03;     // Batch progress
constexpr uint8_t BIN_CMD_ERROR = 0x04;     // Error message
constexpr uint8_t BIN_CMD_PING = 0x05;      // Ping (keepalive)
constexpr uint8_t BIN_CMD_ACK = 0x06;       // Acknowledgment
constexpr uint8_t BIN_CMD_NAK = 0x15;       // Negative acknowledgment

// Binary tracking packet structure (6 bytes, packed)
#pragma pack(push, 1)
struct BinaryTrackingData {
    uint16_t x;         // X position (0-465)
    uint16_t y;         // Y position (0-465)
    uint8_t progress;   // Progress (0-100)
    uint8_t flags;      // Bit 0: valid, Bits 1-7: reserved
};
#pragma pack(pop)

// Serial commands (matching LilyGo ESP32 protocol) - Legacy
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
     * @brief Send progress update for circular motion (binary packet)
     * @param progress_percent Progress (0-100)
     */
    bool send_progress(int progress_percent);
    
    /**
     * @brief Send batch processing progress (text format for Screen 10)
     * @param progress_percent Progress (0-100)
     * Format: "B:50\n"
     */
    bool send_batch_progress(int progress_percent);
    
    /**
     * @brief Send combined tracking data (position, progress, target color)
     * @param x X coordinate (screen pixels)
     * @param y Y coordinate (screen pixels)
     * @param progress_percent Progress (0-100)
     * @param target_valid true = green target, false = grey target
     * @return true if sent successfully
     * 
     * Format: "X:233,Y:180,P:75,C:1\n" (text mode)
     */
    bool send_tracking_data(int x, int y, int progress_percent, bool target_valid);
    
    /**
     * @brief Send tracking data using binary protocol (more efficient)
     * @param x X coordinate (0-465)
     * @param y Y coordinate (0-465)
     * @param progress_percent Progress (0-100)
     * @param target_valid true = green target, false = grey target
     * @return true if sent and ACKed successfully
     * 
     * Binary format: [0xAA][SEQ][0x02][6][X_lo][X_hi][Y_lo][Y_hi][P][FLAGS][CHK][0x55]
     * Total: 12 bytes vs 24 bytes text = 50% reduction
     */
    bool send_tracking_binary(int x, int y, int progress_percent, bool target_valid);
    
    /**
     * @brief Send state change with ACK waiting and retry
     * @param state_id State/Screen ID
     * @param max_retries Maximum retry attempts (default: 3)
     * @param timeout_ms Timeout per attempt in ms (default: 200)
     * @return true if sent and ACKed successfully
     */
    bool send_state_reliable(int state_id, int max_retries = 3, int timeout_ms = 200);
    
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
    
    /**
     * @brief Check for incoming messages from ESP32 (heartbeat, status, etc.)
     * Should be called periodically from main loop
     * @return true if ESP32 is responding (heartbeat received recently)
     */
    bool check_esp32_status();
    
    /**
     * @brief Check if ESP32 is alive (received heartbeat within timeout)
     * @param timeout_ms Timeout in milliseconds (default: 5 seconds)
     * @return true if ESP32 is alive
     */
    bool is_esp32_alive(int timeout_ms = 5000) const;
    
    /**
     * @brief Get last reported screen from ESP32
     */
    int get_esp32_screen() const { return esp32_current_screen_; }
    
    /**
     * @brief Get ESP32 heap status (from last STATUS message)
     */
    uint32_t get_esp32_heap() const { return esp32_heap_; }
    
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
    
    // ESP32 heartbeat monitoring
    mutable std::mutex heartbeat_mutex_;
    std::chrono::steady_clock::time_point last_heartbeat_time_;
    int esp32_current_screen_ = -1;   // Last reported screen from ESP32
    uint32_t esp32_heap_ = 0;         // Last reported heap from ESP32
    bool esp32_has_error_ = false;    // ESP32 error flag
    char read_buffer_[256];           // Buffer for reading from ESP32
    int read_buffer_pos_ = 0;
    
    // Binary protocol state
    std::atomic<uint8_t> sequence_number_{0};     // Sequence number for packets
    std::atomic<uint8_t> last_ack_seq_{0};        // Last ACKed sequence number
    std::atomic<bool> waiting_for_ack_{false};    // Whether waiting for ACK
    std::condition_variable ack_cv_;              // Condition variable for ACK waiting
    std::mutex ack_mutex_;                        // Mutex for ACK signaling
    
    // Binary protocol helpers
    bool send_binary_packet(uint8_t cmd, const uint8_t* payload, uint8_t len);
    bool wait_for_ack(uint8_t seq, int timeout_ms);
    uint8_t get_next_sequence();
};

}  // namespace mdai


