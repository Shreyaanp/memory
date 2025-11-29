#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <cstdint>
#include <thread>
#include <queue>
#include <atomic>
#include <condition_variable>
#include <variant>
#include <chrono>

namespace mdai {

// Serial protocol constants
constexpr uint8_t SERIAL_START_BYTE = 0xAA;
constexpr uint8_t SERIAL_END_BYTE = 0x55;

// Binary command IDs (must match ESP32 SerialManager)
#define BIN_CMD_STATE     0x01
#define BIN_CMD_TRACKING  0x02
#define BIN_CMD_BATCH     0x03
#define BIN_CMD_ERROR     0x04
#define BIN_CMD_PING      0x05
#define BIN_CMD_ACK       0x06
#define BIN_CMD_NAK       0x15

// Serial commands (matching LilyGo ESP32 protocol)
enum class SerialCommand : uint8_t {
    CMD_SET_STATE = 0x01,       // Switch to a specific screen/state
    CMD_UPDATE_NOSE = 0x02,     // Update nose tracking dot position
    CMD_UPDATE_PROGRESS = 0x03, // Update circular progress ring
    CMD_SHOW_ERROR = 0x04,      // Display error message
    CMD_HEARTBEAT = 0x05        // Keepalive
};

// Binary tracking data structure (must match ESP32)
struct BinaryTrackingData {
    uint16_t x;
    uint16_t y;
    uint8_t progress;
    uint8_t flags;  // bit 0 = target_valid
} __attribute__((packed));

// Async message types for queue
struct SerialMsg_State { int state_id; std::string text; };
struct SerialMsg_Tracking { int x; int y; int progress; bool valid; };
struct SerialMsg_BatchProgress { int progress; };
struct SerialMsg_Error { std::string message; };

using SerialMessage = std::variant<SerialMsg_State, SerialMsg_Tracking, SerialMsg_BatchProgress, SerialMsg_Error>;

/**
 * @brief Configuration for serial communication
 */
struct SerialConfig {
    std::string port_name = "/dev/ttyACM0";  // LilyGo ESP32
    int baud_rate = 115200;
    int timeout_ms = 1000;
    bool async_mode = true;  // Use async queue by default
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
    
    // ============================================
    // Async Queue API (non-blocking)
    // ============================================
    
    /**
     * @brief Start async worker thread
     */
    void start_async();
    
    /**
     * @brief Stop async worker thread
     */
    void stop_async();
    
    /**
     * @brief Queue state change (async)
     */
    void queue_state(int state_id, const std::string& text = "");
    
    /**
     * @brief Queue tracking data (async)
     */
    void queue_tracking(int x, int y, int progress_percent, bool target_valid);
    
    /**
     * @brief Queue batch progress (async)
     */
    void queue_batch_progress(int progress_percent);
    
    /**
     * @brief Queue error message (async)
     */
    void queue_error(const std::string& error_msg);
    
    /**
     * @brief Get queue size
     */
    size_t get_queue_size() const;
    
    /**
     * @brief Get dropped message count
     */
    uint64_t get_queue_dropped() const { return queue_dropped_.load(); }
    
    // ============================================
    // Binary Protocol API
    // ============================================
    
    /**
     * @brief Send tracking data using binary protocol (more efficient)
     */
    bool send_tracking_binary(int x, int y, int progress_percent, bool target_valid);
    
    /**
     * @brief Read feedback from ESP32 (heartbeat, ACK, status)
     */
    void read_serial_feedback();
    
    /**
     * @brief Check if ESP32 is responsive (received heartbeat recently)
     */
    bool is_esp32_responsive() const;
    
    /**
     * @brief Get ESP32's reported screen (from status messages)
     */
    int get_esp32_screen() const { return esp32_reported_screen_.load(); }
    
private:
    bool configure_port();
    bool send_packet(SerialCommand cmd, const std::vector<uint8_t>& payload);
    uint8_t calculate_checksum(const std::vector<uint8_t>& data);
    bool auto_detect_port();  // Auto-detect ttyACM0 or ttyACM1
    
    // Binary protocol helpers
    bool send_binary_packet(uint8_t cmd, const uint8_t* payload, uint8_t len);
    bool wait_for_ack(uint8_t seq, int timeout_ms);
    
    // Async worker
    void async_worker();
    void process_message(const SerialMessage& msg);
    
    Config config_;
    int fd_ = -1;
    bool connected_ = false;
    std::mutex write_mutex_;
    int last_state_ = -1;  // Track last sent state for recovery
    
    // Async queue members
    std::thread async_thread_;
    std::atomic<bool> async_running_{false};
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::queue<SerialMessage> message_queue_;
    static constexpr size_t MAX_QUEUE_SIZE = 50;
    std::atomic<uint64_t> queue_dropped_{0};
    
    // Binary protocol members
    std::atomic<uint8_t> sequence_number_{0};
    std::condition_variable ack_cv_;
    std::mutex ack_mutex_;
    uint8_t last_acked_sequence_ = 0;
    bool ack_received_ = false;
    
    // ESP32 feedback
    std::chrono::steady_clock::time_point last_heartbeat_;
    std::atomic<int> esp32_reported_screen_{-1};
    std::string serial_read_buffer_;
};

}  // namespace mdai


