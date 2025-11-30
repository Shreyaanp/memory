#pragma once

#include <string>
#include <functional>
#include <atomic>
#include <thread>
#include <vector>
#include <mutex>
#include <queue>
#include <condition_variable>

namespace mdai {

class NetworkManager {
public:
    using MsgCallback = std::function<void(const std::string&)>;
    using ConnectCallback = std::function<void()>;

    NetworkManager();
    ~NetworkManager();

    // WiFi Management
    bool connect_wifi(const std::string& ssid, const std::string& password);
    bool is_connected_to_internet();
    std::string get_ip_address();
    std::string get_current_ssid();  // Get currently connected WiFi SSID
    void start_wifi_monitor();
    void stop_wifi_monitor();
    bool is_wifi_stable() const;

    // WebSocket Client (WSS)
    bool connect_to_middleware(const std::string& host, int port, const std::string& path, const std::string& device_id);
    void disconnect();
    void stop_reconnect();  // Stop reconnection loop without blocking (use after delete/logout)
    bool send_message(const std::string& message);  // Synchronous send
    void queue_message(const std::string& message); // Async send via queue (non-blocking)
    void set_message_callback(MsgCallback cb);
    void set_connect_callback(ConnectCallback cb);  // Called on (re)connect
    bool is_connected() const;
    
    // HTTP Image Upload
    std::string upload_image(const std::string& host, const std::vector<uint8_t>& jpeg_data, 
                             const std::string& session_id, const std::string& device_id);
    
    // Queue statistics
    size_t get_send_queue_size() const;
    uint64_t get_send_queue_dropped() const;

private:
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::thread client_thread_;
    MsgCallback message_callback_;
    ConnectCallback connect_callback_;
    
    // WiFi monitoring
    std::atomic<bool> wifi_monitor_running_{false};
    std::thread wifi_monitor_thread_;
    std::string last_ssid_;
    std::string last_password_;
    
    // Socket handles
    int socket_fd_ = -1;
    void* ssl_ctx_ = nullptr; // OpenSSL SSL_CTX
    void* ssl_ = nullptr;     // OpenSSL SSL connection

    // Internal helpers
    void client_loop(std::string host, int port, std::string path, std::string device_id);
    void wifi_monitor_loop();
    bool perform_handshake(const std::string& host, const std::string& path, const std::string& device_id);
    std::string receive_frame();
    bool send_frame_internal(const std::string& message);
    
    // SSL helpers
    bool init_ssl();
    void cleanup_ssl();
    int net_read(void* buf, int len);   // Read from SSL or plain socket
    int net_write(const void* buf, int len); // Write to SSL or plain socket
    
    // Send queue (for async non-blocking sends)
    mutable std::mutex send_queue_mutex_;
    std::queue<std::string> send_queue_;
    static constexpr size_t MAX_SEND_QUEUE = 100;
    std::atomic<uint64_t> send_queue_dropped_{0};
    
    // Cleanup mutex (protects disconnect/cleanup operations)
    mutable std::mutex cleanup_mutex_;
    std::atomic<bool> cleanup_in_progress_{false};
    
    void process_send_queue();  // Called from client_loop
};

} // namespace mdai
