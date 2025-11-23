#pragma once

#include <string>
#include <functional>
#include <atomic>
#include <thread>
#include <vector>
#include <mutex>

namespace mdai {

class NetworkManager {
public:
    using MsgCallback = std::function<void(const std::string&)>;

    NetworkManager();
    ~NetworkManager();

    // WiFi Management
    bool connect_wifi(const std::string& ssid, const std::string& password);
    bool is_connected_to_internet();
    std::string get_ip_address();
    void start_wifi_monitor();
    void stop_wifi_monitor();
    bool is_wifi_stable() const;

    // WebSocket Client (WSS)
    bool connect_to_middleware(const std::string& host, int port, const std::string& path, const std::string& device_id);
    void disconnect();
    bool send_message(const std::string& message);
    void set_message_callback(MsgCallback cb);
    bool is_connected() const;

private:
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::thread client_thread_;
    MsgCallback message_callback_;
    
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
};

} // namespace mdai
