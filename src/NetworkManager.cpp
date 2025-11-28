#include "NetworkManager.hpp"
#include "CryptoUtils.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <netdb.h>
#include <random>
#include <openssl/ssl.h>
#include <openssl/err.h>

namespace mdai {

NetworkManager::NetworkManager() {
    init_ssl();
}

NetworkManager::~NetworkManager() {
    stop_wifi_monitor();
    disconnect();
    cleanup_ssl();
}

bool NetworkManager::init_ssl() {
    SSL_load_error_strings();
    SSL_library_init();
    OpenSSL_add_all_algorithms();
    
    ssl_ctx_ = SSL_CTX_new(TLS_client_method());
    if (!ssl_ctx_) {
        std::cerr << "Unable to create SSL context" << std::endl;
        return false;
    }
    return true;
}

void NetworkManager::cleanup_ssl() {
    if (ssl_ctx_) {
        SSL_CTX_free((SSL_CTX*)ssl_ctx_);
        ssl_ctx_ = nullptr;
    }
    EVP_cleanup();
}

bool NetworkManager::connect_wifi(const std::string& ssid, const std::string& password) {
    std::cout << "Connecting to WiFi: " << ssid << std::endl;
    
    // Store credentials for auto-reconnect
    last_ssid_ = ssid;
    last_password_ = password;
    
    std::string cmd = "nmcli device wifi connect '" + ssid + "' password '" + password + "'";
    bool success = (std::system(cmd.c_str()) == 0);
    
    if (success) {
        std::cout << "✅ WiFi connected successfully" << std::endl;
        // Start monitoring after successful connection
        if (!wifi_monitor_running_) {
            start_wifi_monitor();
        }
    }
    
    return success;
}

bool NetworkManager::is_connected_to_internet() {
    struct hostent* host = gethostbyname("google.com");
    return (host != nullptr);
}

std::string NetworkManager::get_ip_address() {
    // Implementation from previous step remains valid
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == -1) return "0.0.0.0";
    
    // Use Google DNS IP to find route
    struct sockaddr_in serv;
    std::memset(&serv, 0, sizeof(serv));
    serv.sin_family = AF_INET;
    serv.sin_addr.s_addr = inet_addr("8.8.8.8");
    serv.sin_port = htons(53);
    
    if (connect(sock, (const struct sockaddr*)&serv, sizeof(serv)) == -1) {
        close(sock);
        return "0.0.0.0";
    }
    
    struct sockaddr_in name;
    socklen_t namelen = sizeof(name);
    if (getsockname(sock, (struct sockaddr*)&name, &namelen) == -1) {
        close(sock);
        return "0.0.0.0";
    }
    
    char buffer[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &name.sin_addr, buffer, sizeof(buffer));
    close(sock);
    return std::string(buffer);
}

std::string NetworkManager::get_current_ssid() {
    // Use nmcli to get the currently connected WiFi SSID
    FILE* pipe = popen("nmcli -t -f active,ssid dev wifi | awk -F: '$1 ~ /^yes/ {print $2}'", "r");
    if (!pipe) return "";
    
    char buffer[256];
    std::string ssid;
    if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        ssid = buffer;
        // Remove trailing newline
        if (!ssid.empty() && ssid.back() == '\n') {
            ssid.pop_back();
        }
    }
    pclose(pipe);
    
    return ssid;
}

// ============================================================================
// WebSocket Client (WSS) Implementation
// ============================================================================

bool NetworkManager::connect_to_middleware(const std::string& host, int port, const std::string& path, const std::string& device_id) {
    if (running_) return false;
    
    running_ = true;
    client_thread_ = std::thread(&NetworkManager::client_loop, this, host, port, path, device_id);
    return true;
}

void NetworkManager::disconnect() {
    running_ = false;
    
    if (ssl_) {
        SSL_shutdown((SSL*)ssl_);
        SSL_free((SSL*)ssl_);
        ssl_ = nullptr;
    }
    
    if (socket_fd_ != -1) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    
    connected_ = false;
    
    if (client_thread_.joinable()) {
        client_thread_.join();
    }
}

void NetworkManager::stop_reconnect() {
    // Stop the reconnection loop without blocking
    // Use this after delete/logout to prevent auto-reconnect
    running_ = false;
    connected_ = false;
    std::cout << "🔌 WebSocket reconnection disabled" << std::endl;
}

bool NetworkManager::is_connected() const {
    return connected_;
}

void NetworkManager::set_message_callback(MsgCallback cb) {
    message_callback_ = cb;
}

void NetworkManager::set_connect_callback(ConnectCallback cb) {
    connect_callback_ = cb;
}

void NetworkManager::client_loop(std::string host, int port, std::string path, std::string device_id) {
    while (running_) {
        if (!connected_) {
            // 1. Create Socket
            socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
            if (socket_fd_ < 0) {
                std::this_thread::sleep_for(std::chrono::seconds(5));
                continue;
            }

            // 2. Resolve Host
            struct hostent* server = gethostbyname(host.c_str());
            if (!server) {
                std::cerr << "Could not resolve host: " << host << std::endl;
                close(socket_fd_);
                std::this_thread::sleep_for(std::chrono::seconds(5));
                continue;
            }

            struct sockaddr_in serv_addr;
            std::memset(&serv_addr, 0, sizeof(serv_addr));
            serv_addr.sin_family = AF_INET;
            std::memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
            serv_addr.sin_port = htons(port);

            // 3. Connect TCP
            if (connect(socket_fd_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
                std::cerr << "Connection failed" << std::endl;
                close(socket_fd_);
                std::this_thread::sleep_for(std::chrono::seconds(5));
                continue;
            }

            // 4. Upgrade to SSL
            ssl_ = SSL_new((SSL_CTX*)ssl_ctx_);
            SSL_set_fd((SSL*)ssl_, socket_fd_);
            if (SSL_connect((SSL*)ssl_) <= 0) {
                std::cerr << "SSL handshake failed" << std::endl;
                ERR_print_errors_fp(stderr);
                close(socket_fd_);
                std::this_thread::sleep_for(std::chrono::seconds(5));
                continue;
            }

            // 5. WebSocket Handshake
            if (perform_handshake(host, path, device_id)) {
                connected_ = true;
                std::cout << "✓ Connected to Middleware (WSS)" << std::endl;
                
                // Notify on (re)connect so caller can re-auth if needed
                if (connect_callback_) {
                    connect_callback_();
                }
            } else {
                std::cerr << "WebSocket handshake failed" << std::endl;
                close(socket_fd_);
                std::this_thread::sleep_for(std::chrono::seconds(5));
                continue;
            }
        }

        // 6. Read Loop
        std::string msg = receive_frame();
        if (msg.empty()) {
            std::cout << "🔌 WebSocket disconnected" << std::endl;
            connected_ = false;
            if (ssl_) {
                SSL_shutdown((SSL*)ssl_);
                SSL_free((SSL*)ssl_);
                ssl_ = nullptr;
            }
            close(socket_fd_);
            continue;
        }

        if (message_callback_) {
            message_callback_(msg);
        }
    }
}

bool NetworkManager::perform_handshake(const std::string& host, const std::string& path, const std::string& device_id) {
    // Generate random 16-byte key
    std::string key_bytes(16, ' ');
    std::random_device rd;
    std::uniform_int_distribution<int> dist(0, 255);
    for (int i=0; i<16; ++i) key_bytes[i] = static_cast<char>(dist(rd));
    
    std::string sec_key = CryptoUtils::base64_encode(key_bytes);
    
    std::string request = 
        "GET " + path + "?device_id=" + device_id + " HTTP/1.1\r\n"
        "Host: " + host + "\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: " + sec_key + "\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n";
    
    if (SSL_write((SSL*)ssl_, request.c_str(), request.length()) <= 0) return false;
    
    // Read Response Headers (Basic parsing)
    char buffer[4096];
    int bytes = SSL_read((SSL*)ssl_, buffer, sizeof(buffer) - 1);
    if (bytes <= 0) return false;
    buffer[bytes] = '\0';
    
    std::string response(buffer);
    return (response.find("101 Switching Protocols") != std::string::npos);
}

bool NetworkManager::send_message(const std::string& message) {
    return send_frame_internal(message);
}

bool NetworkManager::send_frame_internal(const std::string& message) {
    if (!connected_ || !ssl_) return false;
    
    std::vector<uint8_t> frame;
    frame.push_back(0x81); // FIN + Text Frame
    
    // Proper length encoding per WebSocket RFC 6455
    size_t payload_len = message.length();
    if (payload_len <= 125) {
        frame.push_back(0x80 | payload_len);  // MASK bit + length
    } else if (payload_len <= 65535) {
        frame.push_back(0x80 | 126);  // MASK bit + 126 (extended 16-bit length)
        frame.push_back((payload_len >> 8) & 0xFF);
        frame.push_back(payload_len & 0xFF);
    } else {
        frame.push_back(0x80 | 127);  // MASK bit + 127 (extended 64-bit length)
        for (int i = 7; i >= 0; --i) {
            frame.push_back((payload_len >> (i * 8)) & 0xFF);
        }
    }
    
    // Masking Key (should be random per RFC, using fixed for simplicity)
    uint8_t mask[4] = {0x12, 0x34, 0x56, 0x78}; 
    frame.push_back(mask[0]); frame.push_back(mask[1]); frame.push_back(mask[2]); frame.push_back(mask[3]);
    
    // Mask Payload
    for (size_t i = 0; i < message.length(); ++i) {
        frame.push_back(message[i] ^ mask[i % 4]);
    }
    
    int sent = SSL_write((SSL*)ssl_, frame.data(), frame.size());
    return (sent == static_cast<int>(frame.size()));
}

std::string NetworkManager::receive_frame() {
    if (!connected_ || !ssl_) return "";
    
    while (true) {  // Loop to handle control frames
    uint8_t header[2];
        int read_result = SSL_read((SSL*)ssl_, header, 2);
        if (read_result <= 0) {
            int ssl_error = SSL_get_error((SSL*)ssl_, read_result);
            if (ssl_error == SSL_ERROR_WANT_READ || ssl_error == SSL_ERROR_WANT_WRITE) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;  // Retry
            }
            return "";  // Real error or disconnect
        }
        
        uint8_t opcode = header[0] & 0x0F;
        bool is_fin = (header[0] & 0x80) != 0;
    uint64_t payload_len = header[1] & 0x7F;
        
        // Handle extended payload length
    if (payload_len == 126) {
        uint8_t len_bytes[2];
        if (SSL_read((SSL*)ssl_, len_bytes, 2) <= 0) return "";
        payload_len = (len_bytes[0] << 8) | len_bytes[1];
        } else if (payload_len == 127) {
            uint8_t len_bytes[8];
            if (SSL_read((SSL*)ssl_, len_bytes, 8) <= 0) return "";
            payload_len = 0;
            for (int i = 0; i < 8; i++) {
                payload_len = (payload_len << 8) | len_bytes[i];
            }
        }
        
        // Read payload
    std::vector<uint8_t> buffer(payload_len);
    size_t total_read = 0;
    while (total_read < payload_len) {
        int r = SSL_read((SSL*)ssl_, buffer.data() + total_read, payload_len - total_read);
        if (r <= 0) return "";
        total_read += r;
    }
    
        // Handle different frame types
        switch (opcode) {
            case 0x01:  // Text frame
            case 0x02:  // Binary frame
    return std::string(buffer.begin(), buffer.end());
                
            case 0x08:  // Close frame
                {
                    // Extract close code if present
                    uint16_t close_code = 0;
                    std::string close_reason;
                    if (buffer.size() >= 2) {
                        close_code = (buffer[0] << 8) | buffer[1];
                        if (buffer.size() > 2) {
                            close_reason = std::string(buffer.begin() + 2, buffer.end());
                        }
                    }
                    std::cout << "🔌 WebSocket close frame received - Code: " << close_code 
                              << ", Reason: " << (close_reason.empty() ? "(none)" : close_reason) << std::endl;
                    
                    // Send close frame back
                    uint8_t close_resp[2] = {0x88, 0x00};
                    SSL_write((SSL*)ssl_, close_resp, 2);
                }
                return "";  // Signal disconnect
                
            case 0x09:  // Ping frame - respond with Pong
                {
                    std::vector<uint8_t> pong_frame;
                    pong_frame.push_back(0x8A);  // FIN + Pong opcode
                    pong_frame.push_back(0x80 | (payload_len & 0x7F));  // Mask bit + length
                    // Add mask key
                    uint8_t mask[4] = {0x12, 0x34, 0x56, 0x78};
                    pong_frame.insert(pong_frame.end(), mask, mask + 4);
                    // Add masked payload (echo back ping data)
                    for (size_t i = 0; i < buffer.size(); i++) {
                        pong_frame.push_back(buffer[i] ^ mask[i % 4]);
                    }
                    SSL_write((SSL*)ssl_, pong_frame.data(), pong_frame.size());
                }
                continue;  // Keep reading for actual data
                
            case 0x0A:  // Pong frame - ignore
                continue;  // Keep reading for actual data
                
            default:
                // Unknown opcode, skip
                continue;
        }
    }
}

void NetworkManager::start_wifi_monitor() {
    if (wifi_monitor_running_) return;
    
    wifi_monitor_running_ = true;
    wifi_monitor_thread_ = std::thread(&NetworkManager::wifi_monitor_loop, this);
    std::cout << "✅ WiFi monitoring started" << std::endl;
}

void NetworkManager::stop_wifi_monitor() {
    wifi_monitor_running_ = false;
    if (wifi_monitor_thread_.joinable()) {
        wifi_monitor_thread_.join();
    }
}

bool NetworkManager::is_wifi_stable() const {
    // Check if monitoring is running (don't call is_connected_to_internet from const method)
    return wifi_monitor_running_.load();
}

void NetworkManager::wifi_monitor_loop() {
    int consecutive_failures = 0;
    const int max_failures = 3;
    
    while (wifi_monitor_running_) {
        // Check WiFi every 10 seconds
        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        if (!is_connected_to_internet()) {
            consecutive_failures++;
            std::cerr << "⚠️  WiFi check failed (" << consecutive_failures << "/" << max_failures << ")" << std::endl;
            
            if (consecutive_failures >= max_failures) {
                std::cerr << "🔄 WiFi lost! Attempting reconnection..." << std::endl;
                
                // Try to reconnect using stored credentials
                if (!last_ssid_.empty()) {
                    std::string cmd = "nmcli device wifi connect '" + last_ssid_ + "' password '" + last_password_ + "'";
                    if (std::system(cmd.c_str()) == 0) {
                        std::cout << "✅ WiFi reconnected successfully!" << std::endl;
                        consecutive_failures = 0;
                    } else {
                        std::cerr << "❌ WiFi reconnection failed, will retry..." << std::endl;
                        // Wait longer before next attempt
                        std::this_thread::sleep_for(std::chrono::seconds(5));
                    }
                }
            }
        } else {
            // Reset counter on successful check
            if (consecutive_failures > 0) {
                std::cout << "✅ WiFi connection restored" << std::endl;
            }
            consecutive_failures = 0;
        }
    }
}

} // namespace mdai