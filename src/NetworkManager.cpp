#include "NetworkManager.hpp"
#include "CryptoUtils.hpp"
#include <iostream>
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
    // Multi-layer connectivity check:
    // 1. Try to connect to Google DNS (8.8.8.8) on port 53
    // 2. This tests actual network connectivity, not just DNS cache
    
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return false;
    
    // Set socket timeout to 2 seconds
    struct timeval timeout;
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(53);  // DNS port
    addr.sin_addr.s_addr = inet_addr("8.8.8.8");  // Google DNS
    
    // Try to connect (non-blocking would be better but this is simpler)
    int result = connect(sock, (struct sockaddr*)&addr, sizeof(addr));
    close(sock);
    
    if (result == 0) {
        return true;
    }
    
    // Fallback: Try Cloudflare DNS
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return false;
    
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    addr.sin_addr.s_addr = inet_addr("1.1.1.1");  // Cloudflare DNS
    result = connect(sock, (struct sockaddr*)&addr, sizeof(addr));
    close(sock);
    
    return (result == 0);
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

bool NetworkManager::is_connected() const {
    return connected_;
}

void NetworkManager::set_message_callback(MsgCallback cb) {
    message_callback_ = cb;
}

void NetworkManager::client_loop(std::string host, int port, std::string path, std::string device_id) {
    // NO RETRIES: Connect once, if fails or disconnects, exit cleanly
    // Reconnection is handled at the SystemController level via QR scan
    bool initial_connection = true;
    
    while (running_) {
        if (!connected_) {
            // Only allow ONE connection attempt - no retries from RDK side
            if (!initial_connection) {
                std::cerr << "⚠️  WebSocket disconnected - not retrying (handled by QR scan)" << std::endl;
                running_ = false;
                
                // Notify via callback that connection was lost
                if (message_callback_) {
                    message_callback_("{\"type\":\"disconnected\",\"reason\":\"connection_lost\"}");
                }
                return;
            }
            
            initial_connection = false;  // Mark that we've tried once
            std::cout << "🔌 WebSocket connecting (single attempt, no retries)..." << std::endl;
            
            // 1. Create Socket
            socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
            if (socket_fd_ < 0) {
                std::cerr << "❌ Socket creation failed - not retrying" << std::endl;
                running_ = false;
                if (message_callback_) {
                    message_callback_("{\"type\":\"connection_failed\",\"reason\":\"socket_error\"}");
                }
                return;
            }

            // Set socket timeout
            struct timeval timeout;
            timeout.tv_sec = 5;
            timeout.tv_usec = 0;
            setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
            setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

            // 2. Resolve Host
            struct hostent* server = gethostbyname(host.c_str());
            if (!server) {
                std::cerr << "❌ Could not resolve host: " << host << " - not retrying" << std::endl;
                close(socket_fd_);
                running_ = false;
                if (message_callback_) {
                    message_callback_("{\"type\":\"connection_failed\",\"reason\":\"dns_error\"}");
                }
                return;
            }

            struct sockaddr_in serv_addr;
            std::memset(&serv_addr, 0, sizeof(serv_addr));
            serv_addr.sin_family = AF_INET;
            std::memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
            serv_addr.sin_port = htons(port);

            // 3. Connect TCP
            if (connect(socket_fd_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
                std::cerr << "❌ TCP connection failed - not retrying" << std::endl;
                close(socket_fd_);
                running_ = false;
                if (message_callback_) {
                    message_callback_("{\"type\":\"connection_failed\",\"reason\":\"tcp_error\"}");
                }
                return;
            }

            // 4. Upgrade to SSL
            ssl_ = SSL_new((SSL_CTX*)ssl_ctx_);
            SSL_set_fd((SSL*)ssl_, socket_fd_);
            if (SSL_connect((SSL*)ssl_) <= 0) {
                std::cerr << "❌ SSL handshake failed - not retrying" << std::endl;
                ERR_print_errors_fp(stderr);
                SSL_free((SSL*)ssl_);
                ssl_ = nullptr;
                close(socket_fd_);
                running_ = false;
                if (message_callback_) {
                    message_callback_("{\"type\":\"connection_failed\",\"reason\":\"ssl_error\"}");
                }
                return;
            }

            // 5. WebSocket Handshake
            if (perform_handshake(host, path, device_id)) {
                connected_ = true;
                std::cout << "✅ Connected to Middleware (WSS)" << std::endl;
            } else {
                std::cerr << "❌ WebSocket handshake failed - not retrying" << std::endl;
                SSL_shutdown((SSL*)ssl_);
                SSL_free((SSL*)ssl_);
                ssl_ = nullptr;
                close(socket_fd_);
                running_ = false;
                if (message_callback_) {
                    message_callback_("{\"type\":\"connection_failed\",\"reason\":\"handshake_error\"}");
                }
                return;
            }
        }

        // 6. Read Loop
        std::string msg = receive_frame();
        if (msg.empty()) {
            std::cerr << "⚠️  Connection lost" << std::endl;
            connected_ = false;
            if (ssl_) {
                SSL_shutdown((SSL*)ssl_);
                SSL_free((SSL*)ssl_);
                ssl_ = nullptr;
            }
            close(socket_fd_);
            socket_fd_ = -1;
            
            // Notify about disconnection
            if (message_callback_) {
                message_callback_("{\"type\":\"disconnected\"}");
            }
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
    
    // Proper WebSocket length encoding per RFC 6455
    size_t payload_len = message.length();
    
    if (payload_len <= 125) {
        // 7-bit length
        frame.push_back(0x80 | static_cast<uint8_t>(payload_len));
    } else if (payload_len <= 65535) {
        // 16-bit length
        frame.push_back(0x80 | 126);
        frame.push_back(static_cast<uint8_t>((payload_len >> 8) & 0xFF));
        frame.push_back(static_cast<uint8_t>(payload_len & 0xFF));
    } else {
        // 64-bit length (for very large messages like images)
        frame.push_back(0x80 | 127);
        for (int i = 7; i >= 0; --i) {
            frame.push_back(static_cast<uint8_t>((payload_len >> (i * 8)) & 0xFF));
        }
    }
    
    // Generate random masking key (required for client-to-server)
    uint8_t mask[4];
    std::random_device rd;
    std::uniform_int_distribution<int> dist(0, 255);
    for (int i = 0; i < 4; ++i) {
        mask[i] = static_cast<uint8_t>(dist(rd));
    }
    frame.push_back(mask[0]);
    frame.push_back(mask[1]);
    frame.push_back(mask[2]);
    frame.push_back(mask[3]);
    
    // Mask and append payload
    for (size_t i = 0; i < message.length(); ++i) {
        frame.push_back(message[i] ^ mask[i % 4]);
    }
    
    // Send in chunks if large (SSL may have buffer limits)
    size_t total_sent = 0;
    while (total_sent < frame.size()) {
        int chunk_size = std::min(static_cast<size_t>(16384), frame.size() - total_sent);
        int sent = SSL_write((SSL*)ssl_, frame.data() + total_sent, chunk_size);
        if (sent <= 0) {
            std::cerr << "WebSocket send failed at byte " << total_sent << std::endl;
            return false;
        }
        total_sent += sent;
    }
    
    return true;
}

std::string NetworkManager::receive_frame() {
    if (!connected_ || !ssl_) return "";
    
    // Set read timeout to detect silent disconnects (10 seconds)
    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    uint8_t header[2];
    int read_result = SSL_read((SSL*)ssl_, header, 2);
    if (read_result <= 0) {
        int ssl_error = SSL_get_error((SSL*)ssl_, read_result);
        if (ssl_error == SSL_ERROR_WANT_READ || ssl_error == SSL_ERROR_WANT_WRITE) {
            // Timeout or would block - treat as disconnect
            std::cerr << "⚠️  WebSocket read timeout (10s) - connection may be dead" << std::endl;
        }
        return "";
    }
    
    uint64_t payload_len = header[1] & 0x7F;
    if (payload_len == 126) {
        uint8_t len_bytes[2];
        if (SSL_read((SSL*)ssl_, len_bytes, 2) <= 0) return "";
        payload_len = (len_bytes[0] << 8) | len_bytes[1];
    }
    
    // Note: Server-to-Client frames are usually NOT masked
    
    std::vector<uint8_t> buffer(payload_len);
    size_t total_read = 0;
    while (total_read < payload_len) {
        int r = SSL_read((SSL*)ssl_, buffer.data() + total_read, payload_len - total_read);
        if (r <= 0) return "";
        total_read += r;
    }
    
    return std::string(buffer.begin(), buffer.end());
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
    const int max_failures = 2;  // Faster detection: 2 failures × 2s = 4s max
    int reconnect_attempts = 0;
    const int max_reconnect_attempts = 3;
    
    while (wifi_monitor_running_) {
        // Check WiFi every 2 seconds for faster detection
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        if (!is_connected_to_internet()) {
            consecutive_failures++;
            std::cerr << "⚠️  WiFi check failed (" << consecutive_failures << "/" << max_failures << ")" << std::endl;
            
            if (consecutive_failures >= max_failures) {
                std::cerr << "🔄 WiFi lost! Attempting reconnection..." << std::endl;
                
                // Try to reconnect using stored credentials
                if (!last_ssid_.empty() && reconnect_attempts < max_reconnect_attempts) {
                    reconnect_attempts++;
                    std::cout << "📡 Reconnect attempt " << reconnect_attempts << "/" << max_reconnect_attempts << std::endl;
                    
                    std::string cmd = "nmcli device wifi connect '" + last_ssid_ + "' password '" + last_password_ + "' 2>/dev/null";
                    if (std::system(cmd.c_str()) == 0) {
                        std::cout << "✅ WiFi reconnected successfully!" << std::endl;
                        consecutive_failures = 0;
                        reconnect_attempts = 0;
                    } else {
                        std::cerr << "❌ WiFi reconnection attempt " << reconnect_attempts << " failed" << std::endl;
                        // Exponential backoff: 2s, 4s, 8s
                        std::this_thread::sleep_for(std::chrono::seconds(2 * reconnect_attempts));
                    }
                } else if (reconnect_attempts >= max_reconnect_attempts) {
                    // Give up on auto-reconnect, wait for user to scan WiFi QR
                    std::cerr << "❌ Auto-reconnect failed after " << max_reconnect_attempts << " attempts" << std::endl;
                    std::cerr << "📱 Please scan WiFi QR to reconnect" << std::endl;
                    // Don't keep spamming reconnect - wait longer
                    std::this_thread::sleep_for(std::chrono::seconds(10));
                }
            }
        } else {
            // Reset counters on successful check
            if (consecutive_failures > 0) {
                std::cout << "✅ WiFi connection restored" << std::endl;
            }
            consecutive_failures = 0;
            reconnect_attempts = 0;
        }
    }
}

} // namespace mdai