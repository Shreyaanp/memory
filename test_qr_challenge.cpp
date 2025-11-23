/**
 * Test QR Code Challenge-Response Flow
 * =====================================
 * This test demonstrates the complete flow:
 * 1. Load device's private key
 * 2. Simulate scanning a QR code (parse JSON)
 * 3. Decrypt the challenge with private key
 * 4. Send challenge response to server
 * 5. Receive WiFi credentials or WebSocket session
 */

#include "CryptoUtils.hpp"
#include <iostream>
#include <fstream>
#include <string>

using namespace mdai;

std::string load_file(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << path << std::endl;
        return "";
    }
    
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    return content;
}

void test_qr_flow() {
    std::cout << "═══════════════════════════════════════════════════════════" << std::endl;
    std::cout << "  QR Code Challenge-Response Flow Test" << std::endl;
    std::cout << "═══════════════════════════════════════════════════════════" << std::endl;
    std::cout << std::endl;
    
    // Step 1: Load device's private key
    std::cout << "[1/4] Loading device private key..." << std::endl;
    std::string private_key = load_file("/etc/mdai/device_private.pem");
    
    if (private_key.empty()) {
        std::cerr << "❌ Failed to load private key from /etc/mdai/device_private.pem" << std::endl;
        std::cerr << "    Run device registration first!" << std::endl;
        return;
    }
    
    std::cout << "✅ Private key loaded (" << private_key.length() << " bytes)" << std::endl;
    std::cout << std::endl;
    
    // Step 2: Get QR data (in real use, this comes from camera/scanner)
    std::cout << "[2/4] Enter QR code JSON data:" << std::endl;
    std::cout << "(Paste the QR payload from admin portal)" << std::endl;
    std::cout << "> ";
    
    std::string qr_data;
    std::getline(std::cin, qr_data);
    
    if (qr_data.empty()) {
        std::cerr << "❌ No QR data provided" << std::endl;
        return;
    }
    
    std::cout << std::endl;
    
    // Step 3: Validate and decrypt QR
    std::cout << "[3/4] Validating and decrypting QR code..." << std::endl;
    
    auto [success, bson_id, decrypted_challenge, session_type] = 
        CryptoUtils::validate_and_decrypt_qr(qr_data, private_key);
    
    if (!success) {
        std::cerr << "❌ QR validation/decryption failed" << std::endl;
        return;
    }
    
    std::cout << "✅ QR decrypted successfully" << std::endl;
    std::cout << "   BSON ID: " << bson_id << std::endl;
    std::cout << "   Session Type: " << session_type << std::endl;
    std::cout << std::endl;
    
    // Step 4: Send challenge response to server
    std::cout << "[4/4] Validating challenge with server..." << std::endl;
    
    std::string server_url = "https://mdai.mercle.ai";
    auto [validated, data1, data2, type] = 
        CryptoUtils::validate_challenge_with_server(server_url, bson_id, decrypted_challenge);
    
    if (!validated) {
        std::cerr << "❌ Server validation failed" << std::endl;
        return;
    }
    
    std::cout << "✅ Server validation successful" << std::endl;
    std::cout << std::endl;
    
    // Display results based on session type
    if (type == "wifi") {
        std::cout << "═══════════════════════════════════════════════════════════" << std::endl;
        std::cout << "  ✅ WiFi Credentials Received" << std::endl;
        std::cout << "═══════════════════════════════════════════════════════════" << std::endl;
        std::cout << std::endl;
        std::cout << "SSID:     " << data1 << std::endl;
        std::cout << "Password: " << data2 << std::endl;
        std::cout << std::endl;
        std::cout << "You can now connect to this WiFi network." << std::endl;
        std::cout << std::endl;
    } else if (type == "websocket") {
        std::cout << "═══════════════════════════════════════════════════════════" << std::endl;
        std::cout << "  ✅ WebSocket Session Created" << std::endl;
        std::cout << "═══════════════════════════════════════════════════════════" << std::endl;
        std::cout << std::endl;
        std::cout << "Session ID:  " << data1 << std::endl;
        std::cout << "Expires In:  " << data2 << " seconds" << std::endl;
        std::cout << std::endl;
        std::cout << "You can now establish a WebSocket connection." << std::endl;
        std::cout << "Connection URL: wss://mdai.mercle.ai/ws/" << std::endl;
        std::cout << std::endl;
    }
}

int main(int argc, char* argv[]) {
    // Option 1: Interactive test
    if (argc == 1) {
        test_qr_flow();
    }
    // Option 2: Command-line test with QR data as argument
    else if (argc == 2) {
        std::cout << "═══════════════════════════════════════════════════════════" << std::endl;
        std::cout << "  QR Code Challenge-Response Flow Test (CLI Mode)" << std::endl;
        std::cout << "═══════════════════════════════════════════════════════════" << std::endl;
        std::cout << std::endl;
        
        // Load private key
        std::string private_key = load_file("/etc/mdai/device_private.pem");
        if (private_key.empty()) {
            std::cerr << "❌ Failed to load private key" << std::endl;
            return 1;
        }
        
        std::string qr_data = argv[1];
        
        // Validate and decrypt
        auto [success, bson_id, decrypted_challenge, session_type] = 
            CryptoUtils::validate_and_decrypt_qr(qr_data, private_key);
        
        if (!success) {
            std::cerr << "❌ QR validation/decryption failed" << std::endl;
            return 1;
        }
        
        // Validate with server
        std::string server_url = "https://mdai.mercle.ai";
        auto [validated, data1, data2, type] = 
            CryptoUtils::validate_challenge_with_server(server_url, bson_id, decrypted_challenge);
        
        if (!validated) {
            std::cerr << "❌ Server validation failed" << std::endl;
            return 1;
        }
        
        // Output results
        if (type == "wifi") {
            std::cout << "✅ WiFi Credentials:" << std::endl;
            std::cout << "SSID=" << data1 << std::endl;
            std::cout << "PASSWORD=" << data2 << std::endl;
        } else if (type == "websocket") {
            std::cout << "✅ WebSocket Session:" << std::endl;
            std::cout << "SESSION_ID=" << data1 << std::endl;
            std::cout << "EXPIRES_IN=" << data2 << std::endl;
        }
    }
    else {
        std::cerr << "Usage:" << std::endl;
        std::cerr << "  Interactive mode: " << argv[0] << std::endl;
        std::cerr << "  CLI mode:         " << argv[0] << " '<qr_json_data>'" << std::endl;
        return 1;
    }
    
    return 0;
}

