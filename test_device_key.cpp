/**
 * Test Device Key Derivation and AES Encryption
 */

#include "CryptoUtils.hpp"
#include <iostream>
#include <iomanip>

using namespace mdai;

int main() {
    std::cout << "====================================" << std::endl;
    std::cout << " Device Key Derivation Test" << std::endl;
    std::cout << "====================================" << std::endl;
    std::cout << std::endl;
    
    // Get hardware ID
    std::string hardware_id = CryptoUtils::get_hardware_id();
    std::cout << "Hardware ID: " << hardware_id << std::endl;
    
    // Derive device key
    std::string device_key = CryptoUtils::get_device_key();
    std::cout << "Device Key:  " << device_key << std::endl;
    std::cout << std::endl;
    
    // Test encryption/decryption
    std::cout << "Testing AES-256 Encryption..." << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    
    std::string plaintext = R"({"type":"wifi_setup","ssid":"TestNetwork","password":"TestPass123","security":"WPA2","timestamp":"2025-11-20T12:00:00Z"})";
    
    std::cout << "Plaintext: " << plaintext << std::endl;
    std::cout << std::endl;
    
    try {
        // Encrypt
        std::string encrypted = CryptoUtils::aes256_encrypt(plaintext, device_key);
        std::cout << "Encrypted (Base64): " << std::endl;
        std::cout << encrypted << std::endl;
        std::cout << std::endl;
        
        // Decrypt
        std::string decrypted = CryptoUtils::aes256_decrypt(encrypted, device_key);
        std::cout << "Decrypted: " << decrypted << std::endl;
        std::cout << std::endl;
        
        // Verify
        if (plaintext == decrypted) {
            std::cout << "✅ SUCCESS: Encryption/Decryption works correctly!" << std::endl;
        } else {
            std::cout << "❌ FAILED: Decrypted text doesn't match original!" << std::endl;
            return 1;
        }
        
    } catch (const std::exception& e) {
        std::cout << "❌ ERROR: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << std::endl;
    std::cout << "====================================" << std::endl;
    std::cout << " 📋 Device Label QR Data" << std::endl;
    std::cout << "====================================" << std::endl;
    std::cout << std::endl;
    std::cout << "Generate device label with:" << std::endl;
    std::cout << "  python3 tools/generate_device_label.py " << hardware_id << std::endl;
    std::cout << std::endl;
    std::cout << "Or scan this JSON as QR:" << std::endl;
    std::cout << "{\"type\":\"device_label\",\"hardware_id\":\"" << hardware_id 
              << "\",\"device_key\":\"" << device_key << "\"}" << std::endl;
    std::cout << std::endl;
    
    return 0;
}


