#pragma once

#include <string>
#include <vector>
#include <utility>
#include <cstdint>

namespace mdai {

class CryptoUtils {
public:
    // WebSocket Handshake Helpers
    static std::string sha1(const std::string& input);
    static std::string base64_encode(const std::vector<uint8_t>& input);
    static std::string base64_encode(const std::string& input);

    // RSA Encryption/Decryption
    // Returns {public_key_pem, private_key_pem}
    static std::pair<std::string, std::string> generate_rsa_key_pair(int bits = 2048);
    
    static std::string rsa_decrypt(const std::string& encrypted_data, const std::string& private_key_pem);
    
    // Helper to load/save keys
    static bool save_key_to_file(const std::string& filename, const std::string& key_pem);
    static std::string load_key_from_file(const std::string& filename);
    
    // ============================================================
    // NEW: Device Key Derivation & AES-256 Encryption
    // ============================================================
    
    /**
     * @brief Get hardware ID (MAC address of wlan0)
     * @return Hardware ID string (e.g., "b4:2f:03:31:9a:35")
     */
    static std::string get_hardware_id();
    
    /**
     * @brief Derive device-specific key from master secret and hardware ID
     * @param master_secret Master secret compiled into firmware
     * @param hardware_id Hardware ID (MAC address)
     * @return 256-bit key as hex string
     */
    static std::string derive_device_key(const std::string& master_secret, 
                                         const std::string& hardware_id);
    
    /**
     * @brief Get this device's derived key using compiled master secret
     * @return Device-specific key
     */
    static std::string get_device_key();
    
    /**
     * @brief AES-256-CBC encryption
     * @param plaintext Data to encrypt
     * @param key 256-bit key (32 bytes or 64 hex chars)
     * @return Base64-encoded ciphertext (includes IV)
     */
    static std::string aes256_encrypt(const std::string& plaintext, const std::string& key);
    
    /**
     * @brief AES-256-CBC decryption
     * @param ciphertext_base64 Base64-encoded ciphertext (includes IV)
     * @param key 256-bit key (32 bytes or 64 hex chars)
     * @return Decrypted plaintext
     */
    static std::string aes256_decrypt(const std::string& ciphertext_base64, const std::string& key);
    
    /**
     * @brief HMAC-SHA256
     * @param data Data to hash
     * @param key HMAC key
     * @return HMAC as hex string
     */
    static std::string hmac_sha256(const std::string& data, const std::string& key);
    
    /**
     * @brief SHA-256 hash
     * @param input Data to hash
     * @return Hash as hex string
     */
    static std::string sha256(const std::string& input);
    
    /**
     * @brief Base64 decode
     * @param input Base64 string
     * @return Decoded bytes
     */
    static std::vector<uint8_t> base64_decode(const std::string& input);
    
    /**
     * @brief Hex to bytes
     * @param hex Hex string
     * @return Bytes
     */
    static std::vector<uint8_t> hex_to_bytes(const std::string& hex);
    
    /**
     * @brief Bytes to hex
     * @param bytes Byte vector
     * @return Hex string
     */
    static std::string bytes_to_hex(const std::vector<uint8_t>& bytes);
    
    // ============================================================
    // Device Registration & Challenge-Response
    // ============================================================
    
    /**
     * @brief Get or generate device ID (unique identifier)
     * @return Device ID (hardware_id based)
     */
    static std::string get_device_id();
    
    /**
     * @brief Register device with server (send public key)
     * @param server_url Server URL (e.g., "https://mdai.mercle.ai")
     * @param device_id Device identifier
     * @param public_key_pem RSA public key in PEM format
     * @return true if registration successful
     */
    static bool register_device_with_server(const std::string& server_url,
                                             const std::string& device_id,
                                             const std::string& public_key_pem);
    
    /**
     * @brief Get or create device RSA key pair
     * @param key_path Path to store keys (default: /etc/mdai/)
     * @return {public_key_pem, private_key_pem}
     */
    static std::pair<std::string, std::string> get_or_create_device_keys(
        const std::string& key_path = "/etc/mdai/");
    
    /**
     * @brief Decrypt challenge with device's private key
     * @param encrypted_challenge_b64 Base64-encoded encrypted challenge
     * @param private_key_pem Private key in PEM format
     * @return Decrypted challenge (base64-encoded for transmission)
     */
    static std::string decrypt_challenge(const std::string& encrypted_challenge_b64,
                                          const std::string& private_key_pem);
    
    /**
     * @brief Validate QR code and extract challenge
     * @param qr_data JSON string from QR code
     * @param private_key_pem Device's private key
     * @return {success, bson_id, decrypted_challenge_b64, session_type}
     */
    static std::tuple<bool, std::string, std::string, std::string> validate_and_decrypt_qr(
        const std::string& qr_data,
        const std::string& private_key_pem);
    
    /**
     * @brief Send challenge response to server and get WiFi credentials
     * @param server_url Server URL (e.g., "https://mdai.mercle.ai")
     * @param bson_id Session ID from QR code
     * @param challenge_response_b64 Base64-encoded decrypted challenge
     * @return {success, wifi_ssid, wifi_password, session_type}
     */
    static std::tuple<bool, std::string, std::string, std::string> validate_challenge_with_server(
        const std::string& server_url,
        const std::string& bson_id,
        const std::string& challenge_response_b64);
};

// Master secret (compiled into firmware)
// TODO: Change this before production deployment!
constexpr const char* MASTER_SECRET = "mdai-master-secret-key-v1-change-in-production";

} // namespace mdai

