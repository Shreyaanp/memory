#include "CryptoUtils.hpp"
#include <openssl/sha.h>
#include <openssl/bio.h>
#include <openssl/evp.h>
#include <openssl/buffer.h>
#include <openssl/rsa.h>
#include <openssl/pem.h>
#include <openssl/err.h>
#include <nlohmann/json.hpp>
#include <curl/curl.h>
#include <sys/stat.h>
#include <vector>
#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

namespace mdai {

std::string CryptoUtils::sha1(const std::string& input) {
    unsigned char hash[SHA_DIGEST_LENGTH];
    SHA1(reinterpret_cast<const unsigned char*>(input.c_str()), input.length(), hash);
    
    // Convert to string (binary)
    return std::string(reinterpret_cast<char*>(hash), SHA_DIGEST_LENGTH);
}

std::string CryptoUtils::base64_encode(const std::vector<uint8_t>& input) {
    BIO *bio, *b64;
    BUF_MEM *bufferPtr;

    b64 = BIO_new(BIO_f_base64());
    bio = BIO_new(BIO_s_mem());
    bio = BIO_push(b64, bio);

    BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL); // No newlines
    BIO_write(bio, input.data(), input.size());
    BIO_flush(bio);
    BIO_get_mem_ptr(bio, &bufferPtr);

    std::string result(bufferPtr->data, bufferPtr->length);
    BIO_free_all(bio);

    return result;
}

std::string CryptoUtils::base64_encode(const std::string& input) {
    std::vector<uint8_t> vec(input.begin(), input.end());
    return base64_encode(vec);
}

std::pair<std::string, std::string> CryptoUtils::generate_rsa_key_pair(int bits) {
    EVP_PKEY* pkey = EVP_PKEY_new();
    EVP_PKEY_CTX* ctx = EVP_PKEY_CTX_new_id(EVP_PKEY_RSA, NULL);
    
    EVP_PKEY_keygen_init(ctx);
    EVP_PKEY_CTX_set_rsa_keygen_bits(ctx, bits);
    EVP_PKEY_keygen(ctx, &pkey);

    // Extract Public Key
    BIO* pub_bio = BIO_new(BIO_s_mem());
    PEM_write_bio_PUBKEY(pub_bio, pkey);
    
    char* pub_data;
    long pub_len = BIO_get_mem_data(pub_bio, &pub_data);
    std::string pub_key(pub_data, pub_len);
    BIO_free(pub_bio);

    // Extract Private Key
    BIO* priv_bio = BIO_new(BIO_s_mem());
    PEM_write_bio_PrivateKey(priv_bio, pkey, NULL, NULL, 0, NULL, NULL);
    
    char* priv_data;
    long priv_len = BIO_get_mem_data(priv_bio, &priv_data);
    std::string priv_key(priv_data, priv_len);
    BIO_free(priv_bio);

    EVP_PKEY_CTX_free(ctx);
    EVP_PKEY_free(pkey);

    return {pub_key, priv_key};
}

std::string CryptoUtils::rsa_decrypt(const std::string& encrypted_data, const std::string& private_key_pem) {
    BIO* bio = BIO_new_mem_buf(private_key_pem.c_str(), -1);
    EVP_PKEY* pkey = PEM_read_bio_PrivateKey(bio, NULL, NULL, NULL);
    BIO_free(bio);

    if (!pkey) return "";

    EVP_PKEY_CTX* ctx = EVP_PKEY_CTX_new(pkey, NULL);
    EVP_PKEY_decrypt_init(ctx);
    EVP_PKEY_CTX_set_rsa_padding(ctx, RSA_PKCS1_OAEP_PADDING);

    size_t outlen;
    EVP_PKEY_decrypt(ctx, NULL, &outlen, 
                     reinterpret_cast<const unsigned char*>(encrypted_data.c_str()), 
                     encrypted_data.length());

    std::vector<unsigned char> out(outlen);
    if (EVP_PKEY_decrypt(ctx, out.data(), &outlen, 
                         reinterpret_cast<const unsigned char*>(encrypted_data.c_str()), 
                         encrypted_data.length()) <= 0) {
        EVP_PKEY_CTX_free(ctx);
        EVP_PKEY_free(pkey);
        return "";
    }

    EVP_PKEY_CTX_free(ctx);
    EVP_PKEY_free(pkey);

    return std::string(out.begin(), out.begin() + outlen);
}

bool CryptoUtils::save_key_to_file(const std::string& filename, const std::string& key_pem) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    file << key_pem;
    return true;
}

std::string CryptoUtils::load_key_from_file(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) return "";
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// ============================================================
// NEW: Device Key Derivation & AES-256 Implementation
// ============================================================

#include <openssl/hmac.h>
#include <openssl/aes.h>
#include <openssl/rand.h>
#include <openssl/sha.h>

std::string CryptoUtils::get_hardware_id() {
    // Read MAC address from wlan0
    std::ifstream mac_file("/sys/class/net/wlan0/address");
    if (mac_file.is_open()) {
        std::string mac;
        std::getline(mac_file, mac);
        mac_file.close();
        
        // Remove newline if present
        if (!mac.empty() && mac[mac.length()-1] == '\n') {
            mac.erase(mac.length()-1);
        }
        
        return mac;
    }
    
    // Fallback: Try eth0
    std::ifstream eth_file("/sys/class/net/eth0/address");
    if (eth_file.is_open()) {
        std::string mac;
        std::getline(eth_file, mac);
        eth_file.close();
        
        if (!mac.empty() && mac[mac.length()-1] == '\n') {
            mac.erase(mac.length()-1);
        }
        
        return mac;
    }
    
    // Fallback: Use hostname
    std::ifstream hostname_file("/etc/hostname");
    if (hostname_file.is_open()) {
        std::string hostname;
        std::getline(hostname_file, hostname);
        hostname_file.close();
        return hostname;
    }
    
    return "unknown-device";
}

std::string CryptoUtils::hmac_sha256(const std::string& data, const std::string& key) {
    unsigned char hash[EVP_MAX_MD_SIZE];
    unsigned int hash_len;
    
    HMAC(EVP_sha256(), 
         key.c_str(), key.length(),
         reinterpret_cast<const unsigned char*>(data.c_str()), data.length(),
         hash, &hash_len);
    
    return bytes_to_hex(std::vector<uint8_t>(hash, hash + hash_len));
}

std::string CryptoUtils::sha256(const std::string& input) {
    unsigned char hash[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char*>(input.c_str()), 
           input.length(), hash);
    
    return bytes_to_hex(std::vector<uint8_t>(hash, hash + SHA256_DIGEST_LENGTH));
}

std::string CryptoUtils::derive_device_key(const std::string& master_secret, 
                                            const std::string& hardware_id) {
    // HMAC-SHA256(master_secret, hardware_id)
    return hmac_sha256(hardware_id, master_secret);
}

std::string CryptoUtils::get_device_key() {
    std::string hardware_id = get_hardware_id();
    return derive_device_key(MASTER_SECRET, hardware_id);
}

std::vector<uint8_t> CryptoUtils::base64_decode(const std::string& input) {
    BIO *bio, *b64;
    
    b64 = BIO_new(BIO_f_base64());
    bio = BIO_new_mem_buf(input.c_str(), input.length());
    bio = BIO_push(b64, bio);
    
    BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL);
    
    std::vector<uint8_t> output(input.length());
    int decoded_size = BIO_read(bio, output.data(), input.length());
    
    BIO_free_all(bio);
    
    if (decoded_size > 0) {
        output.resize(decoded_size);
    } else {
        output.clear();
    }
    
    return output;
}

std::vector<uint8_t> CryptoUtils::hex_to_bytes(const std::string& hex) {
    std::vector<uint8_t> bytes;
    
    for (size_t i = 0; i < hex.length(); i += 2) {
        std::string byte_str = hex.substr(i, 2);
        uint8_t byte = static_cast<uint8_t>(std::strtol(byte_str.c_str(), nullptr, 16));
        bytes.push_back(byte);
    }
    
    return bytes;
}

std::string CryptoUtils::bytes_to_hex(const std::vector<uint8_t>& bytes) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    
    for (uint8_t byte : bytes) {
        ss << std::setw(2) << static_cast<int>(byte);
    }
    
    return ss.str();
}

std::string CryptoUtils::aes256_encrypt(const std::string& plaintext, const std::string& key_hex) {
    // Convert hex key to bytes
    std::vector<uint8_t> key_bytes = hex_to_bytes(key_hex);
    
    // Ensure key is 32 bytes (256 bits)
    if (key_bytes.size() != 32) {
        // Hash the key if wrong size
        std::string hashed_key = sha256(key_hex);
        key_bytes = hex_to_bytes(hashed_key);
    }
    
    // Generate random IV (16 bytes for AES)
    unsigned char iv[AES_BLOCK_SIZE];
    if (RAND_bytes(iv, AES_BLOCK_SIZE) != 1) {
        throw std::runtime_error("Failed to generate IV");
    }
    
    // Setup encryption
    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    if (!ctx) {
        throw std::runtime_error("Failed to create cipher context");
    }
    
    if (EVP_EncryptInit_ex(ctx, EVP_aes_256_cbc(), nullptr, key_bytes.data(), iv) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("Failed to initialize encryption");
    }
    
    // Encrypt
    std::vector<unsigned char> ciphertext(plaintext.size() + AES_BLOCK_SIZE);
    int len;
    
    if (EVP_EncryptUpdate(ctx, ciphertext.data(), &len, 
                          reinterpret_cast<const unsigned char*>(plaintext.c_str()), 
                          plaintext.size()) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("Encryption failed");
    }
    
    int ciphertext_len = len;
    
    if (EVP_EncryptFinal_ex(ctx, ciphertext.data() + len, &len) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("Encryption finalization failed");
    }
    
    ciphertext_len += len;
    ciphertext.resize(ciphertext_len);
    
    EVP_CIPHER_CTX_free(ctx);
    
    // Prepend IV to ciphertext
    std::vector<uint8_t> result(iv, iv + AES_BLOCK_SIZE);
    result.insert(result.end(), ciphertext.begin(), ciphertext.end());
    
    // Return as base64
    return base64_encode(result);
}

std::string CryptoUtils::aes256_decrypt(const std::string& ciphertext_base64, const std::string& key_hex) {
    // Decode base64
    std::vector<uint8_t> encrypted_data = base64_decode(ciphertext_base64);
    
    if (encrypted_data.size() < AES_BLOCK_SIZE) {
        throw std::runtime_error("Invalid ciphertext: too short");
    }
    
    // Extract IV (first 16 bytes)
    std::vector<uint8_t> iv(encrypted_data.begin(), encrypted_data.begin() + AES_BLOCK_SIZE);
    std::vector<uint8_t> ciphertext(encrypted_data.begin() + AES_BLOCK_SIZE, encrypted_data.end());
    
    // Convert hex key to bytes
    std::vector<uint8_t> key_bytes = hex_to_bytes(key_hex);
    
    // Ensure key is 32 bytes
    if (key_bytes.size() != 32) {
        std::string hashed_key = sha256(key_hex);
        key_bytes = hex_to_bytes(hashed_key);
    }
    
    // Setup decryption
    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    if (!ctx) {
        throw std::runtime_error("Failed to create cipher context");
    }
    
    if (EVP_DecryptInit_ex(ctx, EVP_aes_256_cbc(), nullptr, key_bytes.data(), iv.data()) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("Failed to initialize decryption");
    }
    
    // Decrypt
    std::vector<unsigned char> plaintext(ciphertext.size() + AES_BLOCK_SIZE);
    int len;
    
    if (EVP_DecryptUpdate(ctx, plaintext.data(), &len, ciphertext.data(), ciphertext.size()) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("Decryption failed");
    }
    
    int plaintext_len = len;
    
    if (EVP_DecryptFinal_ex(ctx, plaintext.data() + len, &len) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("Decryption finalization failed");
    }
    
    plaintext_len += len;
    plaintext.resize(plaintext_len);
    
    EVP_CIPHER_CTX_free(ctx);
    
    return std::string(reinterpret_cast<char*>(plaintext.data()), plaintext_len);
}

// ============================================================
// Device Registration & Challenge-Response
// ============================================================

std::string CryptoUtils::get_device_id() {
    // Use hardware ID as device ID
    std::string hw_id = get_hardware_id();
    
    // Format: MDAI-<first 12 chars of MAC>
    std::string clean_mac = hw_id;
    // Remove colons
    clean_mac.erase(std::remove(clean_mac.begin(), clean_mac.end(), ':'), clean_mac.end());
    
    return "MDAI-" + clean_mac;
}

std::pair<std::string, std::string> CryptoUtils::get_or_create_device_keys(const std::string& key_path) {
    std::string public_key_file = key_path + "device_public.pem";
    std::string private_key_file = key_path + "device_private.pem";
    
    // Check if keys already exist
    struct stat buffer;
    bool pub_exists = (stat(public_key_file.c_str(), &buffer) == 0);
    bool priv_exists = (stat(private_key_file.c_str(), &buffer) == 0);
    
    if (pub_exists && priv_exists) {
        // Load existing keys
        std::string public_key = load_key_from_file(public_key_file);
        std::string private_key = load_key_from_file(private_key_file);
        
        if (!public_key.empty() && !private_key.empty()) {
            std::cout << "[CryptoUtils] Loaded existing device keys" << std::endl;
            return {public_key, private_key};
        }
    }
    
    // Generate new keys
    std::cout << "[CryptoUtils] Generating new RSA key pair..." << std::endl;
    auto keys = generate_rsa_key_pair(2048);
    
    // Create directory if it doesn't exist
    std::string mkdir_cmd = "mkdir -p " + key_path;
    system(mkdir_cmd.c_str());
    
    // Save keys
    save_key_to_file(public_key_file, keys.first);
    save_key_to_file(private_key_file, keys.second);
    
    // Set permissions (private key should be read-only by owner)
    chmod(private_key_file.c_str(), 0600);
    chmod(public_key_file.c_str(), 0644);
    
    std::cout << "[CryptoUtils] Device keys generated and saved" << std::endl;
    
    return keys;
}

// Callback for CURL to receive response
static size_t curl_write_callback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    size_t total_size = size * nmemb;
    userp->append(static_cast<char*>(contents), total_size);
    return total_size;
}

bool CryptoUtils::register_device_with_server(const std::string& server_url,
                                                const std::string& device_id,
                                                const std::string& public_key_pem) {
    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "[CryptoUtils] Failed to initialize CURL" << std::endl;
        return false;
    }
    
    // Prepare JSON payload
    std::string json_payload = "{"
        "\"device_id\":\"" + device_id + "\","
        "\"hardware_id\":\"" + get_hardware_id() + "\","
        "\"public_key\":\"" + public_key_pem + "\","
        "\"timestamp\":\"" + std::to_string(time(nullptr)) + "\""
        "}";
    
    // Replace newlines in JSON
    size_t pos = 0;
    while ((pos = json_payload.find("\n", pos)) != std::string::npos) {
        json_payload.replace(pos, 1, "\\n");
        pos += 2;
    }
    
    std::string response;
    std::string url = server_url + "/api/register-device";
    
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_payload.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
    
    // Set headers
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    
    CURLcode res = curl_easy_perform(curl);
    
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    
    if (res != CURLE_OK) {
        std::cerr << "[CryptoUtils] Registration failed: " << curl_easy_strerror(res) << std::endl;
        return false;
    }
    
    std::cout << "[CryptoUtils] Device registered successfully" << std::endl;
    std::cout << "[CryptoUtils] Response: " << response << std::endl;
    
    return true;
}

// ============================================================
// QR Code Validation & Challenge Decryption
// ============================================================

std::string CryptoUtils::decrypt_challenge(const std::string& encrypted_challenge_b64,
                                            const std::string& private_key_pem) {
    // Decode base64
    std::vector<uint8_t> encrypted_data = base64_decode(encrypted_challenge_b64);
    
    // Load private key
    BIO* bio = BIO_new_mem_buf(private_key_pem.c_str(), -1);
    if (!bio) {
        std::cerr << "[CryptoUtils] Failed to create BIO for private key" << std::endl;
        return "";
    }
    
    EVP_PKEY* pkey = PEM_read_bio_PrivateKey(bio, nullptr, nullptr, nullptr);
    BIO_free(bio);
    
    if (!pkey) {
        std::cerr << "[CryptoUtils] Failed to load private key" << std::endl;
        ERR_print_errors_fp(stderr);
        return "";
    }
    
    // Create decryption context
    EVP_PKEY_CTX* ctx = EVP_PKEY_CTX_new(pkey, nullptr);
    if (!ctx) {
        std::cerr << "[CryptoUtils] Failed to create decryption context" << std::endl;
        EVP_PKEY_free(pkey);
        return "";
    }
    
    if (EVP_PKEY_decrypt_init(ctx) <= 0) {
        std::cerr << "[CryptoUtils] Failed to initialize decryption" << std::endl;
        EVP_PKEY_CTX_free(ctx);
        EVP_PKEY_free(pkey);
        return "";
    }
    
    // Set padding (must match server: OAEP with SHA-256)
    if (EVP_PKEY_CTX_set_rsa_padding(ctx, RSA_PKCS1_OAEP_PADDING) <= 0) {
        std::cerr << "[CryptoUtils] Failed to set padding" << std::endl;
        EVP_PKEY_CTX_free(ctx);
        EVP_PKEY_free(pkey);
        return "";
    }
    
    if (EVP_PKEY_CTX_set_rsa_oaep_md(ctx, EVP_sha256()) <= 0) {
        std::cerr << "[CryptoUtils] Failed to set OAEP hash" << std::endl;
        EVP_PKEY_CTX_free(ctx);
        EVP_PKEY_free(pkey);
        return "";
    }
    
    if (EVP_PKEY_CTX_set_rsa_mgf1_md(ctx, EVP_sha256()) <= 0) {
        std::cerr << "[CryptoUtils] Failed to set MGF1 hash" << std::endl;
        EVP_PKEY_CTX_free(ctx);
        EVP_PKEY_free(pkey);
        return "";
    }
    
    // Determine buffer size
    size_t decrypted_len = 0;
    if (EVP_PKEY_decrypt(ctx, nullptr, &decrypted_len, encrypted_data.data(), encrypted_data.size()) <= 0) {
        std::cerr << "[CryptoUtils] Failed to determine decrypted size" << std::endl;
        ERR_print_errors_fp(stderr);
        EVP_PKEY_CTX_free(ctx);
        EVP_PKEY_free(pkey);
        return "";
    }
    
    // Decrypt
    std::vector<uint8_t> decrypted(decrypted_len);
    if (EVP_PKEY_decrypt(ctx, decrypted.data(), &decrypted_len, encrypted_data.data(), encrypted_data.size()) <= 0) {
        std::cerr << "[CryptoUtils] Failed to decrypt challenge" << std::endl;
        ERR_print_errors_fp(stderr);
        EVP_PKEY_CTX_free(ctx);
        EVP_PKEY_free(pkey);
        return "";
    }
    
    EVP_PKEY_CTX_free(ctx);
    EVP_PKEY_free(pkey);
    
    // Resize to actual decrypted size
    decrypted.resize(decrypted_len);
    
    // Encode to base64 for transmission
    std::string decrypted_b64 = base64_encode(decrypted);
    
    std::cout << "[CryptoUtils] Challenge decrypted successfully (" << decrypted_len << " bytes)" << std::endl;
    
    return decrypted_b64;
}

std::tuple<bool, std::string, std::string, std::string> CryptoUtils::validate_and_decrypt_qr(
    const std::string& qr_data,
    const std::string& private_key_pem) {
    
    try {
        // Parse JSON from QR code
        nlohmann::json qr_json = nlohmann::json::parse(qr_data);
        
        // Extract fields
        if (!qr_json.contains("bson_id") || !qr_json.contains("challenge") || !qr_json.contains("type")) {
            std::cerr << "[CryptoUtils] Invalid QR format: missing required fields" << std::endl;
            return {false, "", "", ""};
        }
        
        std::string bson_id = qr_json["bson_id"];
        std::string encrypted_challenge = qr_json["challenge"];
        std::string session_type = qr_json["type"];
        
        std::cout << "[CryptoUtils] QR validated: BSON ID=" << bson_id << ", Type=" << session_type << std::endl;
        
        // Decrypt challenge
        std::string decrypted_challenge = decrypt_challenge(encrypted_challenge, private_key_pem);
        
        if (decrypted_challenge.empty()) {
            std::cerr << "[CryptoUtils] Failed to decrypt challenge" << std::endl;
            return {false, "", "", ""};
        }
        
        std::cout << "[CryptoUtils] QR decryption successful" << std::endl;
        
        return {true, bson_id, decrypted_challenge, session_type};
        
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "[CryptoUtils] JSON parse error: " << e.what() << std::endl;
        return {false, "", "", ""};
    } catch (const std::exception& e) {
        std::cerr << "[CryptoUtils] Error: " << e.what() << std::endl;
        return {false, "", "", ""};
    }
}

std::tuple<bool, std::string, std::string, std::string> CryptoUtils::validate_challenge_with_server(
    const std::string& server_url,
    const std::string& bson_id,
    const std::string& challenge_response_b64) {
    
    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "[CryptoUtils] Failed to initialize CURL" << std::endl;
        return {false, "", "", ""};
    }
    
    // Prepare JSON payload
    nlohmann::json payload = {
        {"bson_id", bson_id},
        {"challenge_response", challenge_response_b64}
    };
    
    std::string json_payload = payload.dump();
    std::string response;
    std::string url = server_url + "/api/validate-challenge";
    
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_payload.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
    
    // Set headers
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    
    CURLcode res = curl_easy_perform(curl);
    
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    
    if (res != CURLE_OK) {
        std::cerr << "[CryptoUtils] Server request failed: " << curl_easy_strerror(res) << std::endl;
        return {false, "", "", ""};
    }
    
    // Parse response
    try {
        nlohmann::json response_json = nlohmann::json::parse(response);
        
        if (!response_json["success"].get<bool>()) {
            std::cerr << "[CryptoUtils] Server rejected challenge: " 
                      << response_json.value("error", "Unknown error") << std::endl;
            return {false, "", "", ""};
        }
        
        std::string session_type = response_json.value("session_type", "unknown");
        
        if (session_type == "wifi") {
            // Extract WiFi credentials
            if (response_json.contains("wifi_credentials")) {
                std::string ssid = response_json["wifi_credentials"]["ssid"];
                std::string password = response_json["wifi_credentials"]["password"];
                
                std::cout << "[CryptoUtils] WiFi credentials received: SSID=" << ssid << std::endl;
                
                return {true, ssid, password, session_type};
            } else {
                std::cerr << "[CryptoUtils] No WiFi credentials in response" << std::endl;
                return {false, "", "", ""};
            }
        } else if (session_type == "websocket") {
            // Extract WebSocket session info
            if (response_json.contains("websocket_session")) {
                std::string ws_session_id = response_json["websocket_session"]["session_id"];
                int expires_in = response_json["websocket_session"]["expires_in"];
                
                std::cout << "[CryptoUtils] WebSocket session created: " << ws_session_id 
                          << " (expires in " << expires_in << "s)" << std::endl;
                
                return {true, ws_session_id, std::to_string(expires_in), session_type};
            } else {
                std::cerr << "[CryptoUtils] No WebSocket session in response" << std::endl;
                return {false, "", "", ""};
            }
        } else {
            std::cerr << "[CryptoUtils] Unknown session type: " << session_type << std::endl;
            return {false, "", "", ""};
        }
        
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "[CryptoUtils] Failed to parse server response: " << e.what() << std::endl;
        std::cerr << "[CryptoUtils] Response was: " << response << std::endl;
        return {false, "", "", ""};
    }
}

} // namespace mdai

