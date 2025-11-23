#include "TrustZoneIdentity.hpp"
#include "CryptoUtils.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

namespace mdai {

// Configuration file path for device identity
static const std::string DEVICE_CONFIG_PATH = "/opt/mdai/device_config.json";

bool TrustZoneIdentity::initialize() {
    // TODO: Implement actual TPM/TrustZone initialization
    // For now, just check if device config exists or can be created
    std::cout << "[TrustZoneIdentity] Using stub implementation - TPM integration pending" << std::endl;
    return true;
}

std::string TrustZoneIdentity::get_device_id() {
    // TODO: Get from TPM/hardware
    // For now, read from config file or generate
    std::ifstream config_file(DEVICE_CONFIG_PATH);
    if (config_file.is_open()) {
        std::string line;
        while (std::getline(config_file, line)) {
            if (line.find("\"device_id\"") != std::string::npos) {
                size_t start = line.find(":") + 1;
                size_t first_quote = line.find("\"", start);
                size_t second_quote = line.find("\"", first_quote + 1);
                if (first_quote != std::string::npos && second_quote != std::string::npos) {
                    return line.substr(first_quote + 1, second_quote - first_quote - 1);
                }
            }
        }
    }
    
    // Fallback: generate from MAC address or hardware info
    std::string hardware_serial = get_hardware_serial();
    return "dev_" + CryptoUtils::sha256(hardware_serial).substr(0, 16);
}

std::string TrustZoneIdentity::get_hardware_serial() {
    // TODO: Get from actual hardware serial
    // For now, try to read from various system sources
    
    // Try machine-id first
    std::ifstream machine_id("/etc/machine-id");
    if (machine_id.is_open()) {
        std::string id;
        std::getline(machine_id, id);
        if (!id.empty()) {
            return id;
        }
    }
    
    // Try DMI product UUID
    std::ifstream dmi_uuid("/sys/class/dmi/id/product_uuid");
    if (dmi_uuid.is_open()) {
        std::string uuid;
        std::getline(dmi_uuid, uuid);
        if (!uuid.empty()) {
            return uuid;
        }
    }
    
    // Fallback to a generated value
    std::cerr << "[TrustZoneIdentity] WARNING: Using fallback hardware ID" << std::endl;
    return "hardware_" + std::to_string(std::hash<std::string>{}("mdai_device"));
}

std::string TrustZoneIdentity::export_private_key_for_backup() {
    // TODO: Export from TPM
    // For now, check if key exists in config directory
    std::string key_path = "/opt/mdai/device_key.pem";
    std::ifstream key_file(key_path);
    if (key_file.is_open()) {
        std::stringstream buffer;
        buffer << key_file.rdbuf();
        return buffer.str();
    }
    
    // If no key exists, generate one
    std::cerr << "[TrustZoneIdentity] WARNING: No private key found, generating new one" << std::endl;
    auto [public_key, private_key] = CryptoUtils::get_or_create_device_keys(key_path);
    return private_key;
}

} // namespace mdai

