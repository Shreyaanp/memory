#include "CameraInitializer.hpp"
#include <iostream>
#include <utility>

namespace mdai {

CameraInitializer::CameraInitializer(const rs2::config& config,
                                     std::string device_serial)
    : config_(config),
      locked_serial_(std::move(device_serial)) {
}

void CameraInitializer::set_status_callback(StatusCallback callback) {
    status_callback_ = std::move(callback);
}

void CameraInitializer::set_progress_callback(ProgressCallback callback) {
    progress_callback_ = std::move(callback);
}

CameraInitializer::Result CameraInitializer::initialize(int /*timeout_ms*/) {
    if (initialized_.load()) {
        report_status("Camera already initialized");
        return Result::SUCCESS;
    }
    
    cancelled_.store(false);
    current_phase_.store(Phase::CONNECTING);
    report_status("Detecting RealSense device...");
    report_progress(0);
    
    if (is_cancelled()) return Result::CANCELLED;
    
    try {
        if (!find_and_lock_device()) {
            report_status("ERROR: No RealSense device found");
            return Result::CONNECTION_FAILED;
        }
        
        report_progress(50);
        
        if (is_cancelled()) return Result::CANCELLED;
        
        // Validate the config can resolve with the device
        // (This doesn't start streaming, just checks compatibility)
        if (!locked_serial_.empty()) {
            config_.enable_device(locked_serial_);
        }
        
        report_status("Device validated: " + locked_serial_);
        report_progress(100);
        
        current_phase_.store(Phase::READY);
        initialized_.store(true);
        report_status("Camera ready - Producer will handle sensor stabilization via frame discarding");
        
        return Result::SUCCESS;
        
    } catch (const rs2::error& e) {
        report_status("Camera initialization failed: " + std::string(e.what()));
        return Result::CONNECTION_FAILED;
    }
}

void CameraInitializer::cancel() {
    cancelled_.store(true);
    report_status("Camera initialization cancelled");
}

void CameraInitializer::report_status(const std::string& message) {
    if (status_callback_) {
        status_callback_(message);
    } else {
        std::cout << "[CameraInit] " << message << std::endl;
    }
}

void CameraInitializer::report_progress(int progress) {
    if (progress_callback_) {
        progress_callback_(progress);
    }
}

bool CameraInitializer::find_and_lock_device() {
    rs2::context ctx;
    auto devices = ctx.query_devices();
    
    if (devices.size() == 0) {
        report_status("No RealSense devices detected");
        return false;
    }
    
    report_status("Found " + std::to_string(devices.size()) + " RealSense device(s)");
    
    // If we have a target serial, find that specific device
    if (!locked_serial_.empty()) {
        for (auto&& dev : devices) {
            try {
                const char* serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                if (serial && locked_serial_ == serial) {
                    const char* name = dev.get_info(RS2_CAMERA_INFO_NAME);
                    report_status("Found target device: " + std::string(name ? name : "Unknown") + 
                                  " [" + locked_serial_ + "]");
                    return true;
                }
            } catch (const rs2::error&) {
                // Continue searching
            }
        }
        report_status("WARNING: Target serial " + locked_serial_ + " not found, using first device");
    }
    
    // Use first available device
    rs2::device dev = devices[0];
    try {
        const char* serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        const char* name = dev.get_info(RS2_CAMERA_INFO_NAME);
        
        if (serial) {
            locked_serial_ = serial;
        }
        
        report_status("Using device: " + std::string(name ? name : "Unknown") + 
                      " [" + locked_serial_ + "]");
        return true;
        
    } catch (const rs2::error& e) {
        report_status("Warning: Could not read device info: " + std::string(e.what()));
        return true; // Still usable, just can't get serial
    }
}

} // namespace mdai
