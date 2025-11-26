#include "CameraInitializer.hpp"
#include <iostream>
#include <thread>
#include <utility>

namespace mdai {

CameraInitializer::CameraInitializer(const rs2::config& config,
                                     std::string device_serial)
    : config_(config),
      locked_serial_(std::move(device_serial)),
      total_phases_(5) {
}

CameraInitializer::~CameraInitializer() {
    // Note: Pipeline may have been moved to Producer, so don't try to stop it here
    // The Producer is responsible for stopping the pipeline
}

void CameraInitializer::set_status_callback(StatusCallback callback) {
    status_callback_ = callback;
}

void CameraInitializer::set_progress_callback(ProgressCallback callback) {
    progress_callback_ = callback;
}

CameraInitializer::Result CameraInitializer::initialize(int timeout_ms) {
    if (initialized_.load()) {
        report_status("Camera already initialized");
        return Result::SUCCESS;
    }
    
    cancelled_.store(false);
    start_time_ = std::chrono::steady_clock::now();
    
    // Set timeout (default to 30 seconds if not specified)
    int effective_timeout = (timeout_ms > 0) ? timeout_ms : 30000;
    report_status("Initialization timeout: " + std::to_string(effective_timeout) + "ms");
    
    try {
        // Phase 1: Connecting
        current_phase_.store(Phase::CONNECTING);
        report_status("Connecting to camera...");
        
        // Check timeout before starting
        if (get_elapsed_time_ms() > effective_timeout) {
            report_status("ERROR: Initialization timeout during connection phase");
            return Result::TIMEOUT;
        }
        report_progress(0);
        
        // Validate configuration without starting pipeline
        // The pipeline will be started by the Producer after initialization
        report_status("Camera configuration validated successfully");
        report_progress(20);
        
        if (is_cancelled()) return Result::CANCELLED;
        if (get_elapsed_time_ms() > effective_timeout) {
            report_status("ERROR: Initialization timeout during sensor warm-up phase");
            return Result::TIMEOUT;
        }
        
        // Phase 2: Sensor Warm-up
        current_phase_.store(Phase::SENSOR_WARMUP);
        if (!execute_sensor_warmup()) {
            return is_cancelled() ? Result::CANCELLED : Result::SENSOR_ERROR;
        }
        report_progress(40);
        
        if (is_cancelled()) return Result::CANCELLED;
        if (get_elapsed_time_ms() > effective_timeout) {
            report_status("ERROR: Initialization timeout during autofocus stabilization phase");
            return Result::TIMEOUT;
        }
        
        // Phase 3: Autofocus Stabilization
        current_phase_.store(Phase::AUTOFOCUS_STABILIZATION);
        if (!execute_autofocus_stabilization()) {
            return is_cancelled() ? Result::CANCELLED : Result::SENSOR_ERROR;
        }
        report_progress(60);
        
        if (is_cancelled()) return Result::CANCELLED;
        if (get_elapsed_time_ms() > effective_timeout) {
            report_status("ERROR: Initialization timeout during exposure stabilization phase");
            return Result::TIMEOUT;
        }
        
        // Phase 4: Exposure Stabilization
        current_phase_.store(Phase::EXPOSURE_STABILIZATION);
        if (!execute_exposure_stabilization()) {
            return is_cancelled() ? Result::CANCELLED : Result::SENSOR_ERROR;
        }
        report_progress(80);
        
        if (is_cancelled()) return Result::CANCELLED;
        if (get_elapsed_time_ms() > effective_timeout) {
            report_status("ERROR: Initialization timeout during final calibration phase");
            return Result::TIMEOUT;
        }
        
        // Phase 5: Final Calibration
        current_phase_.store(Phase::FINAL_CALIBRATION);
        if (!execute_final_calibration()) {
            return is_cancelled() ? Result::CANCELLED : Result::SENSOR_ERROR;
        }
        report_progress(100);
        
        // Mark as ready
        current_phase_.store(Phase::READY);
        initialized_.store(true);
        report_status("Camera initialization complete - ready for capture");
        
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

int CameraInitializer::get_progress() const {
    if (initialized_.load()) return 100;
    
    int base_progress = 0;
    switch (current_phase_.load()) {
        case Phase::CONNECTING:
            base_progress = 0;
            break;
        case Phase::SENSOR_WARMUP:
            base_progress = 20;
            break;
        case Phase::AUTOFOCUS_STABILIZATION:
            base_progress = 40;
            break;
        case Phase::EXPOSURE_STABILIZATION:
            base_progress = 60;
            break;
        case Phase::FINAL_CALIBRATION:
            base_progress = 80;
            break;
        case Phase::READY:
            base_progress = 100;
            break;
    }
    
    // Add sub-progress based on elapsed time within current phase
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time_).count();
    
    int phase_progress = 0;
    switch (current_phase_.load()) {
        case Phase::SENSOR_WARMUP:
            phase_progress = std::min(20, (int)(elapsed * 20 / SENSOR_WARMUP_TIME));
            break;
        case Phase::AUTOFOCUS_STABILIZATION:
            phase_progress = std::min(20, (int)(elapsed * 20 / AUTOFOCUS_STABILIZATION_TIME));
            break;
        case Phase::EXPOSURE_STABILIZATION:
            phase_progress = std::min(20, (int)(elapsed * 20 / EXPOSURE_STABILIZATION_TIME));
            break;
        case Phase::FINAL_CALIBRATION:
            phase_progress = std::min(20, (int)(elapsed * 20 / FINAL_CALIBRATION_TIME));
            break;
        default:
            phase_progress = 0;
            break;
    }
    
    return base_progress + phase_progress;
}

int CameraInitializer::get_elapsed_time_ms() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time_).count();
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

rs2::device CameraInitializer::get_locked_device(const std::string& phase) {
    rs2::context ctx;
    auto devices = ctx.query_devices();
    
    if (devices.size() == 0) {
        report_status("No devices detected for " + phase);
        return rs2::device();
    }
    
    // If we already have a locked serial, try to find that exact device first.
    if (!locked_serial_.empty()) {
        for (auto&& dev : devices) {
            try {
                const char* serial_cstr = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                if (serial_cstr && locked_serial_ == serial_cstr) {
                    return dev;
                }
            } catch (const rs2::error&) {
                // Ignore and continue searching
            }
        }
        report_status("Serial " + locked_serial_ + " not found during " + phase +
                      " - defaulting to first detected device");
    }
    
    rs2::device selected = devices[0];
    try {
        const char* serial_cstr = selected.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        if (serial_cstr) {
            std::string serial(serial_cstr);
            if (locked_serial_.empty()) {
                locked_serial_ = serial;
                // Hardcode the serial because this setup uses a single fixed RealSense camera.
                report_status("Locking to camera serial: " + locked_serial_);
                report_status("Serial hardcoded for fixed camera setup; update if hardware changes.");
            } else if (locked_serial_ != serial) {
                report_status("Updating hardcoded serial from " + locked_serial_ +
                              " to " + serial);
                locked_serial_ = serial;
            }
        }
    } catch (const rs2::error& e) {
        report_status("Warning: unable to read device serial during " + phase +
                      ": " + std::string(e.what()));
    }
    
    return selected;
}

bool CameraInitializer::execute_sensor_warmup() {
    int warmup_time = quick_mode_ ? QUICK_SENSOR_WARMUP_TIME : SENSOR_WARMUP_TIME;
    report_status(quick_mode_ ? "Sensor warm-up phase (0.5 seconds - quick mode)..." 
                              : "Sensor warm-up phase (2 seconds)...");
    return wait_with_progress(warmup_time, "Sensor warm-up");
}

bool CameraInitializer::execute_autofocus_stabilization() {
    int af_time = quick_mode_ ? QUICK_AUTOFOCUS_TIME : AUTOFOCUS_STABILIZATION_TIME;
    report_status(quick_mode_ ? "Autofocus stabilization phase (0.5 seconds - quick mode)..." 
                              : "Autofocus stabilization phase (3 seconds)...");
    
    try {
        rs2::device device = get_locked_device("autofocus stabilization");
        if (!device) {
            report_status("No devices found for autofocus stabilization");
            return wait_with_progress(af_time, "Autofocus stabilization");
        }
        auto sensors = device.query_sensors();
        
        // Enable autofocus if available
        for (auto& sensor : sensors) {
            if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
                report_status("Auto-exposure enabled");
            }
        }
        
        // Wait for stabilization
        return wait_with_progress(af_time, "Autofocus stabilization");
        
    } catch (const rs2::error& e) {
        report_status("Autofocus setup warning: " + std::string(e.what()));
        // Continue with time-based stabilization even if autofocus setup fails
        return wait_with_progress(af_time, "Autofocus stabilization");
    }
}

bool CameraInitializer::execute_exposure_stabilization() {
    int exp_time = quick_mode_ ? QUICK_EXPOSURE_TIME : EXPOSURE_STABILIZATION_TIME;
    report_status(quick_mode_ ? "Exposure stabilization phase (0.5 seconds - quick mode)..." 
                              : "Exposure stabilization phase (2 seconds)...");
    
    try {
        rs2::device device = get_locked_device("exposure stabilization");
        if (!device) {
            report_status("No devices found for exposure stabilization");
            return wait_with_progress(exp_time, "Exposure stabilization");
        }
        auto sensors = device.query_sensors();
        
        // Wait for auto-exposure to stabilize first
        wait_with_progress(exp_time, "Exposure stabilization");
        
        // DO NOT LOCK EXPOSURE - keep auto-exposure enabled for adaptive lighting
        // The previous implementation locked exposure which broke QR scanning in varying light
        report_status("Exposure stabilization complete - keeping auto-exposure enabled");
        
        return true;
        
    } catch (const rs2::error& e) {
        report_status("Exposure setup warning: " + std::string(e.what()));
        // Continue anyway - not fatal
        return true;
    }
}

bool CameraInitializer::execute_final_calibration() {
    int cal_time = quick_mode_ ? QUICK_CALIBRATION_TIME : FINAL_CALIBRATION_TIME;
    report_status(quick_mode_ ? "Final calibration phase (0.5 seconds - quick mode)..." 
                              : "Final calibration phase (1 second)...");
    
    try {
        rs2::device device = get_locked_device("final calibration");
        if (!device) {
            report_status("No devices found for final calibration");
            return wait_with_progress(cal_time, "Final calibration");
        }
        
        // Validate device is ready
        report_status("Device ready: " + std::string(device.get_info(RS2_CAMERA_INFO_NAME)));
        
        // Check available sensors
        auto sensors = device.query_sensors();
        report_status("Available sensors: " + std::to_string(sensors.size()));
        
        // Wait for final stabilization
        return wait_with_progress(cal_time, "Final calibration");
        
    } catch (const rs2::error& e) {
        report_status("Final calibration warning: " + std::string(e.what()));
        // Continue with time-based stabilization
        return wait_with_progress(cal_time, "Final calibration");
    }
}

bool CameraInitializer::wait_with_progress(int duration_ms, const std::string& phase_name) {
    auto start = std::chrono::steady_clock::now();
    auto end = start + std::chrono::milliseconds(duration_ms);
    
    while (std::chrono::steady_clock::now() < end) {
        if (is_cancelled()) {
            return false;
        }
        
        // Update progress every 100ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        
        int progress = (elapsed * 100) / duration_ms;
        if (progress % 10 == 0) { // Report every 10%
            report_status(phase_name + " progress: " + std::to_string(progress) + "%");
        }
    }
    
    report_status(phase_name + " complete");
    return true;
}


} // namespace mdai
