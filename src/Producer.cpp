#include "Producer.hpp"
#include <iostream>
#include <chrono>

namespace mdai {

Producer::Producer(const CameraConfig& config, DynamicRingBuffer* ring_buffer)
    : config_(config), ring_buffer_(ring_buffer) {
    last_fps_calc_time_ = std::chrono::steady_clock::now();
}

Producer::~Producer() {
    stop();
}

bool Producer::start() {
    if (running_.load()) {
        report_error("Producer already running");
        return false;
    }
    
    try {
        if (!configure_pipeline()) {
            return false;
        }
        
        // Create fresh pipeline
        pipe_ = rs2::pipeline();
        
        report_status("Starting IR-only camera pipeline...");
        
        // Start pipeline in polling mode
        profile_ = pipe_.start(rs_config_);
        
        configure_sensor_options();
        query_camera_parameters();
        
        running_.store(true);
        camera_connected_.store(true);
        
        capture_thread_ = std::make_unique<std::thread>(&Producer::capture_loop, this);
        
        // Wait for first frame
        report_status("Waiting for first IR frame...");
        auto start_wait = std::chrono::steady_clock::now();
        const int FIRST_FRAME_TIMEOUT_MS = 5000;
        
        while (!first_frame_received_.load()) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_wait
            ).count();
            
            if (elapsed > FIRST_FRAME_TIMEOUT_MS) {
                report_error("Camera timeout - no IR frames received");
                stop();
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        report_status("IR camera ready - frames flowing");
        return true;
        
    } catch (const rs2::error& e) {
        report_error(std::string("RealSense error: ") + e.what());
        return false;
    } catch (const std::exception& e) {
        report_error(std::string("Error starting producer: ") + e.what());
        return false;
    }
}

void Producer::stop() {
    if (!running_.load()) {
        return;
    }
    
    running_.store(false);
    camera_connected_.store(false);
    
    if (capture_thread_ && capture_thread_->joinable()) {
        capture_thread_->join();
    }
    
    try {
        if (pipe_.get_active_profile()) {
            pipe_.stop();
            report_status("Pipeline stopped");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    } catch (const rs2::error& e) {
        report_error(std::string("Error stopping pipeline: ") + e.what());
    } catch (...) {
        report_error("Unknown error during pipeline stop");
    }
    
    // Reset state for clean restart
    first_frame_received_.store(false);
    rs_config_ = rs2::config();
    frames_since_last_calc_ = 0;
    last_fps_calc_time_ = std::chrono::steady_clock::now();
    current_fps_.store(0.0f);
    
    report_status("Producer stopped");
}

bool Producer::is_running() const {
    return running_.load();
}

const CameraConfig& Producer::get_config() const {
    return config_;
}

float Producer::get_fps() const {
    return current_fps_.load();
}

uint64_t Producer::get_total_frames_captured() const {
    return total_frames_captured_.load();
}

bool Producer::is_camera_connected() const {
    return camera_connected_.load();
}

bool Producer::is_camera_healthy(int timeout_ms) const {
    if (!running_.load() || !camera_connected_.load() || !first_frame_received_.load()) {
        return false;
    }
    
    int64_t ms_since_last = get_ms_since_last_frame();
    return ms_since_last >= 0 && ms_since_last <= timeout_ms;
}

int64_t Producer::get_ms_since_last_frame() const {
    if (!first_frame_received_.load()) {
        return -1;
    }
    
    std::lock_guard<std::mutex> lock(frame_time_mutex_);
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_frame_time_
    ).count();
}

void Producer::set_error_callback(std::function<void(const std::string&)> callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    error_callback_ = callback;
}

void Producer::set_status_callback(std::function<void(const std::string&)> callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    status_callback_ = callback;
}

bool Producer::configure_pipeline() {
    // Enable ONLY the left IR stream (index 1)
    rs_config_.enable_stream(
        RS2_STREAM_INFRARED, 1,  // Left IR sensor
        config_.ir_width,
        config_.ir_height,
        RS2_FORMAT_Y8,          // 8-bit grayscale
        config_.ir_fps
    );
    
    // Select specific device if serial provided
    if (!config_.device_serial.empty()) {
        rs_config_.enable_device(config_.device_serial);
    }
    
    std::cout << "📹 IR-only mode: " << config_.ir_width << "x" 
              << config_.ir_height << "@" << config_.ir_fps << "fps" << std::endl;
    
    return true;
}

void Producer::configure_sensor_options() {
    try {
        auto device = profile_.get_device();
        auto sensors = device.query_sensors();
        
        for (auto& sensor : sensors) {
            std::string sensor_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
            
            // Find stereo/depth sensor (controls IR)
            if (sensor_name.find("Stereo") != std::string::npos ||
                sensor_name.find("Depth") != std::string::npos) {
                
                // CRITICAL: Disable laser emitter for passive IR
                if (sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
                    sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
                    std::cout << "📹 Laser emitter: DISABLED (passive IR mode)" << std::endl;
                }
                
                // Configure exposure
                if (config_.auto_exposure) {
                    if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
                        std::cout << "📹 IR auto-exposure: ENABLED" << std::endl;
                    }
                } else {
                    if (sensor.supports(RS2_OPTION_EXPOSURE)) {
                        sensor.set_option(RS2_OPTION_EXPOSURE, config_.manual_exposure);
                        std::cout << "📹 IR exposure: " << config_.manual_exposure << "us" << std::endl;
                    }
                    if (sensor.supports(RS2_OPTION_GAIN)) {
                        sensor.set_option(RS2_OPTION_GAIN, config_.manual_gain);
                        std::cout << "📹 IR gain: " << config_.manual_gain << std::endl;
                    }
                }
            }
        }
        
    } catch (const rs2::error& e) {
        report_error(std::string("Error configuring sensor: ") + e.what());
    }
}

void Producer::query_camera_parameters() {
    try {
        auto ir_stream = profile_.get_stream(RS2_STREAM_INFRARED, 1)
                                .as<rs2::video_stream_profile>();
        ir_intrinsics_ = ir_stream.get_intrinsics();
        
        std::cout << "📹 IR intrinsics: fx=" << ir_intrinsics_.fx 
                  << ", fy=" << ir_intrinsics_.fy << std::endl;
        
    } catch (const rs2::error& e) {
        report_error(std::string("Error querying camera parameters: ") + e.what());
    }
}

void Producer::capture_loop() {
    report_status("IR capture loop started");
    
    int frame_count = 0;
    
    while (running_.load()) {
        try {
            rs2::frameset frames;
            if (pipe_.poll_for_frames(&frames)) {
                frame_count++;
                
                // Get IR frame
                auto ir_frame = frames.get_infrared_frame(1);
                if (ir_frame) {
                    FrameBox framebox = process_ir_frame(ir_frame);
                    
                    {
                        std::lock_guard<std::mutex> lock(frame_time_mutex_);
                        last_frame_time_ = std::chrono::steady_clock::now();
                    }
                    first_frame_received_.store(true);
                    
                    if (ring_buffer_->write(std::move(framebox))) {
                        total_frames_captured_.fetch_add(1);
                        frames_since_last_calc_++;
                    }
                }
                
                // Log progress occasionally
                if (frame_count <= 5 || frame_count % 100 == 0) {
                    std::cout << "📹 IR frame #" << frame_count << std::endl;
                }
            }
            
            // Update FPS
            update_fps();
            
            // Small sleep to avoid busy-waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            
        } catch (const std::exception& e) {
            report_error(std::string("Capture error: ") + e.what());
        }
    }
    
    report_status("IR capture loop stopped");
}

FrameBox Producer::process_ir_frame(rs2::video_frame& ir_frame) {
    FrameBox fb;
    
    fb.sequence_id = sequence_id_.fetch_add(1);
    fb.timestamp = ir_frame.get_timestamp();
    
    // Get frame dimensions
    fb.ir_width = ir_frame.get_width();
    fb.ir_height = ir_frame.get_height();
    
    // Copy IR data
    size_t ir_size = fb.ir_width * fb.ir_height;
    fb.ir_data.resize(ir_size);
    
    const uint8_t* ir_data = reinterpret_cast<const uint8_t*>(ir_frame.get_data());
    std::copy(ir_data, ir_data + ir_size, fb.ir_data.begin());
    
    // Store frame dimensions in metadata
    fb.metadata.frame_width = fb.ir_width;
    fb.metadata.frame_height = fb.ir_height;
    
    return fb;
}

void Producer::update_fps() {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_fps_calc_time_).count();
    
    // Update FPS every second
    if (duration >= 1000) {
        float fps = (frames_since_last_calc_ * 1000.0f) / duration;
        current_fps_.store(fps);
        
        frames_since_last_calc_ = 0;
        last_fps_calc_time_ = now;
    }
}

void Producer::report_error(const std::string& error) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    std::cerr << "[Producer Error] " << error << std::endl;
    if (error_callback_) {
        error_callback_(error);
    }
}

void Producer::report_status(const std::string& status) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    std::cout << "[Producer] " << status << std::endl;
    if (status_callback_) {
        status_callback_(status);
    }
}

} // namespace mdai
