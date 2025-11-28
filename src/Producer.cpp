#include "Producer.hpp"
#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>

namespace mdai {

Producer::Producer(const CameraConfig& config, DynamicRingBuffer* ring_buffer)
    : config_(config), ring_buffer_(ring_buffer) {
    
    if (config_.align_to_color) {
        aligner_ = new rs2::align(RS2_STREAM_COLOR);
    }
    
    last_fps_calc_time_ = std::chrono::steady_clock::now();
}

Producer::~Producer() {
    stop();
    if (aligner_) {
        delete aligner_;
    }
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
        
        // CRITICAL: Create fresh pipeline - cannot reuse after stop()
        pipe_ = rs2::pipeline();
        
        // Determine if we should use polling mode (more reliable for multi-stream)
        bool use_polling = config_.enable_depth || config_.enable_ir;
        
        report_status("Starting camera pipeline...");
        
        if (use_polling) {
            // POLLING MODE: More reliable for multi-stream (Depth + Color + IR)
            std::cout << "📹 Using POLLING mode for multi-stream capture" << std::endl;
            profile_ = pipe_.start(rs_config_);  // No callback - polling mode
            use_polling_mode_ = true;
        } else {
            // CALLBACK MODE: Single stream (RGB-only) - works reliably
            std::cout << "📹 Using CALLBACK mode for single-stream capture" << std::endl;
            
            auto callback = [this](const rs2::frame& frame) {
                static thread_local int callback_count = 0;
                static thread_local auto last_log_time = std::chrono::steady_clock::now();
                callback_count++;
                
                (void)callback_count;  // Suppress unused warning
                
                try {
                    rs2::frameset fs = frame.as<rs2::frameset>();
                    if (!fs) return;
                    
                    rs2::frameset processed = fs;
                    if (config_.align_to_color && aligner_) {
                        processed = aligner_->process(processed);
                    }
                    
                    FrameBox framebox = process_frameset(processed);
                    
                    {
                        std::lock_guard<std::mutex> lock(frame_time_mutex_);
                        last_frame_time_ = std::chrono::steady_clock::now();
                    }
                    first_frame_received_.store(true);
                    
                    if (ring_buffer_->write(std::move(framebox))) {
                        total_frames_captured_.fetch_add(1);
                        frames_since_last_calc_++;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "[Callback Error] " << e.what() << std::endl;
                }
            };
            
            profile_ = pipe_.start(rs_config_, callback);
            use_polling_mode_ = false;
        }
        
        // Log active streams
        auto streams = profile_.get_streams();
        std::cout << "📹 Pipeline started with " << streams.size() << " stream(s)" << std::endl;
        
        configure_sensor_options();
        query_camera_parameters();
        
        running_.store(true);
        camera_connected_.store(true);
        
        capture_thread_ = std::make_unique<std::thread>(&Producer::capture_loop, this);
        
        // Wait for first frame to confirm camera is actually working
        report_status("Waiting for first frame...");
        auto start_wait = std::chrono::steady_clock::now();
        const int FIRST_FRAME_TIMEOUT_MS = 5000;
        
        while (!first_frame_received_.load()) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_wait
            ).count();
            
            if (elapsed > FIRST_FRAME_TIMEOUT_MS) {
                report_error("Camera timeout - no frames received in 5 seconds");
                stop();
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        report_status("Camera ready - frames flowing");
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
            
            // CRITICAL: Give the hardware time to flush buffers and release resources
            // RealSense cameras need time to fully release USB and memory resources
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    } catch (const rs2::error& e) {
        report_error(std::string("Error stopping pipeline: ") + e.what());
    } catch (...) {
        report_error("Unknown error during pipeline stop");
    }
    
    // Clear the camera initializer
    camera_initializer_.reset();
    
    // Reset state for clean restart
    first_frame_received_.store(false);
    rs_config_ = rs2::config();  // Fresh config object to avoid duplicate stream registration
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

bool Producer::set_option(rs2_option option, float value) {
    try {
        auto sensors = profile_.get_device().query_sensors();
        for (auto& sensor : sensors) {
            if (sensor.supports(option)) {
                sensor.set_option(option, value);
                return true;
            }
        }
        return false;
    } catch (const rs2::error& e) {
        report_error(std::string("Error setting option: ") + e.what());
        return false;
    }
}

float Producer::get_option(rs2_option option) const {
    try {
        auto sensors = profile_.get_device().query_sensors();
        for (auto& sensor : sensors) {
            if (sensor.supports(option)) {
                return sensor.get_option(option);
            }
        }
        return 0.0f;
    } catch (const rs2::error& e) {
        return 0.0f;
    }
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
    // Must be running
    if (!running_.load()) {
        return false;
    }
    
    // Must be connected
    if (!camera_connected_.load()) {
        return false;
    }
    
    // Check if we've received any frames yet
    if (!first_frame_received_.load()) {
        return false;
    }
    
    // Check if frames are still arriving within timeout
    int64_t ms_since_last = get_ms_since_last_frame();
    if (ms_since_last < 0 || ms_since_last > timeout_ms) {
        return false;
    }
    
    return true;
}

int64_t Producer::get_ms_since_last_frame() const {
    if (!first_frame_received_.load()) {
        return -1;  // No frame received yet
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
    // Enable depth stream if requested
    if (config_.enable_depth && config_.depth_width > 0 && config_.depth_height > 0) {
        rs_config_.enable_stream(
            RS2_STREAM_DEPTH,
            config_.depth_width,
            config_.depth_height,
            RS2_FORMAT_Z16,
            config_.depth_fps
        );
    }
    
    // Enable color stream (required)
    if (config_.color_width > 0 && config_.color_height > 0) {
        rs_config_.enable_stream(
            RS2_STREAM_COLOR,
            config_.color_width,
            config_.color_height,
            RS2_FORMAT_BGR8,
            config_.color_fps
        );
    } else {
        report_error("Invalid color stream dimensions");
        return false;
    }
    
    // Enable IR streams if requested
    if (config_.enable_ir && config_.ir_width > 0 && config_.ir_height > 0) {
        rs_config_.enable_stream(
            RS2_STREAM_INFRARED, 1,
            config_.ir_width,
            config_.ir_height,
            RS2_FORMAT_Y8,
            config_.ir_fps
        );
        
        rs_config_.enable_stream(
            RS2_STREAM_INFRARED, 2,
            config_.ir_width,
            config_.ir_height,
            RS2_FORMAT_Y8,
            config_.ir_fps
        );
    }
    
    // Select specific device if serial provided
    if (!config_.device_serial.empty()) {
        rs_config_.enable_device(config_.device_serial);
    }
    
    // Configuration complete - pipeline will be started later
    return true;
}

void Producer::configure_sensor_options() {
    try {
        auto device = profile_.get_device();
        
        // Configure depth sensor
        if (config_.enable_depth) {
            auto depth_sensor = device.first<rs2::depth_sensor>();
            
            // Set internal queue size
            if (depth_sensor.supports(RS2_OPTION_FRAMES_QUEUE_SIZE)) {
                depth_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, config_.internal_queue_size);
                std::cout << "  Internal queue size set to: " << config_.internal_queue_size << std::endl;
            }
            
            // Set emitter
            if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
                depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, config_.emitter_enabled);
                std::cout << "  Emitter enabled: " << config_.emitter_enabled << std::endl;
            }
            
            // Set laser power
            if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
                depth_sensor.set_option(RS2_OPTION_LASER_POWER, config_.laser_power);
                std::cout << "  Laser power: " << config_.laser_power << " mW" << std::endl;
            }
        }
        
        // Configure color sensor with full auto settings
        auto sensors = device.query_sensors();
        for (auto& sensor : sensors) {
            // Auto exposure
            if (config_.auto_exposure && sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
            } else if (!config_.auto_exposure && sensor.supports(RS2_OPTION_EXPOSURE)) {
                sensor.set_option(RS2_OPTION_EXPOSURE, config_.manual_exposure);
                if (sensor.supports(RS2_OPTION_GAIN)) {
                    sensor.set_option(RS2_OPTION_GAIN, config_.manual_gain);
                }
            }
            
            // Auto white balance (always enable for consistent color)
            if (sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)) {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 1.0f);
            }
        }
        
        // Configure filters
        if (config_.enable_spatial_filter) {
            spatial_filter_.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, config_.spatial_alpha);
            spatial_filter_.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, config_.spatial_delta);
        }
        
        if (config_.enable_temporal_filter) {
            temporal_filter_.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, config_.temporal_alpha);
            temporal_filter_.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, config_.temporal_delta);
        }
        
    } catch (const rs2::error& e) {
        report_error(std::string("Error configuring sensor options: ") + e.what());
    }
}

void Producer::query_camera_parameters() {
    try {
        // Depth info (only if enabled)
        rs2::video_stream_profile depth_stream;
        if (config_.enable_depth) {
            depth_stream = profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
            depth_intrinsics_ = depth_stream.get_intrinsics();
            auto depth_sensor = profile_.get_device().first<rs2::depth_sensor>();
            depth_scale_ = depth_sensor.get_depth_scale();
        } else {
            depth_scale_ = 0.0f;
            depth_intrinsics_ = {};
        }
        
        // Color stream profile
        auto color_stream = profile_.get_stream(RS2_STREAM_COLOR)
                                   .as<rs2::video_stream_profile>();
        color_intrinsics_ = color_stream.get_intrinsics();
        
        // Get IR intrinsics (same for left and right)
        if (config_.enable_ir) {
            auto ir_stream = profile_.get_stream(RS2_STREAM_INFRARED, 1)
                                    .as<rs2::video_stream_profile>();
            ir_intrinsics_ = ir_stream.get_intrinsics();
        }
        
        // Get extrinsics if both depth and color are available
        if (config_.enable_depth) {
            extrinsics_depth_to_color_ = depth_stream.get_extrinsics_to(color_stream);
        } else {
            extrinsics_depth_to_color_ = {};
        }
        
        std::cout << "Camera parameters:" << std::endl;
        if (config_.enable_depth) {
            std::cout << "  Depth scale: " << depth_scale_ << " m/unit" << std::endl;
            std::cout << "  Depth intrinsics: fx=" << depth_intrinsics_.fx 
                      << ", fy=" << depth_intrinsics_.fy << std::endl;
        } else {
            std::cout << "  Depth stream disabled in this mode" << std::endl;
        }
        std::cout << "  Color intrinsics: fx=" << color_intrinsics_.fx 
                  << ", fy=" << color_intrinsics_.fy << std::endl;
        
    } catch (const rs2::error& e) {
        report_error(std::string("Error querying camera parameters: ") + e.what());
    }
}

void Producer::capture_loop() {
    report_status("Capture loop started (" + std::string(use_polling_mode_ ? "polling" : "callback") + " mode)");
    
    int poll_frame_count = 0;
    
    while (running_.load()) {
        // POLLING MODE: Actively fetch frames
        if (use_polling_mode_) {
            try {
                rs2::frameset frames;
                if (pipe_.poll_for_frames(&frames)) {
                    poll_frame_count++;
                    
                    if (poll_frame_count <= 5) {
                        std::cout << "📥 [POLL #" << poll_frame_count << "] Got frameset with " 
                                  << frames.size() << " frames" << std::endl;
                    } else if (poll_frame_count % 30 == 0) {
                        std::cout << "📥 [POLL] Total polled: " << poll_frame_count << " framesets" << std::endl;
                    }
                    
                    // Process the frameset
                    rs2::frameset processed = frames;
                    
                    if (config_.enable_spatial_filter) {
                        processed = spatial_filter_.process(processed);
                    }
                    if (config_.enable_temporal_filter) {
                        processed = temporal_filter_.process(processed);
                    }
                    if (config_.enable_hole_filling) {
                        processed = hole_filling_filter_.process(processed);
                    }
                    if (config_.align_to_color && aligner_) {
                        processed = aligner_->process(processed);
                    }
                    
                    FrameBox framebox = process_frameset(processed);
                    
                    {
                        std::lock_guard<std::mutex> lock(frame_time_mutex_);
                        last_frame_time_ = std::chrono::steady_clock::now();
                    }
                    first_frame_received_.store(true);
                    
                    bool write_success = ring_buffer_->write(std::move(framebox));
                    if (write_success) {
                        total_frames_captured_.fetch_add(1);
                        frames_since_last_calc_++;
                    }
                }
            } catch (const std::exception& e) {
                std::cerr << "[Poll Error] " << e.what() << std::endl;
            }
            
            // Small sleep to not spin too fast
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        
        // CALLBACK MODE: Just monitor health
        update_fps();
        
        // Sleep to avoid busy-waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    report_status("Capture loop stopped");
}

FrameBox Producer::process_frameset(rs2::frameset& frames) {
    FrameBox fb;
    
    fb.sequence_id = sequence_id_.fetch_add(1);
    
    // Extract frames and copy raw data immediately (no rs2::frame references!)
    auto depth_frame = frames.get_depth_frame();
    auto color_frame = frames.get_color_frame();
    
    if (depth_frame) {
        // Copy depth data
        fb.depth_width = depth_frame.get_width();
        fb.depth_height = depth_frame.get_height();
        size_t depth_size = fb.depth_width * fb.depth_height;
        fb.depth_data.resize(depth_size);
        
        const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
        std::copy(depth_data, depth_data + depth_size, fb.depth_data.begin());
        
        fb.time_depth = depth_frame.get_timestamp();
    }
    
    if (color_frame) {
        // Get raw frame dimensions
        int raw_width = color_frame.get_width();
        int raw_height = color_frame.get_height();
        
        // Create cv::Mat from raw frame data (no copy, just wraps pointer)
        cv::Mat raw_mat(raw_height, raw_width, CV_8UC3, 
                        const_cast<void*>(color_frame.get_data()));
        
        // Rotate 90° clockwise for portrait orientation
        // Camera is mounted sideways (USB ports to right)
        cv::Mat rotated;
        cv::rotate(raw_mat, rotated, cv::ROTATE_90_CLOCKWISE);
        
        // Store rotated dimensions (848x480 -> 480x848)
        fb.color_width = rotated.cols;
        fb.color_height = rotated.rows;
        
        // Copy rotated data to FrameBox
        size_t color_size = fb.color_width * fb.color_height * 3;
        fb.color_data.resize(color_size);
        std::memcpy(fb.color_data.data(), rotated.data, color_size);
        
        fb.time_color = color_frame.get_timestamp();
    }
    
    if (config_.enable_ir) {
        auto ir_left = frames.get_infrared_frame(1);
        auto ir_right = frames.get_infrared_frame(2);
        
        if (ir_left) {
            fb.ir_width = ir_left.get_width();
            fb.ir_height = ir_left.get_height();
            size_t ir_size = fb.ir_width * fb.ir_height;
            fb.ir_left_data.resize(ir_size);
            
            const uint8_t* ir_data = reinterpret_cast<const uint8_t*>(ir_left.get_data());
            std::copy(ir_data, ir_data + ir_size, fb.ir_left_data.begin());
            
            fb.time_ir_left = ir_left.get_timestamp();
        }
        
        if (ir_right) {
            size_t ir_size = fb.ir_width * fb.ir_height;
            fb.ir_right_data.resize(ir_size);
            
            const uint8_t* ir_data = reinterpret_cast<const uint8_t*>(ir_right.get_data());
            std::copy(ir_data, ir_data + ir_size, fb.ir_right_data.begin());
            
            fb.time_ir_right = ir_right.get_timestamp();
        }
    }
    
    // Camera parameters
    fb.depth_intrinsics = depth_intrinsics_;
    fb.color_intrinsics = color_intrinsics_;
    fb.ir_intrinsics = ir_intrinsics_;
    fb.extrinsics_depth_to_color = extrinsics_depth_to_color_;
    fb.depth_scale = depth_scale_;
    
    // Camera settings snapshot
    fb.exposure = get_option(RS2_OPTION_EXPOSURE);
    fb.gain = get_option(RS2_OPTION_GAIN);
    fb.emitter_state = static_cast<int>(get_option(RS2_OPTION_EMITTER_ENABLED));
    fb.laser_power = get_option(RS2_OPTION_LASER_POWER);
    
    // Filter settings
    fb.filter_settings.spatial_alpha = config_.spatial_alpha;
    fb.filter_settings.spatial_delta = config_.spatial_delta;
    fb.filter_settings.temporal_alpha = config_.temporal_alpha;
    fb.filter_settings.temporal_delta = config_.temporal_delta;
    
    return fb;
}

rs2::frameset Producer::apply_filters(rs2::frameset frames) {
    // Alignment
    if (config_.align_to_color && aligner_) {
        frames = aligner_->process(frames);
    }
    
    // Spatial filter
    if (config_.enable_spatial_filter) {
        frames = spatial_filter_.process(frames);
    }
    
    // Temporal filter
    if (config_.enable_temporal_filter) {
        frames = temporal_filter_.process(frames);
    }
    
    // Hole filling
    if (config_.enable_hole_filling) {
        frames = hole_filling_filter_.process(frames);
    }
    
    return frames;
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
    std::cout << "[Producer Status] " << status << std::endl;
    if (status_callback_) {
        status_callback_(status);
    }
}

} // namespace mdai
