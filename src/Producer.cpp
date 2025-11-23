#include "Producer.hpp"
#include <iostream>
#include <chrono>

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
        
        // Initialize camera with proper stabilization
        camera_initializer_ = std::make_unique<CameraInitializer>(rs_config_, config_.device_serial);
        
        // Set up callbacks for initialization progress
        camera_initializer_->set_status_callback([this](const std::string& status) {
            report_status(status);
        });
        
        camera_initializer_->set_progress_callback([this](int progress) {
            (void)progress; // Suppress unused parameter warning
            // Could add progress reporting here if needed
        });
        
        // Perform camera initialization with stabilization
        report_status("Starting camera initialization with stabilization...");
        auto init_result = camera_initializer_->initialize(30000); // 30 second timeout
        
        if (init_result != CameraInitializer::Result::SUCCESS) {
            std::string error_msg;
            switch (init_result) {
                case CameraInitializer::Result::CONNECTION_FAILED:
                    error_msg = "Camera connection failed";
                    break;
                case CameraInitializer::Result::SENSOR_ERROR:
                    error_msg = "Camera sensor error during initialization";
                    break;
                case CameraInitializer::Result::TIMEOUT:
                    error_msg = "Camera initialization timeout";
                    break;
                case CameraInitializer::Result::CANCELLED:
                    error_msg = "Camera initialization cancelled";
                    break;
                default:
                    error_msg = "Unknown camera initialization error";
                    break;
            }
            report_error(error_msg);
            return false;
        }
        
        // Get the initialized profile and pipeline
        profile_ = camera_initializer_->get_profile();
        pipe_ = std::move(camera_initializer_->get_pipeline());
        
        // Now start the pipeline with callback
        report_status("Starting camera pipeline...");
        auto callback = [this](const rs2::frame& frame) {
            try {
                if (rs2::frameset fs = frame.as<rs2::frameset>()) {
                    // Apply post-processing filters for better data quality
                    rs2::frameset processed = fs;
                    
                    // Apply spatial filter for smooth depth surfaces
                    if (config_.enable_spatial_filter) {
                        processed = spatial_filter_.process(processed);
                    }
                    
                    // Apply temporal filter for consistent depth over time
                    if (config_.enable_temporal_filter) {
                        processed = temporal_filter_.process(processed);
                    }
                    
                    // Apply hole filling for complete depth data
                    if (config_.enable_hole_filling) {
                        processed = hole_filling_filter_.process(processed);
                    }
                    
                    // Align/process frames if configured (ensure depth/color share same space)
                    if (config_.align_to_color && aligner_) {
                        processed = aligner_->process(processed);
                    }
                    
                    // Process frameset and extract raw data (no rs2::frame references!)
                    FrameBox framebox = process_frameset(processed);
                    
                    // Write to ring buffer (non-blocking)
                    // Ring buffer internally handles recording_active flag
                    if (ring_buffer_->write(std::move(framebox))) {
                        // Update statistics only on successful write
                        total_frames_captured_.fetch_add(1);
                        frames_since_last_calc_++;
                    }
                }
            } catch (const std::exception& e) {
                // Don't let exceptions kill the callback
                std::cerr << "[Callback Error] " << e.what() << std::endl;
            }
        };
        
        profile_ = pipe_.start(rs_config_, callback);
        
        configure_sensor_options();
        query_camera_parameters();
        
        running_.store(true);
        camera_connected_.store(true);
        
        capture_thread_ = std::make_unique<std::thread>(&Producer::capture_loop, this);
        
        report_status("Producer started successfully");
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
        }
    } catch (const rs2::error& e) {
        report_error(std::string("Error stopping pipeline: ") + e.what());
    } catch (...) {
        report_error("Unknown error during pipeline stop");
    }
    
    // Clear the camera initializer
    camera_initializer_.reset();
    
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

void Producer::set_error_callback(std::function<void(const std::string&)> callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    error_callback_ = callback;
}

void Producer::set_status_callback(std::function<void(const std::string&)> callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    status_callback_ = callback;
}

bool Producer::configure_pipeline() {
    // Enable depth stream
    rs_config_.enable_stream(
        RS2_STREAM_DEPTH,
        config_.depth_width,
        config_.depth_height,
        RS2_FORMAT_Z16,
        config_.depth_fps
    );
    
    // Enable color stream
    rs_config_.enable_stream(
        RS2_STREAM_COLOR,
        config_.color_width,
        config_.color_height,
        RS2_FORMAT_BGR8,
        config_.color_fps
    );
    
    // Enable IR streams if requested
    if (config_.enable_ir) {
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
        
        // Configure color sensor
        auto sensors = device.query_sensors();
        for (auto& sensor : sensors) {
            if (sensor.supports(RS2_OPTION_EXPOSURE)) {
                if (config_.auto_exposure) {
                    if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
                    }
                } else {
                    sensor.set_option(RS2_OPTION_EXPOSURE, config_.manual_exposure);
                    sensor.set_option(RS2_OPTION_GAIN, config_.manual_gain);
                }
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
        // Get depth stream profile
        auto depth_stream = profile_.get_stream(RS2_STREAM_DEPTH)
                                   .as<rs2::video_stream_profile>();
        depth_intrinsics_ = depth_stream.get_intrinsics();
        
        // Get color stream profile
        auto color_stream = profile_.get_stream(RS2_STREAM_COLOR)
                                   .as<rs2::video_stream_profile>();
        color_intrinsics_ = color_stream.get_intrinsics();
        
        // Get IR intrinsics (same for left and right)
        if (config_.enable_ir) {
            auto ir_stream = profile_.get_stream(RS2_STREAM_INFRARED, 1)
                                    .as<rs2::video_stream_profile>();
            ir_intrinsics_ = ir_stream.get_intrinsics();
        }
        
        // Get extrinsics
        extrinsics_depth_to_color_ = depth_stream.get_extrinsics_to(color_stream);
        
        // Get depth scale
        auto depth_sensor = profile_.get_device().first<rs2::depth_sensor>();
        depth_scale_ = depth_sensor.get_depth_scale();
        
        std::cout << "Camera parameters:" << std::endl;
        std::cout << "  Depth scale: " << depth_scale_ << " m/unit" << std::endl;
        std::cout << "  Depth intrinsics: fx=" << depth_intrinsics_.fx 
                  << ", fy=" << depth_intrinsics_.fy << std::endl;
        std::cout << "  Color intrinsics: fx=" << color_intrinsics_.fx 
                  << ", fy=" << color_intrinsics_.fy << std::endl;
        
    } catch (const rs2::error& e) {
        report_error(std::string("Error querying camera parameters: ") + e.what());
    }
}

void Producer::capture_loop() {
    // With callback mode, frames are delivered automatically
    // This thread just updates FPS and monitors health
    report_status("Capture loop started (callback mode)");
    
    while (running_.load()) {
        // Update FPS periodically
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
        // Copy color data
        fb.color_width = color_frame.get_width();
        fb.color_height = color_frame.get_height();
        size_t color_size = fb.color_width * fb.color_height * 3; // BGR8 = 3 bytes per pixel
        fb.color_data.resize(color_size);
        
        const uint8_t* color_data = reinterpret_cast<const uint8_t*>(color_frame.get_data());
        std::copy(color_data, color_data + color_size, fb.color_data.begin());
        
        fb.time_color = color_frame.get_timestamp();
    }
    
    if (config_.enable_ir) {
        auto ir_left = frames.get_infrared_frame(1);
        auto ir_right = frames.get_infrared_frame(2);
        
        // Debug: Check if IR frames are being received
        static int ir_frame_debug = 0;
        if (++ir_frame_debug % 30 == 0) {
            std::cout << "🔍 IR Frame Debug: ir_left=" << (ir_left ? "YES" : "NO") 
                      << ", ir_right=" << (ir_right ? "YES" : "NO") << std::endl;
        }
        
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
