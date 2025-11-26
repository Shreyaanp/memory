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
        if (config_.quick_init) {
            report_status("Starting QUICK camera initialization (mode switch)...");
            camera_initializer_->set_quick_mode(true);
        } else {
            report_status("Starting camera initialization with stabilization...");
        }
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
        
        // Detect camera orientation using IMU (before starting main pipeline)
        report_status("Checking camera orientation via IMU...");
        detect_camera_orientation();
        
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
                    
                    // Update frame health tracking BEFORE ring buffer write
                    // This ensures we know frames are arriving even if buffer is full/disabled
                    {
                        std::lock_guard<std::mutex> lock(frame_time_mutex_);
                        last_frame_time_ = std::chrono::steady_clock::now();
                    }
                    first_frame_received_.store(true);
                    
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
    // With callback mode, frames are delivered automatically
    // This thread updates FPS and monitors camera health via frame timing only
    // NOTE: Do NOT create rs2::context() here - it conflicts with running pipeline!
    report_status("Capture loop started (callback mode)");
    
    const int FRAME_TIMEOUT_MS = 5000;  // 5 seconds without frames = error
    const int HEALTH_CHECK_INTERVAL = 20;  // Check every 2 seconds (20 * 100ms)
    int health_check_counter = 0;
    bool error_reported = false;
    
    while (running_.load()) {
        // Update FPS periodically
        update_fps();
        
        // Check camera health periodically (frame timing only - safe)
        if (++health_check_counter >= HEALTH_CHECK_INTERVAL) {
            health_check_counter = 0;
            
            // Only check if we've received at least one frame
            if (first_frame_received_.load()) {
                int64_t ms_since_last = get_ms_since_last_frame();
                
                if (ms_since_last > FRAME_TIMEOUT_MS) {
                    // Camera has stopped sending frames
                    if (!error_reported) {
                        std::cerr << "⚠️  Camera frame timeout! No frames for " << ms_since_last << "ms" << std::endl;
                        camera_connected_.store(false);
                        report_error("Camera disconnected - no frames for " + 
                                     std::to_string(ms_since_last / 1000) + "s");
                        error_reported = true;
                    }
                } else if (error_reported && ms_since_last < FRAME_TIMEOUT_MS) {
                    // Frames started arriving again
                    std::cout << "✅ Camera recovered - frames arriving again" << std::endl;
                    camera_connected_.store(true);
                    report_status("Camera recovered");
                    error_reported = false;
                }
            }
            // Removed rs2::context() query - it conflicts with running pipeline!
        }
        
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

// ============================================================================
// IMU-Based Camera Orientation Detection
// ============================================================================

Producer::CameraOrientation Producer::detect_camera_orientation() {
    try {
        // Check if device has IMU (D435I has it, D435 doesn't)
        rs2::context ctx;
        auto devices = ctx.query_devices();
        if (devices.size() == 0) {
            report_status("No RealSense device found for IMU check");
            return CameraOrientation::UNKNOWN;
        }
        
        rs2::device dev = devices[0];
        
        // Check for motion sensor
        bool found_accel = false;
        for (auto&& sensor : dev.query_sensors()) {
            for (auto&& profile : sensor.get_stream_profiles()) {
                if (profile.stream_type() == RS2_STREAM_ACCEL) {
                    found_accel = true;
                    break;
                }
            }
        }
        
        if (!found_accel) {
            report_status("⚠️  Camera does not have IMU (D435 vs D435I?) - orientation detection unavailable");
            has_imu_ = false;
            detected_orientation_ = CameraOrientation::UNKNOWN;  // Don't assume - unknown orientation
            return detected_orientation_;
        }
        
        has_imu_ = true;
        report_status("IMU detected - reading accelerometer for orientation...");
        
        // Create a separate pipeline for IMU
        rs2::pipeline imu_pipe;
        rs2::config imu_config;
        imu_config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 200);
        
        imu_pipe.start(imu_config);
        
        // Collect accelerometer samples to average out noise
        float accel_x = 0, accel_y = 0, accel_z = 0;
        int sample_count = 0;
        const int NUM_SAMPLES = 50;
        
        auto start_time = std::chrono::steady_clock::now();
        while (sample_count < NUM_SAMPLES) {
            rs2::frameset frames;
            if (imu_pipe.poll_for_frames(&frames)) {
                for (auto&& frame : frames) {
                    if (auto motion = frame.as<rs2::motion_frame>()) {
                        if (motion.get_profile().stream_type() == RS2_STREAM_ACCEL) {
                            auto data = motion.get_motion_data();
                            accel_x += data.x;
                            accel_y += data.y;
                            accel_z += data.z;
                            sample_count++;
                        }
                    }
                }
            }
            
            // Timeout after 2 seconds
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count();
            if (elapsed > 2000) break;
        }
        
        imu_pipe.stop();
        
        if (sample_count == 0) {
            report_status("Failed to read IMU data");
            detected_orientation_ = CameraOrientation::NORMAL;
            return detected_orientation_;
        }
        
        // Average the samples
        accel_x /= sample_count;
        accel_y /= sample_count;
        accel_z /= sample_count;
        
        report_status("IMU Accelerometer: X=" + std::to_string(accel_x) + 
                     " Y=" + std::to_string(accel_y) + 
                     " Z=" + std::to_string(accel_z));
        
        // Determine orientation from gravity vector
        // In RealSense coordinate system:
        // - Camera facing user horizontally: gravity should be mostly in -Y
        // - Rotated 90° CW: gravity in -X
        // - Rotated 90° CCW: gravity in +X
        // - Upside down: gravity in +Y
        
        const float GRAVITY_THRESHOLD = 7.0f;  // ~70% of 9.8 m/s²
        
        if (accel_y < -GRAVITY_THRESHOLD) {
            detected_orientation_ = CameraOrientation::NORMAL;
            report_status("📷 Camera orientation: NORMAL (correct)");
        } else if (accel_y > GRAVITY_THRESHOLD) {
            detected_orientation_ = CameraOrientation::UPSIDE_DOWN;
            report_status("⚠️  Camera orientation: UPSIDE DOWN (180° rotation applied)");
        } else if (accel_x < -GRAVITY_THRESHOLD) {
            detected_orientation_ = CameraOrientation::ROTATED_90_CW;
            report_status("⚠️  Camera orientation: ROTATED 90° CW (rotation applied)");
        } else if (accel_x > GRAVITY_THRESHOLD) {
            detected_orientation_ = CameraOrientation::ROTATED_90_CCW;
            report_status("⚠️  Camera orientation: ROTATED 90° CCW (rotation applied)");
        } else {
            // Camera might be tilted or pointing up/down
            detected_orientation_ = CameraOrientation::NORMAL;
            report_status("📷 Camera orientation: Could not determine precisely, assuming NORMAL");
        }
        
        return detected_orientation_;
        
    } catch (const rs2::error& e) {
        report_status("IMU detection error: " + std::string(e.what()));
        detected_orientation_ = CameraOrientation::NORMAL;
        return detected_orientation_;
    }
}

int Producer::get_rotation_degrees() const {
    switch (detected_orientation_) {
        case CameraOrientation::NORMAL:        return 0;
        case CameraOrientation::ROTATED_90_CW: return 90;
        case CameraOrientation::UPSIDE_DOWN:   return 180;
        case CameraOrientation::ROTATED_90_CCW: return 270;
        default:                               return 0;
    }
}

void Producer::transform_coordinates(int x, int y, int max_x, int max_y, int& out_x, int& out_y) const {
    switch (detected_orientation_) {
        case CameraOrientation::NORMAL:
            // No transformation needed
            out_x = x;
            out_y = y;
            break;
            
        case CameraOrientation::ROTATED_90_CW:
            // Rotate 90° clockwise: (x,y) -> (max_y - y, x)
            out_x = max_y - y;
            out_y = x;
            break;
            
        case CameraOrientation::ROTATED_90_CCW:
            // Rotate 90° counter-clockwise: (x,y) -> (y, max_x - x)
            out_x = y;
            out_y = max_x - x;
            break;
            
        case CameraOrientation::UPSIDE_DOWN:
            // Rotate 180°: (x,y) -> (max_x - x, max_y - y)
            out_x = max_x - x;
            out_y = max_y - y;
            break;
            
        default:
            out_x = x;
            out_y = y;
            break;
    }
}

std::string Producer::orientation_to_string(CameraOrientation orientation) {
    switch (orientation) {
        case CameraOrientation::NORMAL:        return "NORMAL";
        case CameraOrientation::ROTATED_90_CW: return "ROTATED_90_CW";
        case CameraOrientation::ROTATED_90_CCW: return "ROTATED_90_CCW";
        case CameraOrientation::UPSIDE_DOWN:   return "UPSIDE_DOWN";
        default:                               return "UNKNOWN";
    }
}

} // namespace mdai
