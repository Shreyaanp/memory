#include "FrameBox.hpp"
#include "RingBuffer.hpp"
#include "Producer.hpp"
#include "Pipeline.hpp"
#include "Utils.hpp"
#include <iostream>
#include <signal.h>

using namespace mdai;

// Global flag for graceful shutdown
std::atomic<bool> g_running{true};

void signal_handler(int signum) {
    std::cout << "\nReceived signal " << signum << ", shutting down..." << std::endl;
    g_running.store(false);
}

// Example Anti-Spoofing Stage
class AntiSpoofingStage : public PipelineStage {
public:
    bool process(FrameBox* frame) override {
        // Simulate anti-spoofing check
        // In real implementation, this would run ML model on depth/color frames
        
        // Simple mock: check if depth data is present
        if (!frame->get_depth_ptr()) {
            frame->metadata.is_ready_for_processing = false;
            frame->metadata.anti_spoofing.overall_liveness_score = 0.0f;
            frame->metadata.anti_spoofing.is_live = false;
            return false; // Stop pipeline for this frame
        }
        
        // Get depth at center
        auto [width, height] = frame->get_dimensions();
        float center_depth = frame->get_depth_at(width/2, height/2);
        
        // Mock scoring: frames with center depth in reasonable range get high score
        if (center_depth > 0.3f && center_depth < 3.0f) {
            frame->metadata.anti_spoofing.overall_liveness_score = 0.95f;
            frame->metadata.anti_spoofing.is_live = true;
            frame->metadata.is_ready_for_processing = true;
        } else {
            frame->metadata.anti_spoofing.overall_liveness_score = 0.3f;
            frame->metadata.anti_spoofing.is_live = false;
            frame->metadata.is_ready_for_processing = false;
        }
        
        frame->metadata.anti_spoofing_processed = true;
        
        // Notify that anti-spoofing stage is complete
        frame->notify_stage_complete("anti_spoofing");
        
        std::cout << "[AntiSpoofing] Frame " << frame->sequence_id 
                  << " - Score: " << frame->metadata.anti_spoofing.overall_liveness_score
                  << " - Valid: " << (frame->metadata.is_ready_for_processing ? "YES" : "NO")
                  << " - Depth: " << center_depth << "m" << std::endl;
        
        // Continue to next stage only if valid
        return frame->metadata.is_ready_for_processing;
    }
    
    std::string get_name() const override {
        return "AntiSpoofingStage";
    }
    
    void on_start() override {
        std::cout << "[AntiSpoofing] Stage started" << std::endl;
    }
    
    void on_stop() override {
        std::cout << "[AntiSpoofing] Stage stopped" << std::endl;
    }
};

// Example Face Detection Stage (runs only if anti-spoofing passes)
class FaceDetectionStage : public PipelineStage {
public:
    bool process(FrameBox* frame) override {
        // This stage only runs if anti-spoofing passed
        
        // Wait for anti-spoofing to complete (should already be done in this pipeline)
        if (!frame->wait_for_stage("anti_spoofing", 1000)) {
            std::cout << "[FaceDetection] Timeout waiting for anti-spoofing" << std::endl;
            return false;
        }
        
        // Simulate face detection
        // In real implementation, this would run face detection model
        
        frame->metadata.detected_face_id = "person_" + std::to_string(frame->sequence_id % 10);
        frame->metadata.face_confidence = 0.88f;
        
        std::cout << "[FaceDetection] Frame " << frame->sequence_id 
                  << " - Detected: " << frame->metadata.detected_face_id
                  << " - Confidence: " << frame->metadata.face_confidence << std::endl;
        
        frame->notify_stage_complete("face_detection");
        return true;
    }
    
    std::string get_name() const override {
        return "FaceDetectionStage";
    }
};

// Example Logging Stage (independent pipeline)
class LoggingStage : public PipelineStage {
private:
    std::string output_dir_;
    uint64_t saved_count_ = 0;
    
public:
    LoggingStage(const std::string& output_dir) : output_dir_(output_dir) {}
    
    bool process(FrameBox* frame) override {
        // This runs in a separate pipeline, independently
        // Save every 30th frame
        if (frame->sequence_id % 30 == 0) {
            // Use unified access system to save frame
            if (frame->get_depth_ptr() && frame->get_color_ptr()) {
                utils::save_framebox(*frame, output_dir_, "logged");
                saved_count_++;
                std::cout << "[Logging] Saved frame " << frame->sequence_id 
                          << " (total saved: " << saved_count_ << ")" << std::endl;
            }
        }
        return true;
    }
    
    std::string get_name() const override {
        return "LoggingStage";
    }
    
    void on_start() override {
        utils::ensure_directory_exists(output_dir_);
        std::cout << "[Logging] Stage started, output dir: " << output_dir_ << std::endl;
    }
};

int main(int argc, char** argv) {
    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "=== MDai RealSense Pipeline Example ===" << std::endl;
    std::cout << "This example demonstrates:" << std::endl;
    std::cout << "  - Pipeline X: Anti-spoofing → Face Detection (blocking within pipeline)" << std::endl;
    std::cout << "  - Pipeline Y: Logging (runs independently)" << std::endl;
    std::cout << "\nPress Ctrl+C to stop\n" << std::endl;
    
    // Create ring buffer
    DynamicRingBuffer ring_buffer(32, 5ULL * 1024 * 1024 * 1024);
    
    // Configure camera
    CameraConfig config;
    config.depth_width = 848;
    config.depth_height = 480;
    config.depth_fps = 30;
    config.color_width = 848;
    config.color_height = 480;
    config.color_fps = 30;
    config.enable_ir = true;
    config.internal_queue_size = 2;
    config.emitter_enabled = 1;
    config.auto_exposure = true;
    
    // Create producer
    Producer producer(config, &ring_buffer);
    
    // Set callbacks
    producer.set_error_callback([](const std::string& error) {
        std::cerr << "[ERROR] " << error << std::endl;
    });
    
    producer.set_status_callback([](const std::string& status) {
        std::cout << "[STATUS] " << status << std::endl;
    });
    
    // Start recording mode
    ring_buffer.set_recording_active(true);
    
    // Create Pipeline X: Anti-spoofing → Face Detection
    auto pipeline_x = std::make_unique<Pipeline>("AntiSpoofing_Pipeline", 
                                                   &ring_buffer, 
                                                   ConsumerMode::REALTIME);
    pipeline_x->add_stage(std::make_unique<AntiSpoofingStage>());
    pipeline_x->add_stage(std::make_unique<FaceDetectionStage>());
    
    // Create Pipeline Y: Logging (independent)
    auto pipeline_y = std::make_unique<Pipeline>("Logging_Pipeline",
                                                   &ring_buffer,
                                                   ConsumerMode::SEQUENTIAL);
    pipeline_y->add_stage(std::make_unique<LoggingStage>("./output"));
    
    // Create pipeline manager
    PipelineManager manager(&ring_buffer);
    manager.add_pipeline(std::move(pipeline_x));
    manager.add_pipeline(std::move(pipeline_y));
    
    // Start producer
    if (!producer.start()) {
        std::cerr << "Failed to start producer" << std::endl;
        return 1;
    }
    
    // Start all pipelines
    manager.start_all();
    
    std::cout << "\n=== System Running ===" << std::endl;
    std::cout << "Producer and pipelines started successfully" << std::endl;
    
    // Main loop - print statistics every 2 seconds
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::cout << "\n=== Statistics ===" << std::endl;
        std::cout << "Producer:" << std::endl;
        std::cout << "  FPS: " << producer.get_fps() << std::endl;
        std::cout << "  Total captured: " << producer.get_total_frames_captured() << std::endl;
        std::cout << "  Camera connected: " << (producer.is_camera_connected() ? "YES" : "NO") << std::endl;
        
        std::cout << "\nRing Buffer:" << std::endl;
        std::cout << "  Capacity: " << ring_buffer.get_capacity() << " slots" << std::endl;
        std::cout << "  Usage: " << ring_buffer.get_usage() << " / " << ring_buffer.get_capacity() << std::endl;
        std::cout << "  Memory: " << (ring_buffer.get_memory_usage() / (1024*1024)) << " MB" << std::endl;
        std::cout << "  Total written: " << ring_buffer.get_total_frames_written() << std::endl;
        std::cout << "  Total dropped: " << ring_buffer.get_total_frames_dropped() << std::endl;
        std::cout << "  Growth count: " << ring_buffer.get_growth_count() << std::endl;
        
        for (const auto& pipeline : manager.get_pipelines()) {
            std::cout << "\nPipeline '" << pipeline->get_name() << "':" << std::endl;
            std::cout << "  Processed: " << pipeline->get_frames_processed() << std::endl;
            std::cout << "  Dropped: " << pipeline->get_frames_dropped() << std::endl;
        }
        
        std::cout << "===================" << std::endl;
    }
    
    // Cleanup
    std::cout << "\nStopping system..." << std::endl;
    manager.stop_all();
    producer.stop();
    ring_buffer.set_recording_active(false);
    ring_buffer.clear();
    
    std::cout << "System stopped successfully" << std::endl;
    
    return 0;
}

