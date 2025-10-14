#include "FrameBox.hpp"
#include "RingBuffer.hpp"
#include "Producer.hpp"
#include "CameraInitializer.hpp"
#include <iostream>
#include <signal.h>
#include <thread>
#include <iomanip>
#include <chrono>

using namespace mdai;

std::atomic<bool> g_running{true};

void signal_handler(int /* signum */) {
    std::cout << "\nShutting down..." << std::endl;
    g_running.store(false);
}

// Consumer to process frames
void frame_consumer(DynamicRingBuffer* buffer, const std::string& name) {
    uint64_t last_seq = 0;
    uint64_t processed_count = 0;
    uint64_t valid_count = 0;
    
    std::cout << "[Consumer " << name << "] Started" << std::endl;
    
    while (g_running.load()) {
        FrameBox* frame = buffer->get_next_frame(last_seq);
        
        if (frame) {
            processed_count++;
            
            // Check if frame has valid data
            if (!frame->depth_data.empty() && !frame->color_data.empty()) {
                valid_count++;
                
                // Log every 30 frames (1 second at 30fps)
                if (processed_count % 30 == 0) {
                    auto [width, height] = frame->get_dimensions();
                    float center_depth = frame->get_depth_at(width/2, height/2);
                    
                    std::cout << "[Consumer " << name << "] Processed " << processed_count 
                              << " frames, " << valid_count << " valid, "
                              << "center_depth=" << std::fixed << std::setprecision(2) 
                              << center_depth << "m" << std::endl;
                }
            }
            
            last_seq = frame->sequence_id;
            buffer->release_frame(frame);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    std::cout << "[Consumer " << name << "] Stopped. Total: " << processed_count 
              << " frames, " << valid_count << " valid" << std::endl;
}

int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "╔═══════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Camera Initialization & Stabilization Test         ║" << std::endl;
    std::cout << "╚═══════════════════════════════════════════════════════╝" << std::endl;
    std::cout << "\nThis test demonstrates:" << std::endl;
    std::cout << "  1. Proper camera initialization with stabilization" << std::endl;
    std::cout << "  2. Sensor warm-up (2 seconds)" << std::endl;
    std::cout << "  3. Autofocus stabilization (3 seconds)" << std::endl;
    std::cout << "  4. Exposure stabilization (2 seconds)" << std::endl;
    std::cout << "  5. Final calibration (1 second)" << std::endl;
    std::cout << "  6. Continuous capture with proper focus" << std::endl;
    std::cout << "\nPress Ctrl+C to stop\n" << std::endl;
    
    // Create ring buffer
    std::cout << "Creating ring buffer: 32 initial slots, 5GB max..." << std::endl;
    DynamicRingBuffer ring_buffer(32, 5ULL * 1024 * 1024 * 1024);
    ring_buffer.set_recording_active(true);
    
    // Configure camera
    CameraConfig config;
    config.depth_width = 848;
    config.depth_height = 480;
    config.depth_fps = 30;
    config.color_width = 848;
    config.color_height = 480;
    config.color_fps = 30;
    config.enable_ir = false;
    config.internal_queue_size = 16;
    config.emitter_enabled = 1;
    config.auto_exposure = true;
    
    // Create producer with proper initialization
    Producer producer(config, &ring_buffer);
    
    producer.set_error_callback([](const std::string& error) {
        std::cerr << "[Producer Error] " << error << std::endl;
    });
    
    producer.set_status_callback([](const std::string& status) {
        std::cout << "[Producer Status] " << status << std::endl;
    });
    
    // Start producer (this will now include proper camera initialization)
    std::cout << "Starting camera with proper initialization..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();
    
    if (!producer.start()) {
        std::cerr << "Failed to start camera!" << std::endl;
        return 1;
    }
    
    auto init_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time).count();
    
    std::cout << "\n✓ Camera started successfully!" << std::endl;
    std::cout << "  Initialization time: " << init_time << " ms" << std::endl;
    std::cout << "  Expected time: ~8000 ms (8 seconds)" << std::endl;
    
    // Start consumer
    std::thread consumer_thread(frame_consumer, &ring_buffer, "Stabilized");
    
    // Monitor system for 30 seconds
    std::cout << "\nMonitoring system for 30 seconds..." << std::endl;
    auto monitor_start = std::chrono::steady_clock::now();
    
    while (g_running.load() && 
           std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::steady_clock::now() - monitor_start).count() < 30) {
        
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - monitor_start).count();
        
        std::cout << "\n╔═══════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║  System Statistics (T+" << std::setw(2) << elapsed << "s)                    ║" << std::endl;
        std::cout << "╠═══════════════════════════════════════════════════════╣" << std::endl;
        std::cout << "║ PRODUCER                                              ║" << std::endl;
        std::cout << "║   FPS: " << std::setw(8) << std::fixed << std::setprecision(2) 
                  << producer.get_fps() << "                                      ║" << std::endl;
        std::cout << "║   Captured: " << std::setw(8) << producer.get_total_frames_captured() 
                  << " frames                            ║" << std::endl;
        std::cout << "║   Camera: " << (producer.is_camera_connected() ? "CONNECTED   " : "DISCONNECTED")
                  << "                                   ║" << std::endl;
        std::cout << "║                                                       ║" << std::endl;
        std::cout << "║ RING BUFFER                                           ║" << std::endl;
        std::cout << "║   Capacity: " << std::setw(6) << ring_buffer.get_capacity() 
                  << " slots                              ║" << std::endl;
        std::cout << "║   Usage: " << std::setw(6) << ring_buffer.get_usage() 
                  << " / " << std::setw(6) << ring_buffer.get_capacity() 
                  << " slots                      ║" << std::endl;
        
        size_t mem_mb = ring_buffer.get_memory_usage() / (1024 * 1024);
        std::cout << "║   Memory: " << std::setw(6) << mem_mb 
                  << " MB                                 ║" << std::endl;
        
        std::cout << "║   Total written: " << std::setw(8) << ring_buffer.get_total_frames_written() 
                  << "                        ║" << std::endl;
        std::cout << "║   Total dropped: " << std::setw(8) << ring_buffer.get_total_frames_dropped() 
                  << "                        ║" << std::endl;
        std::cout << "║   Growth count: " << std::setw(8) << ring_buffer.get_growth_count() 
                  << "                         ║" << std::endl;
        std::cout << "╚═══════════════════════════════════════════════════════╝" << std::endl;
    }
    
    // Cleanup
    std::cout << "\nStopping system..." << std::endl;
    g_running.store(false);
    
    if (consumer_thread.joinable()) {
        consumer_thread.join();
    }
    
    producer.stop();
    
    std::cout << "\n╔═══════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Final Results                                       ║" << std::endl;
    std::cout << "╠═══════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Camera initialization time: " << std::setw(8) << init_time 
              << " ms              ║" << std::endl;
    std::cout << "║ Total frames captured: " << std::setw(8) << producer.get_total_frames_captured() 
              << "                   ║" << std::endl;
    std::cout << "║ Total frames written:  " << std::setw(8) << ring_buffer.get_total_frames_written() 
              << "                   ║" << std::endl;
    std::cout << "║ Total frames dropped:  " << std::setw(8) << ring_buffer.get_total_frames_dropped() 
              << "                   ║" << std::endl;
    std::cout << "║ Buffer grew:           " << std::setw(8) << ring_buffer.get_growth_count() 
              << " times              ║" << std::endl;
    std::cout << "║ Final capacity:        " << std::setw(8) << ring_buffer.get_capacity() 
              << " slots              ║" << std::endl;
    std::cout << "╚═══════════════════════════════════════════════════════╝" << std::endl;
    
    std::cout << "\nCamera initialization test complete!" << std::endl;
    
    return 0;
}

