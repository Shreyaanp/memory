#include "FrameBox.hpp"
#include "RingBuffer.hpp"
#include "Producer.hpp"
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

// Very slow consumer to stress the buffer
void stress_consumer(DynamicRingBuffer* buffer, const std::string& name, int delay_ms) {
    uint64_t last_seq = 0;
    uint64_t processed_count = 0;
    
    std::cout << "[Consumer " << name << "] Started (delay: " << delay_ms << "ms)" << std::endl;
    
    while (g_running.load()) {
        FrameBox* frame = buffer->get_next_frame(last_seq);
        
        if (frame) {
            processed_count++;
            last_seq = frame->sequence_id;
            
            // Log every 10 frames (since processing is slow)
            if (processed_count % 10 == 0) {
                std::cout << "[Consumer " << name << "] Processed " << processed_count 
                          << " frames" << std::endl;
            }
            
            buffer->release_frame(frame);
            
            // Simulate very slow processing
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    std::cout << "[Consumer " << name << "] Stopped. Total: " << processed_count << std::endl;
}

int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "╔═══════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Buffer Stress Test                                  ║" << std::endl;
    std::cout << "╚═══════════════════════════════════════════════════════╝" << std::endl;
    std::cout << "\nThis test will:" << std::endl;
    std::cout << "  1. Start producer at 30fps (900 frames/minute)" << std::endl;
    std::cout << "  2. Add very slow consumers (100-500ms processing)" << std::endl;
    std::cout << "  3. Test buffer growth under extreme load" << std::endl;
    std::cout << "  4. Monitor memory usage and growth patterns" << std::endl;
    std::cout << "\nPress Ctrl+C to stop\n" << std::endl;
    
    // Create ring buffer with small initial size to force rapid growth
    std::cout << "Creating ring buffer: 16 initial slots, 5GB max..." << std::endl;
    DynamicRingBuffer ring_buffer(16, 5ULL * 1024 * 1024 * 1024);
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
    
    // Create producer
    Producer producer(config, &ring_buffer);
    
    producer.set_error_callback([](const std::string& error) {
        std::cerr << "[Producer Error] " << error << std::endl;
    });
    
    // Start producer
    std::cout << "Starting camera..." << std::endl;
    if (!producer.start()) {
        std::cerr << "Failed to start camera!" << std::endl;
        return 1;
    }
    
    std::cout << "\n✓ Camera started successfully!\n" << std::endl;
    
    // Start very slow consumers to stress the buffer
    std::thread consumer1(stress_consumer, &ring_buffer, "Slow1", 200);
    std::thread consumer2(stress_consumer, &ring_buffer, "Slow2", 300);
    std::thread consumer3(stress_consumer, &ring_buffer, "Slow3", 500);
    
    std::cout << "Started 3 very slow consumers to stress the buffer\n" << std::endl;
    
    // Monitor system for 45 seconds
    auto start_time = std::chrono::steady_clock::now();
    while (g_running.load() && 
           std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::steady_clock::now() - start_time).count() < 45) {
        
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time).count();
        
        std::cout << "\n╔═══════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║  Stress Test Statistics (T+" << std::setw(2) << elapsed << "s)                    ║" << std::endl;
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
        
        // Calculate growth rate
        float growth_rate = (float)ring_buffer.get_growth_count() / elapsed;
        std::cout << "║   Growth rate: " << std::setw(8) << std::fixed << std::setprecision(2) 
                  << growth_rate << " /sec                      ║" << std::endl;
        std::cout << "╚═══════════════════════════════════════════════════════╝" << std::endl;
    }
    
    // Cleanup
    std::cout << "\nStopping system..." << std::endl;
    g_running.store(false);
    
    if (consumer1.joinable()) consumer1.join();
    if (consumer2.joinable()) consumer2.join();
    if (consumer3.joinable()) consumer3.join();
    
    producer.stop();
    
    std::cout << "\n╔═══════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Stress Test Results                                 ║" << std::endl;
    std::cout << "╠═══════════════════════════════════════════════════════╣" << std::endl;
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
    
    size_t final_mem_mb = ring_buffer.get_memory_usage() / (1024 * 1024);
    std::cout << "║ Final memory usage:    " << std::setw(8) << final_mem_mb 
              << " MB                ║" << std::endl;
    std::cout << "╚═══════════════════════════════════════════════════════╝" << std::endl;
    
    std::cout << "\nStress test complete!" << std::endl;
    
    return 0;
}

