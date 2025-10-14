#include "FrameBox.hpp"
#include "RingBuffer.hpp"
#include "Producer.hpp"
#include <iostream>
#include <signal.h>
#include <thread>
#include <iomanip>

using namespace mdai;

std::atomic<bool> g_running{true};

void signal_handler(int /* signum */) {
    std::cout << "\nShutting down..." << std::endl;
    g_running.store(false);
}

// Mock anti-spoofing consumer
void anti_spoofing_consumer(DynamicRingBuffer* buffer) {
    uint64_t last_seq = 0;
    uint64_t processed_count = 0;
    uint64_t valid_count = 0;
    
    std::cout << "[Consumer] Anti-spoofing consumer started" << std::endl;
    
    while (g_running.load()) {
        FrameBox* frame = buffer->get_next_frame(last_seq);
        
        if (frame) {
            // Mock anti-spoofing: check center depth
            auto [width, height] = frame->get_dimensions();
            float center_depth = frame->get_depth_at(width/2, height/2);
            
            processed_count++;
            
            // Simulate anti-spoofing logic (0.5m to 2.0m is valid)
            bool is_valid = (center_depth >= 0.5f && center_depth <= 2.0f);
            if (is_valid) valid_count++;
            
            // Update metadata
            {
                std::lock_guard<std::mutex> lock(frame->metadata_mutex);
                frame->metadata.anti_spoofing_processed = true;
                frame->metadata.anti_spoofing.overall_liveness_score = is_valid ? 0.9f : 0.3f;
                frame->metadata.anti_spoofing.is_live = is_valid;
                frame->metadata.is_ready_for_processing = is_valid;
            }
            
            // Log every 30 frames (1 second at 30fps)
            if (processed_count % 30 == 0) {
                float validity_rate = 100.0f * valid_count / processed_count;
                std::cout << "[Consumer] Processed " << processed_count 
                          << " frames, " << valid_count << " valid (" 
                          << std::fixed << std::setprecision(1) << validity_rate << "%)"
                          << ", center_depth=" << center_depth << "m" << std::endl;
            }
            
            last_seq = frame->sequence_id;
            
            // IMPORTANT: Release frame when done
            buffer->release_frame(frame);
        } else {
            // No frame available, small sleep
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    std::cout << "[Consumer] Anti-spoofing consumer stopped. Total processed: " 
              << processed_count << std::endl;
}

int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "╔═══════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Camera Test with Mock Anti-Spoofing Consumer        ║" << std::endl;
    std::cout << "╚═══════════════════════════════════════════════════════╝" << std::endl;
    std::cout << "\nThis test demonstrates:" << std::endl;
    std::cout << "  1. Producer captures from D435" << std::endl;
    std::cout << "  2. Ring buffer stores frames" << std::endl;
    std::cout << "  3. Consumer processes frames (mock anti-spoofing)" << std::endl;
    std::cout << "  4. Frames are released back to buffer" << std::endl;
    std::cout << "\nPress Ctrl+C to stop\n" << std::endl;
    
    // Create ring buffer
    std::cout << "Creating ring buffer: 32 initial slots, 5GB max..." << std::endl;
    DynamicRingBuffer ring_buffer(32, 5ULL * 1024 * 1024 * 1024);
    
    // Start with recording active
    ring_buffer.set_recording_active(true);
    
    // Configure camera
    std::cout << "Configuring camera..." << std::endl;
    CameraConfig config;
    config.depth_width = 848;
    config.depth_height = 480;
    config.depth_fps = 30;
    config.color_width = 848;
    config.color_height = 480;
    config.color_fps = 30;
    config.enable_ir = false;  // Disable IR for simplicity
    config.internal_queue_size = 16;  // Larger queue
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
    
    // Start consumer thread
    std::thread consumer_thread(anti_spoofing_consumer, &ring_buffer);
    
    // Monitor statistics
    std::cout << "System running. Statistics updated every 5 seconds...\n" << std::endl;
    
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        std::cout << "\n╔═══════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║  System Statistics                                    ║" << std::endl;
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
    ring_buffer.set_recording_active(false);
    
    std::cout << "\n╔═══════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Final Statistics                                     ║" << std::endl;
    std::cout << "╠═══════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Total frames captured: " << std::setw(8) << producer.get_total_frames_captured() 
              << "                   ║" << std::endl;
    std::cout << "║ Total frames written:  " << std::setw(8) << ring_buffer.get_total_frames_written() 
              << "                   ║" << std::endl;
    std::cout << "║ Total frames dropped:  " << std::setw(8) << ring_buffer.get_total_frames_dropped() 
              << "                   ║" << std::endl;
    std::cout << "║ Buffer grew:           " << std::setw(8) << ring_buffer.get_growth_count() 
              << " times              ║" << std::endl;
    std::cout << "╚═══════════════════════════════════════════════════════╝" << std::endl;
    
    std::cout << "\nTest complete!" << std::endl;
    
    return 0;
}


