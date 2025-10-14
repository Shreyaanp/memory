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

// Fast consumer for testing
void fast_consumer(DynamicRingBuffer* buffer, const std::string& name) {
    uint64_t last_seq = 0;
    uint64_t processed_count = 0;
    
    std::cout << "[Consumer " << name << "] Started" << std::endl;
    
    while (g_running.load()) {
        FrameBox* frame = buffer->get_next_frame(last_seq);
        
        if (frame) {
            processed_count++;
            last_seq = frame->sequence_id;
            
            // Log every 30 frames
            if (processed_count % 30 == 0) {
                std::cout << "[Consumer " << name << "] Processed " << processed_count 
                          << " frames" << std::endl;
            }
            
            buffer->release_frame(frame);
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
    std::cout << "║  Recording Mode Switching Test                       ║" << std::endl;
    std::cout << "╚═══════════════════════════════════════════════════════╝" << std::endl;
    std::cout << "\nThis test will:" << std::endl;
    std::cout << "  1. Start with recording OFF (should drop frames)" << std::endl;
    std::cout << "  2. Switch to recording ON (should grow buffer)" << std::endl;
    std::cout << "  3. Switch back to OFF (should drop again)" << std::endl;
    std::cout << "  4. Repeat multiple times" << std::endl;
    std::cout << "\nPress Ctrl+C to stop\n" << std::endl;
    
    // Create ring buffer with small initial size to force growth
    std::cout << "Creating ring buffer: 8 initial slots, 5GB max..." << std::endl;
    DynamicRingBuffer ring_buffer(8, 5ULL * 1024 * 1024 * 1024);
    
    // Start with recording OFF
    ring_buffer.set_recording_active(false);
    std::cout << "Initial state: Recording OFF (should drop frames)" << std::endl;
    
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
    
    // Start fast consumer
    std::thread consumer_thread(fast_consumer, &ring_buffer, "Fast");
    
    // Test sequence
    for (int cycle = 1; cycle <= 3 && g_running.load(); cycle++) {
        std::cout << "\n═══ CYCLE " << cycle << " ═══" << std::endl;
        
        // Phase 1: Recording OFF (5 seconds)
        std::cout << "Phase 1: Recording OFF (5 seconds) - should drop frames" << std::endl;
        ring_buffer.set_recording_active(false);
        
        auto start_time = std::chrono::steady_clock::now();
        while (g_running.load() && 
               std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::steady_clock::now() - start_time).count() < 5) {
            
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            std::cout << "  Capacity: " << std::setw(4) << ring_buffer.get_capacity()
                      << " slots, Written: " << std::setw(4) << ring_buffer.get_total_frames_written()
                      << ", Dropped: " << std::setw(4) << ring_buffer.get_total_frames_dropped()
                      << ", Growth: " << std::setw(2) << ring_buffer.get_growth_count() << std::endl;
        }
        
        if (!g_running.load()) break;
        
        // Phase 2: Recording ON (10 seconds)
        std::cout << "Phase 2: Recording ON (10 seconds) - should grow buffer" << std::endl;
        ring_buffer.set_recording_active(true);
        
        start_time = std::chrono::steady_clock::now();
        while (g_running.load() && 
               std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::steady_clock::now() - start_time).count() < 10) {
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            
            std::cout << "  Capacity: " << std::setw(4) << ring_buffer.get_capacity()
                      << " slots, Written: " << std::setw(4) << ring_buffer.get_total_frames_written()
                      << ", Dropped: " << std::setw(4) << ring_buffer.get_total_frames_dropped()
                      << ", Growth: " << std::setw(2) << ring_buffer.get_growth_count() << std::endl;
        }
        
        if (!g_running.load()) break;
        
        // Phase 3: Recording OFF again (3 seconds)
        std::cout << "Phase 3: Recording OFF (3 seconds) - should drop frames again" << std::endl;
        ring_buffer.set_recording_active(false);
        
        start_time = std::chrono::steady_clock::now();
        while (g_running.load() && 
               std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::steady_clock::now() - start_time).count() < 3) {
            
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            std::cout << "  Capacity: " << std::setw(4) << ring_buffer.get_capacity()
                      << " slots, Written: " << std::setw(4) << ring_buffer.get_total_frames_written()
                      << ", Dropped: " << std::setw(4) << ring_buffer.get_total_frames_dropped()
                      << ", Growth: " << std::setw(2) << ring_buffer.get_growth_count() << std::endl;
        }
    }
    
    // Cleanup
    std::cout << "\nStopping system..." << std::endl;
    g_running.store(false);
    
    if (consumer_thread.joinable()) {
        consumer_thread.join();
    }
    
    producer.stop();
    
    std::cout << "\n╔═══════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Final Results                                      ║" << std::endl;
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
    std::cout << "╚═══════════════════════════════════════════════════════╝" << std::endl;
    
    std::cout << "\nTest complete!" << std::endl;
    
    return 0;
}
