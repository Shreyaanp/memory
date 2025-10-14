#include "FrameBox.hpp"
#include "RingBuffer.hpp"
#include "Producer.hpp"
#include <iostream>
#include <signal.h>
#include <iomanip>
#include <thread>
#include <chrono>

using namespace mdai;

std::atomic<bool> g_running{true};

void signal_handler(int /* signum */) {
    std::cout << "\nShutting down..." << std::endl;
    g_running.store(false);
}

void print_stats(const Producer& producer, const DynamicRingBuffer& buffer) {
    std::cout << "\n╔═══════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Real-time Statistics                                 ║" << std::endl;
    std::cout << "╠═══════════════════════════════════════════════════════╣" << std::endl;
    
    // Producer stats
    std::cout << "║ PRODUCER                                              ║" << std::endl;
    std::cout << "║   FPS: " << std::setw(8) << std::fixed << std::setprecision(2) 
              << producer.get_fps() << "                                      ║" << std::endl;
    std::cout << "║   Captured: " << std::setw(8) << producer.get_total_frames_captured() 
              << " frames                            ║" << std::endl;
    std::cout << "║   Camera: " << (producer.is_camera_connected() ? "CONNECTED " : "DISCONNECTED")
              << "                                   ║" << std::endl;
    
    // Ring buffer stats
    std::cout << "║                                                       ║" << std::endl;
    std::cout << "║ RING BUFFER                                           ║" << std::endl;
    std::cout << "║   Capacity: " << std::setw(6) << buffer.get_capacity() 
              << " slots                              ║" << std::endl;
    std::cout << "║   Usage: " << std::setw(6) << buffer.get_usage() 
              << " / " << std::setw(6) << buffer.get_capacity() 
              << " slots                      ║" << std::endl;
    
    size_t mem_mb = buffer.get_memory_usage() / (1024 * 1024);
    std::cout << "║   Memory: " << std::setw(6) << mem_mb 
              << " MB                                 ║" << std::endl;
    
    std::cout << "║   Total written: " << std::setw(8) << buffer.get_total_frames_written() 
              << "                        ║" << std::endl;
    std::cout << "║   Total dropped: " << std::setw(8) << buffer.get_total_frames_dropped() 
              << "                        ║" << std::endl;
    std::cout << "║   Growth count: " << std::setw(8) << buffer.get_growth_count() 
              << "                         ║" << std::endl;
    
    std::cout << "╚═══════════════════════════════════════════════════════╝" << std::endl;
}

int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "╔═══════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Camera Capture & Buffer Memory Test                  ║" << std::endl;
    std::cout << "╚═══════════════════════════════════════════════════════╝" << std::endl;
    
    std::cout << "\nThis test will:" << std::endl;
    std::cout << "  1. Start capturing from RealSense D435" << std::endl;
    std::cout << "  2. Monitor ring buffer growth" << std::endl;
    std::cout << "  3. Test recording ON/OFF modes" << std::endl;
    std::cout << "  4. Verify no frame drops during recording" << std::endl;
    std::cout << "\nPress Ctrl+C to stop\n" << std::endl;
    
    // Create ring buffer - start small to test growth
    std::cout << "Creating ring buffer: 16 initial slots, 5GB max..." << std::endl;
    DynamicRingBuffer ring_buffer(16, 5ULL * 1024 * 1024 * 1024);
    
    // Configure camera
    std::cout << "Configuring camera..." << std::endl;
    CameraConfig config;
    config.depth_width = 848;
    config.depth_height = 480;
    config.depth_fps = 30;
    config.color_width = 848;
    config.color_height = 480;
    config.color_fps = 30;
    config.enable_ir = true;
    config.internal_queue_size = 16;  // Increased from 2 to 16
    config.emitter_enabled = 1;
    config.auto_exposure = true;
    
    // Create producer
    Producer producer(config, &ring_buffer);
    
    producer.set_error_callback([](const std::string& error) {
        std::cerr << "[ERROR] " << error << std::endl;
    });
    
    producer.set_status_callback([](const std::string& status) {
        std::cout << "[STATUS] " << status << std::endl;
    });
    
    // Start producer
    std::cout << "Starting camera..." << std::endl;
    if (!producer.start()) {
        std::cerr << "Failed to start camera!" << std::endl;
        std::cerr << "Please ensure:" << std::endl;
        std::cerr << "  - RealSense D435 is connected" << std::endl;
        std::cerr << "  - You have permissions (may need to add user to 'video' group)" << std::endl;
        return 1;
    }
    
    std::cout << "\n✓ Camera started successfully!\n" << std::endl;
    
    // Phase 1: Recording OFF (should drop frames)
    std::cout << "═══ PHASE 1: Recording OFF (10 seconds) ═══" << std::endl;
    std::cout << "Buffer should drop old frames if full\n" << std::endl;
    ring_buffer.set_recording_active(false);
    
    auto phase1_start = std::chrono::steady_clock::now();
    while (g_running.load()) {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - phase1_start).count();
        
        if (elapsed >= 10) break;
        
        print_stats(producer, ring_buffer);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    auto phase1_dropped = ring_buffer.get_total_frames_dropped();
    auto phase1_capacity = ring_buffer.get_capacity();
    
    bool test_passed = true;
    
    if (g_running.load()) {
        // Phase 2: Recording ON (20 seconds)
        std::cout << "\n═══ PHASE 2: Recording ON (20 seconds) ═══" << std::endl;
        std::cout << "Buffer should GROW instead of dropping frames\n" << std::endl;
        ring_buffer.set_recording_active(true);
        
        auto phase2_start = std::chrono::steady_clock::now();
        while (g_running.load()) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - phase2_start).count();
            
            if (elapsed >= 20) break;
            
            print_stats(producer, ring_buffer);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        
        auto phase2_dropped = ring_buffer.get_total_frames_dropped();
        auto phase2_capacity = ring_buffer.get_capacity();
        auto phase2_growth = ring_buffer.get_growth_count();
        
        // Final results
        std::cout << "\n╔═══════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║  TEST RESULTS                                         ║" << std::endl;
        std::cout << "╠═══════════════════════════════════════════════════════╣" << std::endl;
        
        std::cout << "║ Phase 1 (Recording OFF):                             ║" << std::endl;
        std::cout << "║   Capacity: " << std::setw(6) << phase1_capacity 
                  << " slots                              ║" << std::endl;
        std::cout << "║   Dropped: " << std::setw(8) << phase1_dropped 
                  << " frames                        ║" << std::endl;
        
        std::cout << "║                                                       ║" << std::endl;
        std::cout << "║ Phase 2 (Recording ON):                              ║" << std::endl;
        std::cout << "║   Capacity: " << std::setw(6) << phase2_capacity 
                  << " slots                              ║" << std::endl;
        std::cout << "║   Dropped: " << std::setw(8) << phase2_dropped 
                  << " frames (new)                  ║" << std::endl;
        std::cout << "║   Growth: " << std::setw(8) << phase2_growth 
                  << " times                          ║" << std::endl;
        
        std::cout << "║                                                       ║" << std::endl;
        
        // Verify results
        if (phase2_capacity > phase1_capacity) {
            std::cout << "║ ✓ Buffer grew during recording                        ║" << std::endl;
        } else {
            std::cout << "║ ✗ Buffer did NOT grow during recording                ║" << std::endl;
            test_passed = false;
        }
        
        if (phase2_dropped == phase1_dropped) {
            std::cout << "║ ✓ No frames dropped during recording                  ║" << std::endl;
        } else {
            std::cout << "║ ✗ Frames were dropped during recording!               ║" << std::endl;
            test_passed = false;
        }
        
        if (phase1_dropped > 0) {
            std::cout << "║ ✓ Drop-oldest worked when recording OFF              ║" << std::endl;
        } else {
            std::cout << "║ ⚠ No drops when recording OFF (buffer may be too big) ║" << std::endl;
        }
        
        std::cout << "║                                                       ║" << std::endl;
        
        if (test_passed) {
            std::cout << "║                  ✓✓✓ TEST PASSED ✓✓✓                  ║" << std::endl;
        } else {
            std::cout << "║                  ✗✗✗ TEST FAILED ✗✗✗                  ║" << std::endl;
        }
        
        std::cout << "╚═══════════════════════════════════════════════════════╝" << std::endl;
    }
    // Cleanup
    std::cout << "\nStopping system..." << std::endl;
    
    // Stop recording first
    ring_buffer.set_recording_active(false);
    
    // Stop the producer (this will stop the camera pipeline)
    producer.stop();
    
    // Clear the ring buffer
    ring_buffer.clear();
    
    // Give the system a moment to fully shutdown
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "Test complete!" << std::endl;
    
    return test_passed ? 0 : 1;
}

