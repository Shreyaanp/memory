#include "FrameBox.hpp"
#include "RingBuffer.hpp"
#include "Producer.hpp"
#include "Utils.hpp"
#include <iostream>
#include <signal.h>

using namespace mdai;

std::atomic<bool> g_running{true};

void signal_handler(int signum) {
    std::cout << "\nShutting down..." << std::endl;
    g_running.store(false);
}

int main() {
    signal(SIGINT, signal_handler);
    
    std::cout << "=== Basic Capture Example ===" << std::endl;
    
    // Create ring buffer
    DynamicRingBuffer ring_buffer(32);
    
    // Configure camera
    CameraConfig config;
    config.depth_width = 848;
    config.depth_height = 480;
    config.color_width = 848;
    config.color_height = 480;
    config.depth_fps = 30;
    config.color_fps = 30;
    config.enable_ir = true;  // Enable IR for material-based spoof detection
    
    // Create producer
    Producer producer(config, &ring_buffer);
    
    // Start
    if (!producer.start()) {
        std::cerr << "Failed to start camera" << std::endl;
        return 1;
    }
    
    std::cout << "Camera started. Press Ctrl+C to stop." << std::endl;
    
    // Simple consumer - get and display latest frames
    int frames_displayed = 0;
    while (g_running.load() && frames_displayed < 100) {
        FrameBox* frame = ring_buffer.get_latest_frame();
        
        if (frame) {
            std::cout << "Frame " << frame->sequence_id 
                      << " - Depth: " << (frame->get_depth_ptr() ? "YES" : "NO")
                      << " - Color: " << (frame->get_color_ptr() ? "YES" : "NO")
                      << " - IR_L: " << (frame->ir_left_data.empty() ? "NO" : "YES")
                      << " - IR_R: " << (frame->ir_right_data.empty() ? "NO" : "YES")
                      << " - Size: " << frame->get_depth_vector().size() << " depth pixels"
                      << " - Dimensions: " << frame->depth_width << "x" << frame->depth_height
                      << std::endl;
            
            ring_buffer.release_frame(frame);
            frames_displayed++;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    producer.stop();
    std::cout << "Capture stopped. Total frames displayed: " << frames_displayed << std::endl;
    
    return 0;
}

