#include "FrameBox.hpp"
#include "RingBuffer.hpp"
#include "Producer.hpp"
#include "Utils.hpp"
#include <iostream>
#include <signal.h>

using namespace mdai;

std::atomic<bool> g_running{true};

void signal_handler(int signum) {
    g_running.store(false);
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    
    std::string output_dir = "./captured_frames";
    if (argc > 1) {
        output_dir = argv[1];
    }
    
    std::cout << "=== Frame Saver Example ===" << std::endl;
    std::cout << "Output directory: " << output_dir << std::endl;
    std::cout << "Press Ctrl+C to stop\n" << std::endl;
    
    utils::ensure_directory_exists(output_dir);
    
    // Create ring buffer
    DynamicRingBuffer ring_buffer(32);
    ring_buffer.set_recording_active(true);
    
    // Configure camera
    CameraConfig config;
    config.depth_width = 848;
    config.depth_height = 480;
    config.color_width = 848;
    config.color_height = 480;
    config.depth_fps = 30;
    config.color_fps = 30;
    config.enable_ir = true;
    
    // Create producer
    Producer producer(config, &ring_buffer);
    
    if (!producer.start()) {
        std::cerr << "Failed to start camera" << std::endl;
        return 1;
    }
    
    std::cout << "Recording started. Saving frames..." << std::endl;
    
    // Consumer - save frames periodically
    uint64_t last_saved_seq = 0;
    int save_interval = 30;  // Save every 30 frames (1 second at 30fps)
    int frames_saved = 0;
    
    while (g_running.load()) {
        FrameBox* frame = ring_buffer.get_next_frame(last_saved_seq);
        
        if (frame) {
            // Save frame
            if (utils::save_framebox(*frame, output_dir, "frame")) {
                frames_saved++;
                std::cout << "Saved frame " << frame->sequence_id 
                          << " (total: " << frames_saved << ")" << std::endl;
            } else {
                std::cerr << "Failed to save frame " << frame->sequence_id << std::endl;
            }
            
            last_saved_seq = frame->sequence_id;
            
            // Wait for next save interval
            for (int i = 0; i < save_interval && g_running.load(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
            }
            
            ring_buffer.release_frame(frame);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    producer.stop();
    
    std::cout << "\nCapture stopped" << std::endl;
    std::cout << "Total frames saved: " << frames_saved << std::endl;
    std::cout << "Files saved to: " << output_dir << std::endl;
    
    return 0;
}


