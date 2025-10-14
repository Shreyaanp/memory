#include <librealsense2/rs.hpp>
#include <iostream>

int main() {
    std::cout << "=== Simple Camera Connection Test ===" << std::endl;
    
    try {
        rs2::context ctx;
        auto devices = ctx.query_devices();
        
        if (devices.size() == 0) {
            std::cout << "❌ No RealSense devices found" << std::endl;
            return 1;
        }
        
        std::cout << "✓ Found " << devices.size() << " RealSense device(s)" << std::endl;
        
        for (size_t i = 0; i < devices.size(); i++) {
            auto dev = devices[i];
            std::cout << "Device " << i << ": " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
            std::cout << "  Serial: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
            std::cout << "  Firmware: " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
        }
        
        // Try to create a pipeline
        rs2::pipeline pipe;
        rs2::config cfg;
        
        // Configure streams (D435 supports 848x480 for depth)
        cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_BGR8, 30);
        
        std::cout << "Attempting to start pipeline..." << std::endl;
        auto profile = pipe.start(cfg);
        
        std::cout << "✓ Pipeline started successfully!" << std::endl;
        
        // Get a few frames
        for (int i = 0; i < 5; i++) {
            auto frames = pipe.wait_for_frames(10000);
            auto depth = frames.get_depth_frame();
            auto color = frames.get_color_frame();
            
            std::cout << "Frame " << i << ": Depth " << depth.get_width() << "x" << depth.get_height() 
                      << ", Color " << color.get_width() << "x" << color.get_height() << std::endl;
        }
        
        pipe.stop();
        std::cout << "✓ Camera test completed successfully!" << std::endl;
        return 0;
        
    } catch (const rs2::error& e) {
        std::cout << "❌ RealSense error: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cout << "❌ Error: " << e.what() << std::endl;
        return 1;
    }
}
