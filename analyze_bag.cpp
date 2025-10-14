#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>

int main() {
    try {
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_device_from_file("tt.bag");
        
        auto profile = pipe.start(cfg);
        auto device = profile.get_device();
        auto playback = device.as<rs2::playback>();
        
        std::cout << "=== BAG FILE ANALYSIS ===" << std::endl;
        std::cout << "File: tt.bag" << std::endl;
        std::cout << "Duration: " << playback.get_duration().count() / 1000000.0 << " seconds" << std::endl;
        
        // Get stream profiles
        auto streams = profile.get_streams();
        std::cout << "\n=== AVAILABLE STREAMS ===" << std::endl;
        for (auto& stream : streams) {
            std::cout << "Stream: " << stream.stream_name() 
                      << " (Type: " << stream.stream_type() << ")"
                      << " Format: " << stream.format()
                      << " Resolution: " << stream.as<rs2::video_stream_profile>().width() 
                      << "x" << stream.as<rs2::video_stream_profile>().height()
                      << " FPS: " << stream.fps() << std::endl;
        }
        
        // Get intrinsics
        std::cout << "\n=== INTRINSICS ===" << std::endl;
        for (auto& stream : streams) {
            if (stream.is<rs2::video_stream_profile>()) {
                auto intrinsics = stream.as<rs2::video_stream_profile>().get_intrinsics();
                std::cout << stream.stream_name() << ":" << std::endl;
                std::cout << "  fx: " << intrinsics.fx << ", fy: " << intrinsics.fy << std::endl;
                std::cout << "  ppx: " << intrinsics.ppx << ", ppy: " << intrinsics.ppy << std::endl;
                std::cout << "  distortion: " << intrinsics.model << std::endl;
            }
        }
        
        // Sample a few frames
        std::cout << "\n=== SAMPLE FRAMES ===" << std::endl;
        for (int i = 0; i < 5; i++) {
            auto frames = pipe.wait_for_frames();
            std::cout << "Frame " << i << ":" << std::endl;
            for (const auto& frame : frames) {
                std::cout << "  " << frame.get_profile().stream_name() 
                          << " - " << frame.get_profile().format()
                          << " - " << frame.get_timestamp() << "μs" << std::endl;
            }
        }
        
        pipe.stop();
        
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
