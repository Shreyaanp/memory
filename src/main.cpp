#include "SystemController.hpp"
#include <iostream>
#include <csignal>
#include <filesystem>
#include <librealsense2/rs.hpp>

std::unique_ptr<mdai::SystemController> controller;

void signal_handler(int signum) {
    const char* sig_name = "UNKNOWN";
    bool is_crash = false;
    
    switch(signum) {
        case SIGINT: 
            sig_name = "SIGINT (Interrupt)"; 
            break;
        case SIGTERM: 
            sig_name = "SIGTERM (Terminate)"; 
            break;
        case SIGSEGV: 
            sig_name = "SIGSEGV (Segmentation Fault)"; 
            is_crash = true;
            break;
        case SIGABRT: 
            sig_name = "SIGABRT (Abort)"; 
            is_crash = true;
            break;
        case SIGFPE: 
            sig_name = "SIGFPE (Floating Point Exception)"; 
            is_crash = true;
            break;
    }
    
    if (signum == SIGINT || signum == SIGTERM) {
        // Normal shutdown - not a crash
        std::cout << "\n🛑 Shutdown signal received (" << sig_name << ")" << std::endl;
        std::cout << "   Stopping gracefully..." << std::endl;
        std::cout.flush();
        if (controller) {
            controller->stop();
        }
    } else {
        // Actual crash
        std::cerr << "\n❌ CRASH: SIGNAL " << signum << " (" << sig_name << ")" << std::endl;
        std::cerr << "   Check code for null pointers or memory issues." << std::endl;
        std::cerr.flush();
    }
    
    exit(signum);
}

// VERSION: RDK v1.0.0 - Initial production release with error recovery
const char* RDK_VERSION = "1.0.0";

// Debug: Check camera orientation via IMU
void debug_camera_orientation() {
    std::cout << "🔍 IMU DEBUG: Checking camera orientation..." << std::endl;
    try {
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 200);
        
        pipe.start(cfg);
        
        float ax = 0, ay = 0, az = 0;
        int samples = 0;
        auto start = std::chrono::steady_clock::now();
        
        while (samples < 30) {
            rs2::frameset frames;
            if (pipe.poll_for_frames(&frames)) {
                for (auto&& f : frames) {
                    if (auto m = f.as<rs2::motion_frame>()) {
                        if (m.get_profile().stream_type() == RS2_STREAM_ACCEL) {
                            auto d = m.get_motion_data();
                            ax += d.x; ay += d.y; az += d.z;
                            samples++;
                        }
                    }
                }
            }
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsed > 2000) break;
        }
        
        pipe.stop();
        
        if (samples > 0) {
            ax /= samples; ay /= samples; az /= samples;
            std::cout << "📊 IMU Accel: X=" << ax << " Y=" << ay << " Z=" << az << std::endl;
            
            float abs_x = std::abs(ax), abs_y = std::abs(ay);
            if (abs_y > abs_x && abs_y > 5) {
                std::cout << "📷 Orientation: " << (ay < 0 ? "NORMAL" : "UPSIDE_DOWN") << std::endl;
            } else if (abs_x > 5) {
                std::cout << "📷 Orientation: " << (ax < 0 ? "90° CW (USB RIGHT)" : "90° CCW (USB LEFT)") << std::endl;
            } else {
                std::cout << "📷 Orientation: UNKNOWN" << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cout << "⚠️  IMU check failed: " << e.what() << std::endl;
    }
}
// Persist logs outside of /tmp so they survive reboots
const char* RDK_LOG_FILE = "/home/mercleDev/mdai_logs/rdk.log";

// Redirect stdout/stderr to log file
void setup_logging() {
    // Ensure log directory exists before redirecting
    try {
        std::filesystem::path log_path(RDK_LOG_FILE);
        std::filesystem::create_directories(log_path.parent_path());
    } catch (const std::exception& e) {
        std::cerr << "Failed to create log directory: " << e.what() << std::endl;
    }

    FILE* result = freopen(RDK_LOG_FILE, "a", stdout);
    if (!result) {
        // Log redirection failed, but continue anyway
        perror("Failed to redirect stdout");
    }
    result = freopen(RDK_LOG_FILE, "a", stderr);
    if (!result) {
        // Log redirection failed, but continue anyway
        perror("Failed to redirect stderr");
    }
    // Make stdout/stderr unbuffered for immediate log writes
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}

int main() {
    setup_logging();
    
    std::cout << "\n╔════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   MDai RDK System v" << RDK_VERSION << "                   ║" << std::endl;
    std::cout << "║   Log: " << RDK_LOG_FILE << "           ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════╝" << std::endl;
    
    // Register signal handlers for graceful shutdown AND crash detection
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGSEGV, signal_handler);  // Catch segfaults
    signal(SIGABRT, signal_handler);  // Catch aborts
    signal(SIGFPE, signal_handler);   // Catch floating point errors

    std::cout << "Initializing MDai RDK X5 System..." << std::endl;
    
    // Debug: Check camera orientation before starting
    debug_camera_orientation();

    controller = std::make_unique<mdai::SystemController>();
    
    if (controller->initialize()) {
        std::cout << "System initialized. Starting main loop..." << std::endl;
        controller->run();
    } else {
        std::cerr << "System initialization failed!" << std::endl;
        return 1;
    }

    return 0;
}
