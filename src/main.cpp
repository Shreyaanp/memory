#include "SystemController.hpp"
#include <iostream>
#include <csignal>
#include <filesystem>
#include <librealsense2/rs.hpp>
#include <execinfo.h>  // For backtrace
#include <cxxabi.h>    // For demangling
#include <fstream>
#include <ctime>
#include <atomic>
#include <thread>
#include <chrono>
#include <exception>   // For std::set_terminate

std::unique_ptr<mdai::SystemController> controller;
std::atomic<bool> shutdown_requested{false};
std::atomic<bool> shutdown_complete{false};

// Global terminate handler - catches uncaught exceptions from threads
void terminate_handler() {
    // Try to get exception info
    std::exception_ptr eptr = std::current_exception();
    
    // Write to crash log immediately (before anything else)
    std::ofstream crash_log("/home/mercleDev/mdai_logs/crash_details.log", std::ios::app);
    
    time_t now = time(nullptr);
    char time_buf[64];
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", localtime(&now));
    
    crash_log << "\n========================================\n";
    crash_log << "UNCAUGHT EXCEPTION at " << time_buf << "\n";
    crash_log << "========================================\n";
    
    if (eptr) {
        try {
            std::rethrow_exception(eptr);
        } catch (const std::exception& e) {
            crash_log << "Exception: " << e.what() << "\n";
            std::cerr << "💥 UNCAUGHT EXCEPTION: " << e.what() << std::endl;
        } catch (...) {
            crash_log << "Exception: Unknown (non-std::exception)\n";
            std::cerr << "💥 UNCAUGHT EXCEPTION: Unknown type" << std::endl;
        }
    } else {
        crash_log << "Exception: std::terminate called without active exception\n";
        std::cerr << "💥 std::terminate called (no active exception)" << std::endl;
    }
    
    // Capture stack trace
    void* frames[64];
    int frame_count = backtrace(frames, 64);
    char** symbols = backtrace_symbols(frames, frame_count);
    
    if (symbols) {
        crash_log << "Stack trace (" << frame_count << " frames):\n";
        for (int i = 0; i < frame_count; i++) {
            crash_log << "  [" << i << "] " << symbols[i] << "\n";
        }
        free(symbols);
    }
    
    crash_log << "========================================\n";
    crash_log.close();
    
    std::cerr << "📁 Crash saved to: /home/mercleDev/mdai_logs/crash_details.log" << std::endl;
    std::cerr << "🔄 Service will auto-restart..." << std::endl;
    std::cerr.flush();
    
    // Exit with error code so systemd restarts
    _exit(1);
}

// Write crash info to dedicated crash log
void write_crash_log(int signum, const char* sig_name, void* crash_frames[], int frame_count) {
    std::ofstream crash_log("/home/mercleDev/mdai_logs/crash_details.log", std::ios::app);
    if (!crash_log.is_open()) return;
    
    // Timestamp
    time_t now = time(nullptr);
    char time_buf[64];
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", localtime(&now));
    
    crash_log << "\n========================================\n";
    crash_log << "CRASH at " << time_buf << "\n";
    crash_log << "Signal: " << signum << " (" << sig_name << ")\n";
    crash_log << "========================================\n";
    
    // Get symbol names
    char** symbols = backtrace_symbols(crash_frames, frame_count);
    if (symbols) {
        crash_log << "Stack trace (" << frame_count << " frames):\n";
        for (int i = 0; i < frame_count; i++) {
            crash_log << "  [" << i << "] " << symbols[i] << "\n";
        }
        free(symbols);
    }
    
    crash_log << "========================================\n";
    crash_log.close();
}

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
        case SIGBUS:
            sig_name = "SIGBUS (Bus Error)";
            is_crash = true;
            break;
    }
    
    if (signum == SIGINT || signum == SIGTERM) {
        // Normal shutdown - not a crash
        // Avoid duplicate handling if signal received multiple times
        if (shutdown_requested.exchange(true)) {
            std::cout << "\n⚠️ Shutdown already in progress, forcing exit..." << std::endl;
            std::cout.flush();
            _exit(signum);
            return;
        }
        
        std::cout << "\n🛑 Shutdown signal received (" << sig_name << ")" << std::endl;
        std::cout << "   Stopping gracefully..." << std::endl;
        std::cout.flush();
        
        if (controller) {
            controller->stop();
            
            // Wait for main loop to exit (with timeout)
            int wait_count = 0;
            const int MAX_WAIT_MS = 5000;  // 5 second timeout
            while (!shutdown_complete.load() && wait_count < MAX_WAIT_MS) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                wait_count += 10;
            }
            
            if (wait_count >= MAX_WAIT_MS) {
                std::cout << "   ⚠️ Shutdown timeout - forcing exit" << std::endl;
            } else {
                std::cout << "   ✅ Graceful shutdown complete" << std::endl;
            }
        }
        
        std::cout.flush();
        // Give a moment for final I/O to complete
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } else {
        // Actual crash - capture stack trace
        std::cerr << "\n" << std::endl;
        std::cerr << "╔══════════════════════════════════════════════════════════╗" << std::endl;
        std::cerr << "║                    💥 CRASH DETECTED                      ║" << std::endl;
        std::cerr << "╠══════════════════════════════════════════════════════════╣" << std::endl;
        std::cerr << "║  Signal: " << signum << " (" << sig_name << ")" << std::endl;
        std::cerr << "╚══════════════════════════════════════════════════════════╝" << std::endl;
        
        // Capture stack trace
        void* frames[64];
        int frame_count = backtrace(frames, 64);
        
        std::cerr << "\n📋 Stack trace:" << std::endl;
        char** symbols = backtrace_symbols(frames, frame_count);
        if (symbols) {
            for (int i = 0; i < frame_count && i < 20; i++) {
                std::cerr << "  [" << i << "] " << symbols[i] << std::endl;
            }
            if (frame_count > 20) {
                std::cerr << "  ... (" << (frame_count - 20) << " more frames)" << std::endl;
            }
            free(symbols);
        }
        
        // Write to crash log file
        write_crash_log(signum, sig_name, frames, frame_count);
        
        std::cerr << "\n📁 Crash details saved to: /home/mercleDev/mdai_logs/crash_details.log" << std::endl;
        std::cerr << "🔄 Service will auto-restart..." << std::endl;
        std::cerr.flush();
    }
    
    // For crashes, use _exit to avoid further signal handling issues
    if (is_crash) {
        _exit(signum);
    } else {
        exit(signum);
    }
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

int main(int argc, char* argv[]) {
    (void)argc;  // Unused
    (void)argv;  // Unused
    
    setup_logging();
    
    // Set global terminate handler for uncaught exceptions (CRITICAL for thread safety)
    std::set_terminate(terminate_handler);
    
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
    signal(SIGBUS, signal_handler);   // Catch bus errors

    std::cout << "Initializing MDai RDK X5 System..." << std::endl;
    
    // Debug: Check camera orientation before starting
    debug_camera_orientation();

    controller = std::make_unique<mdai::SystemController>();
    
    if (controller->initialize()) {
        std::cout << "System initialized. Starting main loop..." << std::endl;
        controller->run();
        
        // Signal that main loop has exited
        shutdown_complete.store(true);
        
        // Brief delay to allow signal handler to complete if it's waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Check if restart was requested
        bool should_restart = controller->is_restart_requested();
        
        std::cout << "Main loop exited." << std::endl;
        
        // Explicitly reset controller to trigger cleanup before exit
        std::cout << "Cleaning up resources..." << std::endl;
        controller.reset();
        std::cout << "Cleanup complete." << std::endl;
        
        if (should_restart) {
            std::cout << "🔄 Restart requested - exiting with code 0 (systemd will restart)" << std::endl;
            return 0;  // Exit code 0 with Restart=always will still restart
        }
        
        return 0;
    } else {
        std::cerr << "System initialization failed!" << std::endl;
        // Exit with error code - systemd will restart due to Restart=always
        return 1;
    }
}
