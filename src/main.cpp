#include "SystemController.hpp"
#include <iostream>
#include <csignal>

std::unique_ptr<mdai::SystemController> controller;

void signal_handler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Shutting down..." << std::endl;
    if (controller) {
        controller->stop();
    }
    exit(signum);
}

// VERSION: RDK v1.0.0 - Initial production release with error recovery
const char* RDK_VERSION = "1.0.0";
const char* RDK_LOG_FILE = "/tmp/mdai/rdk.log";

// Redirect stdout/stderr to log file
void setup_logging() {
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
    
    // Register signal handler for graceful shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    std::cout << "Initializing MDai RDK X5 System..." << std::endl;

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

