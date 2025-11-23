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

int main() {
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

