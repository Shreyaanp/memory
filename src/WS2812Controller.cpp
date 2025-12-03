/**
 * @file WS2812Controller.cpp
 * @brief WS2812 LED Ring Controller Implementation
 * 
 * Uses SPI to control WS2812 LEDs on RDK X5 (pin 19 MOSI)
 * All timing uses non-blocking code with chrono (like millis())
 */

#include "WS2812Controller.hpp"
#include "SystemController.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <cmath>
#include <iostream>

namespace mdai {

WS2812Controller::WS2812Controller(int led_count)
    : spi_fd_(-1)
    , led_count_(led_count)
    , initialized_(false)
    , current_animation_(LEDAnimation::OFF)
    , animation_color_(Colors::OFF)
    , animation_param1_(0)
    , animation_param2_(0)
    , animation_param3_(0)
    , blink_count_(0)
    , blink_state_(false)
{
    pixel_buffer_.fill(Colors::OFF);
}

WS2812Controller::~WS2812Controller() {
    stop();
}

bool WS2812Controller::initialize() {
    if (initialized_) return true;
    
    char spi_path[32];
    snprintf(spi_path, sizeof(spi_path), "/dev/spidev%d.%d", SPI_BUS, SPI_DEVICE);
    
    spi_fd_ = open(spi_path, O_RDWR);
    if (spi_fd_ < 0) {
        std::cerr << "❌ WS2812: Failed to open SPI device " << spi_path << std::endl;
        return false;
    }
    
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = SPI_SPEED_HZ;
    
    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        std::cerr << "❌ WS2812: Failed to configure SPI" << std::endl;
        close(spi_fd_);
        spi_fd_ = -1;
        return false;
    }
    
    initialized_ = true;
    animation_start_ = std::chrono::steady_clock::now();
    last_update_ = animation_start_;
    last_blink_toggle_ = animation_start_;
    
    off();
    
    // Start dedicated animation thread for smooth timing
    thread_running_ = true;
    animation_thread_ = std::thread(&WS2812Controller::animation_loop, this);
    
    std::cout << "✅ WS2812: Initialized " << led_count_ << " LEDs on " << spi_path << " (threaded)" << std::endl;
    return true;
}

void WS2812Controller::stop() {
    // Stop animation thread first
    thread_running_ = false;
    if (animation_thread_.joinable()) {
        animation_thread_.join();
    }
    
    if (initialized_) {
        std::lock_guard<std::mutex> lock(spi_mutex_);
        off();
        send_pixels();
    }
    if (spi_fd_ >= 0) {
        close(spi_fd_);
        spi_fd_ = -1;
    }
    initialized_ = false;
}

void WS2812Controller::encode_byte(uint8_t byte, uint8_t* output) {
    for (int i = 0; i < 8; i++) {
        bool bit = (byte >> (7 - i)) & 1;
        output[i] = bit ? 0xF8 : 0xE0;
    }
}

void WS2812Controller::send_pixels() {
    if (!initialized_ || spi_fd_ < 0) return;
    
    const int data_size = led_count_ * 24 + 100;
    uint8_t* spi_data = new uint8_t[data_size];
    
    int offset = 0;
    for (int i = 0; i < led_count_; i++) {
        encode_byte(pixel_buffer_[i].g, &spi_data[offset]); offset += 8;
        encode_byte(pixel_buffer_[i].r, &spi_data[offset]); offset += 8;
        encode_byte(pixel_buffer_[i].b, &spi_data[offset]); offset += 8;
    }
    
    memset(&spi_data[offset], 0x00, 100);
    
    struct spi_ioc_transfer tr = {};
    tr.tx_buf = reinterpret_cast<unsigned long>(spi_data);
    tr.len = data_size;
    tr.speed_hz = SPI_SPEED_HZ;
    tr.bits_per_word = 8;
    
    ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    
    delete[] spi_data;
}

void WS2812Controller::set_pixel(int index, const RGBColor& color) {
    if (index >= 0 && index < led_count_) {
        pixel_buffer_[index] = color;
    }
}

void WS2812Controller::fill_pixels(const RGBColor& color) {
    for (int i = 0; i < led_count_; i++) {
        pixel_buffer_[i] = color;
    }
}

void WS2812Controller::clear_pixels() {
    fill_pixels(Colors::OFF);
}

float WS2812Controller::get_animation_time_ms() const {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<float, std::milli>(now - animation_start_).count();
}

// ========================================
// Animation setup methods
// ========================================

void WS2812Controller::off() {
    current_animation_ = LEDAnimation::OFF;
    clear_pixels();
    if (initialized_) send_pixels();
}

void WS2812Controller::solid(const RGBColor& color) {
    current_animation_ = LEDAnimation::SOLID;
    animation_color_ = color;
    fill_pixels(color);
    if (initialized_) send_pixels();
}

void WS2812Controller::breathing(const RGBColor& color, int cycle_ms) {
    current_animation_ = LEDAnimation::BREATHING;
    animation_color_ = color;
    animation_param1_ = cycle_ms;
    animation_start_ = std::chrono::steady_clock::now();
}

void WS2812Controller::single_rotate(const RGBColor& color, int rotation_ms) {
    current_animation_ = LEDAnimation::SINGLE_ROTATE;
    animation_color_ = color;
    animation_param1_ = rotation_ms;
    animation_start_ = std::chrono::steady_clock::now();
}

void WS2812Controller::blink_n_times(const RGBColor& color, int count, int on_ms, int off_ms) {
    current_animation_ = LEDAnimation::BLINK_N_TIMES;
    animation_color_ = color;
    animation_param1_ = count;
    animation_param2_ = on_ms;
    animation_param3_ = off_ms;
    blink_count_ = 0;
    blink_state_ = true;  // Start with ON
    animation_start_ = std::chrono::steady_clock::now();
    last_blink_toggle_ = animation_start_;
    fill_pixels(color);
    if (initialized_) send_pixels();
}

void WS2812Controller::alternate_rotate(const RGBColor& color, int rotation_ms) {
    current_animation_ = LEDAnimation::ALTERNATE_ROTATE;
    animation_color_ = color;
    animation_param1_ = rotation_ms;
    animation_start_ = std::chrono::steady_clock::now();
}

void WS2812Controller::group_rotate(const RGBColor& color, int group_size, int rotation_ms) {
    current_animation_ = LEDAnimation::GROUP_ROTATE;
    animation_color_ = color;
    animation_param1_ = group_size;
    animation_param2_ = rotation_ms;
    animation_start_ = std::chrono::steady_clock::now();
}

void WS2812Controller::fast_blink(const RGBColor& color, int on_ms, int off_ms) {
    current_animation_ = LEDAnimation::FAST_BLINK;
    animation_color_ = color;
    animation_param1_ = on_ms;
    animation_param2_ = off_ms;
    blink_state_ = true;
    animation_start_ = std::chrono::steady_clock::now();
    last_blink_toggle_ = animation_start_;
    fill_pixels(color);
    if (initialized_) send_pixels();
}

void WS2812Controller::wipe_off(const RGBColor& start_color, int total_ms) {
    current_animation_ = LEDAnimation::WIPE_OFF;
    animation_color_ = start_color;
    animation_param1_ = total_ms;
    animation_start_ = std::chrono::steady_clock::now();
    fill_pixels(start_color);
    if (initialized_) send_pixels();
}

void WS2812Controller::progress_ring(const RGBColor& color) {
    current_animation_ = LEDAnimation::PROGRESS_RING;
    animation_color_ = color;
    current_progress_ = 0.0f;
    animation_start_ = std::chrono::steady_clock::now();
}

void WS2812Controller::set_progress(float progress) {
    current_progress_ = std::max(0.0f, std::min(1.0f, progress));
}

void WS2812Controller::growing_rotate(const RGBColor& color, int grow_duration_ms, int rotation_ms) {
    current_animation_ = LEDAnimation::GROWING_ROTATE;
    animation_color_ = color;
    animation_param1_ = grow_duration_ms;  // Total time to grow from 3 to 16 LEDs
    animation_param2_ = rotation_ms;        // Time for one full rotation
    animation_start_ = std::chrono::steady_clock::now();
}

// ========================================
// Dedicated animation thread (smooth like Python)
// ========================================

void WS2812Controller::animation_loop() {
    std::cout << "💡 WS2812: Animation thread started" << std::endl;
    
    while (thread_running_) {
        if (!initialized_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        // Lock SPI for thread safety
        {
            std::lock_guard<std::mutex> lock(spi_mutex_);
            
            switch (current_animation_) {
                case LEDAnimation::OFF:
                case LEDAnimation::SOLID:
                    break;
                case LEDAnimation::BREATHING:
                    update_breathing();
                    break;
                case LEDAnimation::SINGLE_ROTATE:
                    update_single_rotate();
                    break;
                case LEDAnimation::BLINK_N_TIMES:
                    update_blink_n_times();
                    break;
                case LEDAnimation::ALTERNATE_ROTATE:
                    update_alternate_rotate();
                    break;
                case LEDAnimation::GROUP_ROTATE:
                    update_group_rotate();
                    break;
                case LEDAnimation::FAST_BLINK:
                    update_fast_blink();
                    break;
                case LEDAnimation::WIPE_OFF:
                    update_wipe_off();
                    break;
                case LEDAnimation::PROGRESS_RING:
                    update_progress_ring();
                    break;
                case LEDAnimation::GROWING_ROTATE:
                    update_growing_rotate();
                    break;
                default:
                    break;
            }
        }
        
        // Sleep exactly 10ms like Python version for smooth 100 FPS
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::cout << "💡 WS2812: Animation thread stopped" << std::endl;
}

// 4. IDLE_STATE: Pulsating green light (smooth breathing)
// Optimized for smoothness - matches Python implementation that worked well
void WS2812Controller::update_breathing() {
    float time_ms = get_animation_time_ms();
    int cycle_ms = animation_param1_ > 0 ? animation_param1_ : 3000;
    
    // Use high-precision floating point throughout
    float phase = std::fmod(time_ms, static_cast<float>(cycle_ms)) / static_cast<float>(cycle_ms);
    
    // Sine wave: 0 to 1 range
    float sine_val = (std::sin(phase * 2.0f * M_PI - M_PI / 2.0f) + 1.0f) / 2.0f;
    
    // Apply gamma correction (2.2) for perceptually linear brightness
    float gamma = 2.2f;
    float brightness = std::pow(sine_val, gamma);
    
    // Scale to 5% - 100% range (never fully off for smooth effect)
    brightness = 0.05f + 0.95f * brightness;
    
    // Apply brightness to each channel with proper rounding
    uint8_t r = static_cast<uint8_t>(std::round(animation_color_.r * brightness));
    uint8_t g = static_cast<uint8_t>(std::round(animation_color_.g * brightness));
    uint8_t b = static_cast<uint8_t>(std::round(animation_color_.b * brightness));
    
    RGBColor scaled_color(r, g, b);
    fill_pixels(scaled_color);
    send_pixels();
}

// 2. BOOT_MDAI_WIFI_SEARCH: Single LED rotating
void WS2812Controller::update_single_rotate() {
    float time_ms = get_animation_time_ms();
    int rotation_ms = animation_param1_ > 0 ? animation_param1_ : 1000;
    
    float rotation = std::fmod(time_ms, rotation_ms) / rotation_ms;
    int pos = static_cast<int>(rotation * led_count_) % led_count_;
    
    clear_pixels();
    set_pixel(pos, animation_color_);
    
    // Add subtle tail for smoothness
    int prev = (pos - 1 + led_count_) % led_count_;
    set_pixel(prev, animation_color_.scaled(0.3f));
    
    send_pixels();
}

// 3. MDAI_READY & 5. COUNTDOWN: Blink N times
void WS2812Controller::update_blink_n_times() {
    if (blink_count_ >= animation_param1_) {
        // Done blinking - turn off
        clear_pixels();
        send_pixels();
        current_animation_ = LEDAnimation::OFF;
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_blink_toggle_).count();
    
    int duration = blink_state_ ? animation_param2_ : animation_param3_;
    
    if (elapsed >= duration) {
        last_blink_toggle_ = now;
        blink_state_ = !blink_state_;
        
        if (blink_state_) {
            fill_pixels(animation_color_);
        } else {
            clear_pixels();
            blink_count_++;
        }
        send_pixels();
    }
}

// 6. ALIGN_STATE: Alternate LEDs rotating (every 2nd LED)
void WS2812Controller::update_alternate_rotate() {
    float time_ms = get_animation_time_ms();
    int rotation_ms = animation_param1_ > 0 ? animation_param1_ : 500;
    
    // Calculate offset (0 or 1) based on time
    float rotation = std::fmod(time_ms, rotation_ms * 2) / (rotation_ms * 2);
    int offset = static_cast<int>(rotation * 2) % 2;
    
    clear_pixels();
    for (int i = offset; i < led_count_; i += 2) {
        set_pixel(i, animation_color_);
    }
    send_pixels();
}

// 7. PROCESSING_STATE: Group of 5 consecutive LEDs rotating
void WS2812Controller::update_group_rotate() {
    float time_ms = get_animation_time_ms();
    int group_size = animation_param1_ > 0 ? animation_param1_ : 5;
    int rotation_ms = animation_param2_ > 0 ? animation_param2_ : 800;
    
    float rotation = std::fmod(time_ms, rotation_ms) / rotation_ms;
    int start_pos = static_cast<int>(rotation * led_count_) % led_count_;
    
    clear_pixels();
    for (int i = 0; i < group_size; i++) {
        int pos = (start_pos + i) % led_count_;
        // Gradient: brightest at front, dimmer at back
        float brightness = 1.0f - (static_cast<float>(i) / group_size) * 0.5f;
        set_pixel(pos, animation_color_.scaled(brightness));
    }
    send_pixels();
}

// 8. VERIFIED_STATE & 9. FAILED_STATE: Fast blink (100ms on/off)
void WS2812Controller::update_fast_blink() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_blink_toggle_).count();
    
    int duration = blink_state_ ? animation_param1_ : animation_param2_;
    
    if (elapsed >= duration) {
        last_blink_toggle_ = now;
        blink_state_ = !blink_state_;
        
        if (blink_state_) {
            fill_pixels(animation_color_);
        } else {
            clear_pixels();
        }
        send_pixels();
    }
}

// 10. DELETING_STATE: Turn off LEDs one by one over 5 seconds
void WS2812Controller::update_wipe_off() {
    float time_ms = get_animation_time_ms();
    int total_ms = animation_param1_ > 0 ? animation_param1_ : 5000;
    
    if (time_ms >= total_ms) {
        clear_pixels();
        send_pixels();
        current_animation_ = LEDAnimation::OFF;
        return;
    }
    
    float progress = time_ms / total_ms;
    int leds_off = static_cast<int>(progress * led_count_);
    
    fill_pixels(animation_color_);
    for (int i = 0; i < leds_off; i++) {
        set_pixel(i, Colors::OFF);
    }
    send_pixels();
}

// 6. ALIGN_STATE: Progress ring - fills up based on face alignment progress
void WS2812Controller::update_progress_ring() {
    float progress = current_progress_.load();
    int leds_lit = static_cast<int>(progress * led_count_);
    
    clear_pixels();
    for (int i = 0; i < leds_lit; i++) {
        set_pixel(i, animation_color_);
    }
    
    // Partial LED at the edge for smooth transition
    if (leds_lit < led_count_) {
        float partial = (progress * led_count_) - leds_lit;
        if (partial > 0.1f) {
            set_pixel(leds_lit, animation_color_.scaled(partial));
        }
    }
    
    send_pixels();
}

// PROCESSING_STATE: Growing rotating animation
// Starts with 3 LEDs, grows to full ring over grow_duration, then breathes
// Exact 1:1 translation from Python version
void WS2812Controller::update_growing_rotate() {
    float elapsed_ms = get_animation_time_ms();
    int grow_duration_ms = animation_param1_ > 0 ? animation_param1_ : 10000;
    int rotation_ms = animation_param2_ > 0 ? animation_param2_ : 800;
    
    const int INITIAL_LEDS = 3;
    
    if (elapsed_ms >= grow_duration_ms) {
        // Growth complete - switch to breathing animation (full ring pulsating)
        // This happens automatically - we just do breathing math now
        float breath_time_ms = elapsed_ms - grow_duration_ms;
        float breath_cycle_ms = 3000.0f;
        float phase = std::fmod(breath_time_ms, breath_cycle_ms) / breath_cycle_ms;
        
        // Sine wave with gamma correction (same as breathing)
        float sine_val = (std::sin(phase * 2.0f * M_PI - M_PI / 2.0f) + 1.0f) / 2.0f;
        float brightness = std::pow(sine_val, 2.2f);
        brightness = 0.05f + 0.95f * brightness;
        
        uint8_t r = static_cast<uint8_t>(std::round(animation_color_.r * brightness));
        uint8_t g = static_cast<uint8_t>(std::round(animation_color_.g * brightness));
        uint8_t b = static_cast<uint8_t>(std::round(animation_color_.b * brightness));
        
        fill_pixels(RGBColor(r, g, b));
        send_pixels();
        return;
    }
    
    // Growing phase - exact Python logic
    float progress = elapsed_ms / static_cast<float>(grow_duration_ms);
    int num_leds = INITIAL_LEDS + static_cast<int>(progress * (led_count_ - INITIAL_LEDS));
    num_leds = std::min(num_leds, led_count_);
    
    // Calculate rotation position
    float rotation_progress = std::fmod(elapsed_ms, static_cast<float>(rotation_ms)) / static_cast<float>(rotation_ms);
    int start_pos = static_cast<int>(rotation_progress * led_count_) % led_count_;
    
    // Build pixel array with gradient
    clear_pixels();
    for (int i = 0; i < num_leds; i++) {
        int pos = (start_pos + i) % led_count_;
        // Gradient: brightest at front, dimmer at back (same as Python)
        float brightness = 1.0f - (static_cast<float>(i) / num_leds) * 0.6f;
        set_pixel(pos, animation_color_.scaled(brightness));
    }
    
    send_pixels();
}

// ========================================
// State-based control
// ========================================

void WS2812Controller::set_system_state(SystemState state) {
    switch (state) {
        case SystemState::BOOT:
            // 1. BOOT_LOGO: All LEDs solid white
            solid(Colors::WHITE);
            break;
            
        case SystemState::PROVISIONING:
            // 2. BOOT_MDAI_WIFI_SEARCH: Single white LED rotating
            single_rotate(Colors::WHITE, 800);
            break;
            
        case SystemState::PROVISIONED:
            // 3. MDAI_READY: Blink white 5 times
            blink_n_times(Colors::WHITE, 5, 200, 200);
            break;
            
        case SystemState::AWAIT_ADMIN_QR:
            // Same as WiFi search
            single_rotate(Colors::WHITE, 800);
            break;
            
        case SystemState::IDLE:
            // 4. IDLE_STATE: Pulsating green (smooth breathing)
            breathing(Colors::GREEN, 3000);
            break;
            
        case SystemState::WIFI_CHANGE_CONNECTING:
            single_rotate(Colors::WHITE, 800);
            break;
            
        case SystemState::WIFI_CHANGE_SUCCESS:
            blink_n_times(Colors::WHITE, 5, 200, 200);
            break;
            
        case SystemState::WIFI_CHANGE_FAILED:
            fast_blink(Colors::RED, 100, 100);
            break;
            
        case SystemState::READY:
            // READY: Faster green breathing/pulsation (1.5 second cycle vs 3 second in IDLE)
            breathing(Colors::GREEN, 1500);
            break;
            
        case SystemState::COUNTDOWN:
            // 5. COUNTDOWN_STATE: Blink green 5 times, 1 second gap
            blink_n_times(Colors::GREEN, 5, 500, 500);
            break;
            
        case SystemState::WARMUP:
            breathing(Colors::GREEN, 2000);
            break;
            
        case SystemState::ALIGN:
            // 6. ALIGN_STATE: Progress ring synced with face alignment
            progress_ring(Colors::GREEN);
            break;
            
        case SystemState::PROCESSING:
            // 7. PROCESSING_STATE: Growing rotate - starts with 3 LEDs, grows to full ring
            // over 10 seconds, then switches to breathing
            growing_rotate(Colors::GREEN, 10000, 800);
            break;
            
        case SystemState::SUCCESS:
            // 8. VERIFIED_STATE: Fast green blink
            fast_blink(Colors::GREEN, 100, 100);
            break;
            
        case SystemState::ERROR:
            // 9. FAILED_STATE: Fast red blink
            fast_blink(Colors::RED, 100, 100);
            break;
            
        case SystemState::DELETE_SCREEN:
            // 10. DELETING_STATE: Red wipe off over 5 seconds
            wipe_off(Colors::RED, 5000);
            break;
            
        default:
            off();
            break;
    }
}

}  // namespace mdai
