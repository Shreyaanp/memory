#pragma once

/**
 * @file WS2812Controller.hpp
 * @brief WS2812 LED Ring Controller for MDAI System
 * 
 * Controls a 16-LED WS2812 ring via SPI (pin 19 MOSI on RDK X5)
 * Provides state-based animations synchronized with SystemState
 * 
 * Uses dedicated thread for smooth animations (like Python version)
 */

#include <cstdint>
#include <atomic>
#include <chrono>
#include <array>
#include <thread>
#include <mutex>

namespace mdai {

// Forward declaration
enum class SystemState;

/**
 * @brief LED Animation types
 */
enum class LEDAnimation {
    OFF,
    SOLID,                  // All LEDs solid color
    BREATHING,              // Pulsating/breathing effect
    SINGLE_ROTATE,          // Single LED rotating (for WiFi search)
    BLINK_N_TIMES,          // Blink N times then stop
    ALTERNATE_ROTATE,       // Alternate LEDs rotating (every 2nd LED)
    GROUP_ROTATE,           // Group of N consecutive LEDs rotating
    FAST_BLINK,             // Fast continuous blink (100ms on/off)
    WIPE_OFF,               // Turn off LEDs one by one (for delete)
    PROGRESS_RING,          // Progress-based ring (for ALIGN)
    GROWING_ROTATE          // Growing rotating animation (for PROCESSING)
};

/**
 * @brief RGB Color structure
 */
struct RGBColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    
    RGBColor() : r(0), g(0), b(0) {}
    RGBColor(uint8_t red, uint8_t green, uint8_t blue) : r(red), g(green), b(blue) {}
    
    // Scale brightness (0.0 - 1.0)
    RGBColor scaled(float brightness) const {
        return RGBColor(
            static_cast<uint8_t>(r * brightness),
            static_cast<uint8_t>(g * brightness),
            static_cast<uint8_t>(b * brightness)
        );
    }
};

// Predefined colors
namespace Colors {
    const RGBColor OFF(0, 0, 0);
    const RGBColor WHITE(255, 255, 255);
    const RGBColor RED(255, 0, 0);
    const RGBColor GREEN(0, 255, 0);
    const RGBColor BLUE(0, 0, 255);
    const RGBColor CYAN(0, 255, 255);
    const RGBColor YELLOW(255, 200, 0);
    const RGBColor ORANGE(255, 100, 0);
    const RGBColor PURPLE(128, 0, 255);
}

/**
 * @brief WS2812 LED Ring Controller
 * 
 * Uses SPI to control WS2812 LEDs with proper timing.
 * Provides various animations for different system states.
 * 
 * All timing uses non-blocking code with chrono (equivalent to millis())
 */
class WS2812Controller {
public:
    // LED ring pixel count - stored as variable at top
    static constexpr int DEFAULT_LED_COUNT = 16;
    static constexpr int SPI_BUS = 1;
    static constexpr int SPI_DEVICE = 0;
    static constexpr int SPI_SPEED_HZ = 6400000;  // 6.4MHz for WS2812 timing
    
    explicit WS2812Controller(int led_count = DEFAULT_LED_COUNT);
    ~WS2812Controller();
    
    bool initialize();
    void stop();
    bool is_initialized() const { return initialized_; }
    
    // State-based control
    void set_system_state(SystemState state);
    
    // Note: update() is now called internally by dedicated thread
    // No need to call from main loop
    
    // Direct animation control
    void off();
    void solid(const RGBColor& color);
    void breathing(const RGBColor& color, int cycle_ms = 3000);
    void single_rotate(const RGBColor& color, int rotation_ms = 1000);
    void blink_n_times(const RGBColor& color, int count, int on_ms, int off_ms);
    void alternate_rotate(const RGBColor& color, int rotation_ms = 500);
    void group_rotate(const RGBColor& color, int group_size, int rotation_ms = 800);
    void fast_blink(const RGBColor& color, int on_ms = 100, int off_ms = 100);
    void wipe_off(const RGBColor& start_color, int total_ms = 5000);
    void progress_ring(const RGBColor& color);
    void set_progress(float progress);  // 0.0 to 1.0
    void growing_rotate(const RGBColor& color, int grow_duration_ms = 10000, int rotation_ms = 800);
    
private:
    // SPI communication
    int spi_fd_;
    int led_count_;
    bool initialized_;
    
    // Animation state
    LEDAnimation current_animation_;
    RGBColor animation_color_;
    int animation_param1_;  // Various uses: cycle_ms, count, group_size
    int animation_param2_;  // Various uses: rotation_ms, on_ms
    int animation_param3_;  // Various uses: off_ms
    int blink_count_;       // Counter for blink_n_times
    bool blink_state_;      // Current blink state (on/off)
    std::atomic<float> current_progress_{0.0f};  // Progress for ALIGN (0.0-1.0)
    
    // Timing (non-blocking with chrono - like millis())
    std::chrono::steady_clock::time_point animation_start_;
    std::chrono::steady_clock::time_point last_update_;
    std::chrono::steady_clock::time_point last_blink_toggle_;
    
    // LED buffer
    std::array<RGBColor, 32> pixel_buffer_;  // Max 32 LEDs
    
    // Dedicated animation thread (for smooth timing like Python)
    std::thread animation_thread_;
    std::atomic<bool> thread_running_{false};
    std::mutex spi_mutex_;
    
    void animation_loop();  // Thread function
    
    // Internal methods
    void encode_byte(uint8_t byte, uint8_t* output);
    void send_pixels();
    void set_pixel(int index, const RGBColor& color);
    void fill_pixels(const RGBColor& color);
    void clear_pixels();
    
    // Animation update methods (non-blocking)
    void update_breathing();
    void update_single_rotate();
    void update_blink_n_times();
    void update_alternate_rotate();
    void update_group_rotate();
    void update_fast_blink();
    void update_wipe_off();
    void update_progress_ring();
    void update_growing_rotate();
    
    float get_animation_time_ms() const;
};

}  // namespace mdai
