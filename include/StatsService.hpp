#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <deque>
#include <thread>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <iomanip>
#include <sstream>

namespace mdai {

/**
 * @brief Centralized statistics and metrics service
 * 
 * Thread-safe collection of performance metrics with periodic logging.
 * Designed for minimal overhead in the hot path.
 */
class StatsService {
public:
    // Timing metrics (in microseconds)
    struct FrameMetrics {
        uint64_t timestamp_us;
        uint32_t capture_time_us;
        uint32_t detection_time_us;
        uint32_t tracking_time_us;
        uint32_t serial_time_us;
        uint32_t total_time_us;
        bool face_detected;
        float nose_x, nose_y;
        float progress;
    };

    // IMU data snapshot
    struct IMUData {
        float accel_x, accel_y, accel_z;
        float gyro_x, gyro_y, gyro_z;
        uint64_t timestamp_us;
    };

    // Aggregate stats
    struct Summary {
        double avg_fps;
        double avg_detection_ms;
        double avg_total_ms;
        uint64_t total_frames;
        uint64_t faces_detected;
        uint64_t frames_dropped;
    };

    StatsService() : running_(false), log_interval_ms_(1000) {}
    
    ~StatsService() {
        stop();
    }

    void start(int log_interval_ms = 1000) {
        log_interval_ms_ = log_interval_ms;
        running_ = true;
        logger_thread_ = std::thread(&StatsService::logger_loop, this);
        std::cout << "📊 StatsService started (interval: " << log_interval_ms << "ms)" << std::endl;
    }

    void stop() {
        if (running_) {
            running_ = false;
            cv_.notify_all();
            if (logger_thread_.joinable()) {
                logger_thread_.join();
            }
            std::cout << "📊 StatsService stopped" << std::endl;
        }
    }

    // === Hot path methods (lock-free where possible) ===

    void record_frame_start() {
        frame_start_time_ = std::chrono::steady_clock::now();
    }

    void record_detection_time() {
        auto now = std::chrono::steady_clock::now();
        last_detection_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
            now - frame_start_time_).count();
    }

    void record_frame_complete(bool face_detected, float nose_x = 0, float nose_y = 0, float progress = 0) {
        auto now = std::chrono::steady_clock::now();
        uint32_t total_us = std::chrono::duration_cast<std::chrono::microseconds>(
            now - frame_start_time_).count();

        // Update atomic counters (lock-free)
        total_frames_.fetch_add(1, std::memory_order_relaxed);
        if (face_detected) {
            faces_detected_.fetch_add(1, std::memory_order_relaxed);
        }

        // Store detailed metrics (with lock, but only if logging is enabled)
        if (detailed_logging_) {
            std::lock_guard<std::mutex> lock(metrics_mutex_);
            
            FrameMetrics m;
            m.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch()).count();
            m.detection_time_us = last_detection_us_;
            m.total_time_us = total_us;
            m.face_detected = face_detected;
            m.nose_x = nose_x;
            m.nose_y = nose_y;
            m.progress = progress;

            recent_metrics_.push_back(m);
            if (recent_metrics_.size() > MAX_METRICS_HISTORY) {
                recent_metrics_.pop_front();
            }
        }

        // Update FPS calculation
        frame_times_[frame_time_idx_++ % FPS_WINDOW] = total_us;
    }

    void record_frame_dropped() {
        frames_dropped_.fetch_add(1, std::memory_order_relaxed);
    }

    void record_imu(float ax, float ay, float az, float gx = 0, float gy = 0, float gz = 0) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        last_imu_ = {ax, ay, az, gx, gy, gz, 
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count())};
    }

    // === Query methods ===

    Summary get_summary() const {
        Summary s;
        s.total_frames = total_frames_.load(std::memory_order_relaxed);
        s.faces_detected = faces_detected_.load(std::memory_order_relaxed);
        s.frames_dropped = frames_dropped_.load(std::memory_order_relaxed);
        
        // Calculate average FPS from recent frame times
        uint64_t sum = 0;
        int count = 0;
        for (int i = 0; i < FPS_WINDOW; i++) {
            if (frame_times_[i] > 0) {
                sum += frame_times_[i];
                count++;
            }
        }
        s.avg_total_ms = count > 0 ? (sum / count) / 1000.0 : 0;
        s.avg_fps = s.avg_total_ms > 0 ? 1000.0 / s.avg_total_ms : 0;
        
        // Calculate average detection time
        if (!recent_metrics_.empty()) {
            uint64_t det_sum = 0;
            for (const auto& m : recent_metrics_) {
                det_sum += m.detection_time_us;
            }
            s.avg_detection_ms = (det_sum / recent_metrics_.size()) / 1000.0;
        } else {
            s.avg_detection_ms = 0;
        }
        
        return s;
    }

    IMUData get_last_imu() const {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        return last_imu_;
    }

    double get_fps() const {
        uint64_t sum = 0;
        int count = 0;
        for (int i = 0; i < FPS_WINDOW; i++) {
            if (frame_times_[i] > 0) {
                sum += frame_times_[i];
                count++;
            }
        }
        double avg_ms = count > 0 ? (sum / count) / 1000.0 : 0;
        return avg_ms > 0 ? 1000.0 / avg_ms : 0;
    }

    void set_detailed_logging(bool enabled) {
        detailed_logging_ = enabled;
    }

    // === Logging callback ===
    void set_log_callback(std::function<void(const Summary&)> callback) {
        log_callback_ = callback;
    }

private:
    static constexpr int FPS_WINDOW = 30;
    static constexpr size_t MAX_METRICS_HISTORY = 100;

    std::atomic<bool> running_;
    std::atomic<bool> detailed_logging_{true};
    int log_interval_ms_;

    // Frame timing
    std::chrono::steady_clock::time_point frame_start_time_;
    uint32_t last_detection_us_ = 0;
    
    // Atomic counters (lock-free)
    std::atomic<uint64_t> total_frames_{0};
    std::atomic<uint64_t> faces_detected_{0};
    std::atomic<uint64_t> frames_dropped_{0};

    // FPS calculation window
    std::atomic<uint32_t> frame_times_[FPS_WINDOW] = {};
    std::atomic<int> frame_time_idx_{0};

    // Detailed metrics (with lock)
    mutable std::mutex metrics_mutex_;
    std::deque<FrameMetrics> recent_metrics_;

    // IMU data
    mutable std::mutex imu_mutex_;
    IMUData last_imu_{};

    // Logger thread
    std::thread logger_thread_;
    std::condition_variable cv_;
    std::mutex cv_mutex_;
    std::function<void(const Summary&)> log_callback_;

    void logger_loop() {
        while (running_) {
            std::unique_lock<std::mutex> lock(cv_mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(log_interval_ms_));
            
            if (!running_) break;

            auto summary = get_summary();
            
            // Default logging
            std::cout << "📊 STATS | "
                      << "FPS: " << std::fixed << std::setprecision(1) << summary.avg_fps
                      << " | Det: " << std::setprecision(1) << summary.avg_detection_ms << "ms"
                      << " | Total: " << std::setprecision(1) << summary.avg_total_ms << "ms"
                      << " | Faces: " << summary.faces_detected << "/" << summary.total_frames
                      << " (" << (summary.total_frames > 0 ? 100 * summary.faces_detected / summary.total_frames : 0) << "%)"
                      << " | Dropped: " << summary.frames_dropped
                      << std::endl;

            // Custom callback if set
            if (log_callback_) {
                log_callback_(summary);
            }
        }
    }
};

} // namespace mdai




