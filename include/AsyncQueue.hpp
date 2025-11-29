#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <chrono>

namespace mdai {

/**
 * @brief Thread-safe message queue for async communication
 * 
 * Used to decouple producers (main loop) from consumers (Serial/WebSocket threads)
 */
template<typename T>
class AsyncQueue {
public:
    explicit AsyncQueue(size_t max_size = 100) : max_size_(max_size) {}

    // Non-blocking push (drops oldest if full)
    bool push(T item) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (queue_.size() >= max_size_) {
            queue_.pop();  // Drop oldest
            dropped_++;
        }
        
        queue_.push(std::move(item));
        cv_.notify_one();
        return true;
    }

    // Blocking pop with timeout
    std::optional<T> pop(int timeout_ms = 100) {
        std::unique_lock<std::mutex> lock(mutex_);
        
        if (!cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                          [this] { return !queue_.empty() || !running_; })) {
            return std::nullopt;
        }
        
        if (!running_ || queue_.empty()) {
            return std::nullopt;
        }
        
        T item = std::move(queue_.front());
        queue_.pop();
        return item;
    }

    // Non-blocking try_pop
    std::optional<T> try_pop() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (queue_.empty()) {
            return std::nullopt;
        }
        
        T item = std::move(queue_.front());
        queue_.pop();
        return item;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty()) {
            queue_.pop();
        }
    }

    void stop() {
        running_ = false;
        cv_.notify_all();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    uint64_t dropped_count() const {
        return dropped_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<T> queue_;
    size_t max_size_;
    std::atomic<bool> running_{true};
    std::atomic<uint64_t> dropped_{0};
};

} // namespace mdai




