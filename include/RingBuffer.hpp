#pragma once

#include "FrameBox.hpp"
#include <vector>
#include <atomic>
#include <mutex>
#include <memory>
#include <stdexcept>

namespace mdai {

/**
 * @brief Slot state in the ring buffer
 */
enum class SlotState {
    EMPTY,       // Slot is empty and can be written
    WRITING,     // Producer is currently writing to this slot
    READY,       // Slot is ready for consumers to read
    READING      // At least one consumer is reading this slot
};

/**
 * @brief A single slot in the ring buffer
 */
struct RingBufferSlot {
    FrameBox frame;
    std::atomic<SlotState> state{SlotState::EMPTY};
    std::atomic<int> reader_count{0};  // Number of consumers currently reading
    std::mutex slot_mutex;
    
    RingBufferSlot() = default;
    
    // Prevent copying
    RingBufferSlot(const RingBufferSlot&) = delete;
    RingBufferSlot& operator=(const RingBufferSlot&) = delete;
    
    // Allow moving
    RingBufferSlot(RingBufferSlot&&) = default;
    RingBufferSlot& operator=(RingBufferSlot&&) = default;
};

/**
 * @brief Dynamic ring buffer for FrameBox storage
 * 
 * Features:
 * - Starts at 32 slots, grows dynamically up to 5GB
 * - Lock-free for producer (never blocks)
 * - Multiple independent consumers
 * - Drop-oldest policy when recording is inactive
 * - No drops during active recording (buffer grows instead)
 */
class DynamicRingBuffer {
public:
    /**
     * @brief Constructor
     * @param initial_capacity Initial number of slots (default: 32)
     * @param max_memory_bytes Maximum memory usage in bytes (default: 5GB)
     */
    explicit DynamicRingBuffer(size_t initial_capacity = 32, 
                               size_t max_memory_bytes = 5ULL * 1024 * 1024 * 1024);
    
    /**
     * @brief Destructor
     */
    ~DynamicRingBuffer();
    
    /**
     * @brief Write a frame to the ring buffer
     * @param frame FrameBox to write
     * @return true if written successfully, false if buffer is full and can't grow
     */
    bool write(FrameBox&& frame);
    
    /**
     * @brief Get the latest frame (real-time consumer pattern)
     * @return Pointer to FrameBox if available, nullptr otherwise
     */
    FrameBox* get_latest_frame();
    
    /**
     * @brief Get the next frame in sequence (sequential consumer pattern)
     * @param last_sequence_id Last sequence ID this consumer processed
     * @return Pointer to FrameBox if available, nullptr if no new frames
     */
    FrameBox* get_next_frame(uint64_t last_sequence_id);
    
    /**
     * @brief Release a frame after consumer is done with it
     * @param frame Pointer to the frame to release
     */
    void release_frame(FrameBox* frame);
    
    /**
     * @brief Set recording active state
     * @param active true to enable recording mode (no drops, grow buffer)
     */
    void set_recording_active(bool active);
    
    /**
     * @brief Check if recording is active
     * @return true if recording is active
     */
    bool is_recording_active() const;
    
    /**
     * @brief Clear all frames from buffer (called when recording stops)
     */
    void clear();
    
    /**
     * @brief Get current buffer capacity (number of slots)
     * @return Number of slots
     */
    size_t get_capacity() const;
    
    /**
     * @brief Get current buffer usage (number of filled slots)
     * @return Number of filled slots
     */
    size_t get_usage() const;
    
    /**
     * @brief Get estimated memory usage in bytes
     * @return Estimated memory usage
     */
    size_t get_memory_usage() const;
    
    /**
     * @brief Get total frames written
     * @return Total frames written to buffer
     */
    uint64_t get_total_frames_written() const;
    
    /**
     * @brief Get total frames dropped
     * @return Total frames dropped (when recording inactive)
     */
    uint64_t get_total_frames_dropped() const;
    
    /**
     * @brief Get number of times buffer was grown
     * @return Number of growth operations
     */
    size_t get_growth_count() const;
    
private:
    std::vector<std::unique_ptr<RingBufferSlot>> slots_;
    std::atomic<size_t> capacity_;
    std::atomic<size_t> write_index_{0};
    
    std::atomic<bool> recording_active_{false};
    size_t max_memory_bytes_;
    
    // Statistics
    std::atomic<uint64_t> total_frames_written_{0};
    std::atomic<uint64_t> total_frames_dropped_{0};
    std::atomic<size_t> growth_count_{0};
    
    // Thread safety for buffer growth
    std::mutex growth_mutex_;
    
    /**
     * @brief Estimate memory usage of a single FrameBox
     * @param frame FrameBox to estimate
     * @return Estimated size in bytes
     */
    size_t estimate_frame_size(const FrameBox& frame) const;
    
    /**
     * @brief Grow the buffer capacity
     * @return true if grown successfully, false if at max capacity
     */
    bool grow_buffer();
    
    /**
     * @brief Find next empty or reclaimable slot
     * @return Index of available slot, or capacity if none available
     */
    size_t find_available_slot();
};

} // namespace mdai


