#pragma once

#include "FrameBox.hpp"
#include <vector>
#include <atomic>
#include <mutex>
#include <shared_mutex>
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
 * - Starts at 32 slots, grows dynamically up to 6GB
 * - Lock-free for producer (never blocks)
 * - Multiple independent consumers
 * - Drop-oldest policy when recording is inactive
 * - No drops during active recording (buffer grows instead)
 * - Support for batch processing iteration
 */
class DynamicRingBuffer {
public:
    explicit DynamicRingBuffer(size_t initial_capacity = 32, 
                               size_t max_memory_bytes = 6ULL * 1024 * 1024 * 1024);
    
    ~DynamicRingBuffer();
    
    bool write(FrameBox&& frame);
    FrameBox* get_latest_frame();
    FrameBox* get_next_frame(uint64_t last_sequence_id);
    void release_frame(FrameBox* frame);
    
    void set_recording_active(bool active);
    bool is_recording_active() const;
    void clear();
    
    size_t get_capacity() const;
    size_t get_usage() const;
    size_t get_memory_usage() const;
    uint64_t get_total_frames_written() const;
    uint64_t get_total_frames_dropped() const;
    size_t get_growth_count() const;
    
    /**
     * @brief Get all valid frames currently in the buffer (for batch processing)
     * @return Vector of pointers to valid frames (sorted by sequence_id)
     */
    std::vector<FrameBox*> get_all_valid_frames();

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
    
    mutable std::shared_mutex slots_mutex_;  // Protects slots_ vector during growth
    
    size_t estimate_frame_size(const FrameBox& frame) const;
    bool grow_buffer();
    size_t find_available_slot();
};

} // namespace mdai
