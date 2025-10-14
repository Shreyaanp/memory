#include "RingBuffer.hpp"
#include <iostream>
#include <algorithm>
#include <thread>

namespace mdai {

DynamicRingBuffer::DynamicRingBuffer(size_t initial_capacity, size_t max_memory_bytes)
    : capacity_(initial_capacity), max_memory_bytes_(max_memory_bytes) {
    
    slots_.reserve(initial_capacity);
    for (size_t i = 0; i < initial_capacity; ++i) {
        slots_.push_back(std::make_unique<RingBufferSlot>());
    }
}

DynamicRingBuffer::~DynamicRingBuffer() {
    clear();
}

bool DynamicRingBuffer::write(FrameBox&& frame) {
    size_t current_capacity = capacity_.load(std::memory_order_acquire);
    size_t write_idx = write_index_.fetch_add(1, std::memory_order_acq_rel) % current_capacity;
    
    auto& slot = slots_[write_idx];
    
    bool slot_was_empty = false;
    SlotState current_state = slot->state.load(std::memory_order_acquire);
    
    // Try to claim an EMPTY slot first
    SlotState expected_empty = SlotState::EMPTY;
    if (slot->state.compare_exchange_strong(expected_empty, SlotState::WRITING)) {
        slot_was_empty = true;
    } else {
        // Slot is not empty (could be READY, READING, or WRITING)
        if (recording_active_.load(std::memory_order_acquire)) {
            // Recording is active - try to grow buffer instead of dropping
            if (grow_buffer()) {
                // Buffer grown, retry write
                return write(std::move(frame));
            } else {
                // Can't grow - buffer at max capacity
                std::cerr << "WARNING: Ring buffer at maximum capacity (" 
                         << max_memory_bytes_ / (1024*1024*1024) << " GB). "
                         << "Frame may be dropped." << std::endl;
                // Fall through to overwrite (drop oldest)
            }
        }
        
        // Recording inactive or buffer can't grow - drop oldest
        // Wait for any readers to finish (with timeout to prevent deadlock)
        int wait_count = 0;
        while (slot->reader_count.load(std::memory_order_acquire) > 0) {
            std::this_thread::yield();
            if (++wait_count > 10000) {
                // Timeout - something is wrong, but don't deadlock
                std::cerr << "WARNING: Timeout waiting for readers on slot " << write_idx << std::endl;
                break;
            }
        }
        
        // Force slot to WRITING state (overwrite)
        slot->state.store(SlotState::WRITING, std::memory_order_release);
        
        // Only count as dropped if slot wasn't empty
        if (current_state != SlotState::EMPTY) {
            total_frames_dropped_.fetch_add(1, std::memory_order_relaxed);
        }
    }
    
    // Write the frame (try_lock to avoid blocking)
    if (slot->slot_mutex.try_lock()) {
        slot->frame = std::move(frame);
        slot->slot_mutex.unlock();
    } else {
        // Mutex locked - skip this write to avoid blocking
        std::cerr << "WARNING: Skipped frame write - slot mutex locked" << std::endl;
        return false;
    }
    
    // Mark slot as READY
    slot->state.store(SlotState::READY, std::memory_order_release);
    total_frames_written_.fetch_add(1, std::memory_order_relaxed);
    
    return true;
}

FrameBox* DynamicRingBuffer::get_latest_frame() {
    size_t current_capacity = capacity_.load(std::memory_order_acquire);
    size_t current_write_idx = write_index_.load(std::memory_order_acquire);
    
    // Start from most recent slot and work backwards
    for (size_t i = 0; i < current_capacity; ++i) {
        size_t idx = (current_write_idx + current_capacity - 1 - i) % current_capacity;
        auto& slot = slots_[idx];
        
        SlotState state = slot->state.load(std::memory_order_acquire);
        if (state == SlotState::READY || state == SlotState::READING) {
            // Try to acquire this slot for reading
            slot->reader_count.fetch_add(1, std::memory_order_acq_rel);
            
            // Double-check it's still ready
            state = slot->state.load(std::memory_order_acquire);
            if (state == SlotState::READY || state == SlotState::READING) {
                slot->state.store(SlotState::READING, std::memory_order_release);
                return &slot->frame;
            } else {
                // State changed, release and continue
                slot->reader_count.fetch_sub(1, std::memory_order_acq_rel);
            }
        }
    }
    
    return nullptr;
}

FrameBox* DynamicRingBuffer::get_next_frame(uint64_t last_sequence_id) {
    size_t current_capacity = capacity_.load(std::memory_order_acquire);
    size_t current_write_idx = write_index_.load(std::memory_order_acquire);
    
    // Search for next frame in sequence
    for (size_t i = 0; i < current_capacity; ++i) {
        size_t idx = (current_write_idx + current_capacity - i) % current_capacity;
        auto& slot = slots_[idx];
        
        SlotState state = slot->state.load(std::memory_order_acquire);
        if (state == SlotState::READY || state == SlotState::READING) {
            // Check sequence ID
            if (slot->frame.sequence_id > last_sequence_id) {
                // Try to acquire this slot
                slot->reader_count.fetch_add(1, std::memory_order_acq_rel);
                
                // Double-check state
                state = slot->state.load(std::memory_order_acquire);
                if (state == SlotState::READY || state == SlotState::READING) {
                    slot->state.store(SlotState::READING, std::memory_order_release);
                    return &slot->frame;
                } else {
                    slot->reader_count.fetch_sub(1, std::memory_order_acq_rel);
                }
            }
        }
    }
    
    return nullptr;
}

void DynamicRingBuffer::release_frame(FrameBox* frame) {
    if (!frame) return;
    
    // Find the slot containing this frame
    for (auto& slot : slots_) {
        if (&slot->frame == frame) {
            int readers = slot->reader_count.fetch_sub(1, std::memory_order_acq_rel) - 1;
            
            if (readers == 0) {
                // Last reader, mark as empty
                slot->state.store(SlotState::EMPTY, std::memory_order_release);
            }
            return;
        }
    }
}

void DynamicRingBuffer::set_recording_active(bool active) {
    recording_active_.store(active, std::memory_order_release);
}

bool DynamicRingBuffer::is_recording_active() const {
    return recording_active_.load(std::memory_order_acquire);
}

void DynamicRingBuffer::clear() {
    // Wait for all readers to finish
    for (auto& slot : slots_) {
        while (slot->reader_count.load(std::memory_order_acquire) > 0) {
            std::this_thread::yield();
        }
        slot->state.store(SlotState::EMPTY, std::memory_order_release);
    }
    
    // Reset to initial capacity
    size_t initial_capacity = 32;
    std::lock_guard<std::mutex> lock(growth_mutex_);
    
    slots_.clear();
    slots_.reserve(initial_capacity);
    for (size_t i = 0; i < initial_capacity; ++i) {
        slots_.push_back(std::make_unique<RingBufferSlot>());
    }
    
    capacity_.store(initial_capacity, std::memory_order_release);
    write_index_.store(0, std::memory_order_release);
}

size_t DynamicRingBuffer::get_capacity() const {
    return capacity_.load(std::memory_order_acquire);
}

size_t DynamicRingBuffer::get_usage() const {
    size_t count = 0;
    for (const auto& slot : slots_) {
        SlotState state = slot->state.load(std::memory_order_acquire);
        if (state != SlotState::EMPTY) {
            ++count;
        }
    }
    return count;
}

size_t DynamicRingBuffer::get_memory_usage() const {
    size_t total = 0;
    for (const auto& slot : slots_) {
        if (slot->frame.is_valid()) {
            total += estimate_frame_size(slot->frame);
        }
    }
    return total;
}

uint64_t DynamicRingBuffer::get_total_frames_written() const {
    return total_frames_written_.load(std::memory_order_acquire);
}

uint64_t DynamicRingBuffer::get_total_frames_dropped() const {
    return total_frames_dropped_.load(std::memory_order_acquire);
}

size_t DynamicRingBuffer::get_growth_count() const {
    return growth_count_.load(std::memory_order_acquire);
}

size_t DynamicRingBuffer::estimate_frame_size(const FrameBox& frame) const {
    size_t size = 0;
    
    // Depth frame: 2 bytes per pixel (Z16 format)
    if (!frame.depth_data.empty()) {
        size += frame.depth_data.size() * sizeof(uint16_t);
    }
    
    // Color frame: 3 bytes per pixel (BGR8 format)
    if (!frame.color_data.empty()) {
        size += frame.color_data.size();
    }
    
    // IR frames: 1 byte per pixel (Y8 format)
    if (!frame.ir_left_data.empty()) {
        size += frame.ir_left_data.size();
    }
    
    if (!frame.ir_right_data.empty()) {
        size += frame.ir_right_data.size();
    }
    
    // Add overhead for metadata (~1KB)
    size += 1024;
    
    return size;
}

bool DynamicRingBuffer::grow_buffer() {
    std::lock_guard<std::mutex> lock(growth_mutex_);
    
    size_t current_capacity = capacity_.load(std::memory_order_acquire);
    size_t new_capacity = current_capacity * 2;
    
    // Check memory limit
    size_t estimated_new_size = get_memory_usage() * new_capacity / current_capacity;
    if (estimated_new_size > max_memory_bytes_) {
        return false;
    }
    
    // Grow the buffer
    slots_.reserve(new_capacity);
    for (size_t i = current_capacity; i < new_capacity; ++i) {
        slots_.push_back(std::make_unique<RingBufferSlot>());
    }
    
    capacity_.store(new_capacity, std::memory_order_release);
    growth_count_.fetch_add(1, std::memory_order_relaxed);
    
    std::cout << "Ring buffer grown to " << new_capacity << " slots" << std::endl;
    
    return true;
}

size_t DynamicRingBuffer::find_available_slot() {
    size_t current_capacity = capacity_.load(std::memory_order_acquire);
    size_t start_idx = write_index_.load(std::memory_order_acquire) % current_capacity;
    
    for (size_t i = 0; i < current_capacity; ++i) {
        size_t idx = (start_idx + i) % current_capacity;
        auto& slot = slots_[idx];
        
        SlotState state = slot->state.load(std::memory_order_acquire);
        if (state == SlotState::EMPTY) {
            return idx;
        }
    }
    
    return current_capacity; // No available slot
}

} // namespace mdai
