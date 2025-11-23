#include "RingBuffer.hpp"
#include <algorithm>
#include <iostream>

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
    // Estimate size and check if we need to grow
    size_t frame_size = estimate_frame_size(frame);
    size_t current_mem = get_memory_usage();
    
    // If recording is active and we are full, try to grow
    if (recording_active_ && get_usage() >= capacity_) {
        if (current_mem + frame_size < max_memory_bytes_) {
            grow_buffer();
        } else {
            // OOM Protection: Drop if we can't grow anymore
            total_frames_dropped_++;
            return false; 
        }
    }

    size_t slot_idx = find_available_slot();
    
    if (slot_idx >= capacity_) {
        // Should not happen if logic is correct, but safety check
        if (!recording_active_) {
            // Force overwrite oldest if not recording
             slot_idx = write_index_ % capacity_;
        } else {
             return false;
        }
    }

    RingBufferSlot& slot = *slots_[slot_idx];
    
    {
        std::lock_guard<std::mutex> lock(slot.slot_mutex);
        slot.state = SlotState::WRITING;
        slot.frame = std::move(frame);
        slot.state = SlotState::READY;
    }
    
    write_index_++;
    total_frames_written_++;
    return true;
}

FrameBox* DynamicRingBuffer::get_latest_frame() {
    // Simple scan for highest sequence ID
    FrameBox* newest = nullptr;
    uint64_t max_seq = 0;
    
    for (auto& slot : slots_) {
        if (slot->state == SlotState::READY) {
             if (slot->frame.sequence_id > max_seq) {
                 max_seq = slot->frame.sequence_id;
                 newest = &slot->frame;
             }
        }
    }
    
    if (newest) {
        // Increment ref count mechanism if needed, or just return ptr
        // For safety in this design, consumer must call release_frame or be careful
        // (Ideally we'd return a wrapper/handle)
        newest->acquire();
    }
    return newest;
}

void DynamicRingBuffer::release_frame(FrameBox* frame) {
    if (frame) {
        frame->release();
    }
}

void DynamicRingBuffer::set_recording_active(bool active) {
    recording_active_ = active;
    if (active) {
        // When starting recording, maybe we want to clear old junk?
        // Or keep it as pre-roll context? Keeping for context is better.
    }
}

bool DynamicRingBuffer::is_recording_active() const {
    return recording_active_;
}

void DynamicRingBuffer::clear() {
    for (auto& slot : slots_) {
        std::lock_guard<std::mutex> lock(slot->slot_mutex);
        slot->state = SlotState::EMPTY;
        slot->frame = FrameBox(); // Clear data
    }
    write_index_ = 0;
}

size_t DynamicRingBuffer::get_capacity() const {
    return capacity_;
}

size_t DynamicRingBuffer::get_usage() const {
    size_t count = 0;
    for (const auto& slot : slots_) {
        if (slot->state == SlotState::READY || slot->state == SlotState::READING) {
            count++;
        }
    }
    return count;
}

size_t DynamicRingBuffer::get_memory_usage() const {
    // Rough estimate
    size_t bytes = 0;
    for (const auto& slot : slots_) {
        if (slot->state != SlotState::EMPTY) {
            bytes += estimate_frame_size(slot->frame);
        }
    }
    return bytes;
}

size_t DynamicRingBuffer::estimate_frame_size(const FrameBox& frame) const {
    return frame.depth_data.size() * 2 + 
           frame.color_data.size() + 
           frame.ir_left_data.size() + 
           frame.ir_right_data.size() + 
           sizeof(FrameBox);
}

bool DynamicRingBuffer::grow_buffer() {
    std::lock_guard<std::mutex> lock(growth_mutex_);
    size_t old_cap = capacity_;
    size_t new_cap = old_cap * 2;
    
    // Reallocation logic (tricky with vector of unique_ptrs if active)
    // We just append
    slots_.reserve(new_cap);
    for (size_t i = old_cap; i < new_cap; ++i) {
        slots_.push_back(std::make_unique<RingBufferSlot>());
    }
    
    capacity_ = new_cap;
    growth_count_++;
    return true;
}

size_t DynamicRingBuffer::find_available_slot() {
    // Linear search for EMPTY slot
    for (size_t i = 0; i < capacity_; ++i) {
        if (slots_[i]->state == SlotState::EMPTY) {
            return i;
        }
    }
    
    // If none empty and NOT recording, overwrite oldest
    if (!recording_active_) {
        // Simple circular overwrite
        return write_index_ % capacity_;
    }
    
    return capacity_; // Signal full
}

// Helper for Sequential Access (Batch Processing)
std::vector<FrameBox*> DynamicRingBuffer::get_all_valid_frames() {
    std::vector<FrameBox*> frames;
    for (auto& slot : slots_) {
        if (slot->state == SlotState::READY || slot->state == SlotState::READING) {
            frames.push_back(&slot->frame);
        }
    }
    
    // Sort by Sequence ID
    std::sort(frames.begin(), frames.end(), 
        [](const FrameBox* a, const FrameBox* b) {
            return a->sequence_id < b->sequence_id;
        });
        
    return frames;
}

// Stub Implementations for unused methods
FrameBox* DynamicRingBuffer::get_next_frame(uint64_t) { return nullptr; }
uint64_t DynamicRingBuffer::get_total_frames_written() const { return total_frames_written_; }
uint64_t DynamicRingBuffer::get_total_frames_dropped() const { return total_frames_dropped_; }
size_t DynamicRingBuffer::get_growth_count() const { return growth_count_; }

} // namespace mdai
