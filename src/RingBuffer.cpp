#include "RingBuffer.hpp"
#include <algorithm>
#include <iostream>
#include <thread>
#include <chrono>
#include <stdexcept>

namespace mdai {

DynamicRingBuffer::DynamicRingBuffer(size_t initial_capacity, size_t max_memory_bytes)
    : capacity_(initial_capacity), max_memory_bytes_(max_memory_bytes) {
    
    if (initial_capacity == 0) {
        throw std::invalid_argument("Ring buffer capacity must be > 0");
    }
    
    try {
        slots_.reserve(initial_capacity);
        for (size_t i = 0; i < initial_capacity; ++i) {
            slots_.push_back(std::make_unique<RingBufferSlot>());
        }
    } catch (const std::exception& e) {
        std::cerr << "❌ RingBuffer init failed: " << e.what() << std::endl;
        throw;
    }
}

DynamicRingBuffer::~DynamicRingBuffer() {
    // Destructor should not throw - just let unique_ptr destructors clean up
    // Don't call clear() as consumers may still hold references
}

bool DynamicRingBuffer::write(FrameBox&& frame) {
    try {
        size_t cap = capacity_.load(std::memory_order_acquire);
        if (cap == 0 || slots_.empty()) {
            return false;
        }
        
        size_t frame_size = estimate_frame_size(frame);
        size_t current_mem = get_memory_usage();
        size_t current_usage = get_usage();
        
        // If recording is active and we are full, try to grow
        // grow_buffer() takes its own exclusive lock, so call it outside shared lock
        if (recording_active_.load(std::memory_order_acquire) && current_usage >= cap) {
            if (current_mem + frame_size < max_memory_bytes_) {
                if (grow_buffer()) {
                    cap = capacity_.load(std::memory_order_acquire);
                }
            } else {
                total_frames_dropped_.fetch_add(1, std::memory_order_relaxed);
                return false; 
            }
        }

        // Take shared lock to prevent vector reallocation during slot access
        std::shared_lock<std::shared_mutex> shared_lock(slots_mutex_);
        
        // Re-read capacity after acquiring lock
        cap = capacity_.load(std::memory_order_acquire);
        if (cap == 0 || slots_.empty()) {
            return false;
        }

        // CIRCULAR BUFFER: Find the oldest frame to overwrite
        // Strategy 1: Try to find an EMPTY slot first
        // Strategy 2: If no empty slots, overwrite the OLDEST frame (lowest sequence_id)
        //             that has no active readers
        
        size_t target_slot = cap; // Invalid
        uint64_t min_seq = UINT64_MAX;
        
        // First pass: Find empty slot OR oldest frame without readers
        for (size_t i = 0; i < cap && i < slots_.size(); ++i) {
            if (!slots_[i]) continue;
            
            RingBufferSlot& slot = *slots_[i];
            SlotState state = slot.state.load(std::memory_order_acquire);
            
            // Prefer EMPTY slots
            if (state == SlotState::EMPTY) {
                target_slot = i;
                break; // Found empty slot, use it immediately
            }
            
            // For READY slots, track the oldest one (lowest sequence_id)
            if (state == SlotState::READY) {
                // Skip if being read
                if (slot.reader_count.load(std::memory_order_acquire) > 0) {
                    continue;
                }
                
                uint64_t seq = slot.frame.sequence_id;
                if (seq < min_seq) {
                    min_seq = seq;
                    target_slot = i;
                }
            }
        }
        
        // If no suitable slot found, try harder - find ANY slot without readers
        if (target_slot >= cap) {
            for (size_t i = 0; i < cap && i < slots_.size(); ++i) {
                if (!slots_[i]) continue;
                
                RingBufferSlot& slot = *slots_[i];
                if (slot.reader_count.load(std::memory_order_acquire) == 0) {
                    target_slot = i;
                    break;
                }
            }
        }
        
        // Write to the target slot
        if (target_slot < cap && target_slot < slots_.size() && slots_[target_slot]) {
            RingBufferSlot& slot = *slots_[target_slot];
            
            // Try to acquire the slot
            std::unique_lock<std::mutex> lock(slot.slot_mutex, std::try_to_lock);
            if (!lock.owns_lock()) {
                // Slot is busy, try simple circular write
                size_t fallback_idx = write_index_.load(std::memory_order_acquire) % cap;
                if (fallback_idx < slots_.size() && slots_[fallback_idx]) {
                    RingBufferSlot& fallback_slot = *slots_[fallback_idx];
                    std::unique_lock<std::mutex> fallback_lock(fallback_slot.slot_mutex, std::try_to_lock);
                    if (fallback_lock.owns_lock() && 
                        fallback_slot.reader_count.load(std::memory_order_acquire) == 0) {
                        fallback_slot.state.store(SlotState::WRITING, std::memory_order_release);
                        fallback_slot.frame = std::move(frame);
                        fallback_slot.state.store(SlotState::READY, std::memory_order_release);
                        write_index_.store(fallback_idx + 1, std::memory_order_release);
                        total_frames_written_.fetch_add(1, std::memory_order_relaxed);
                        return true;
                    }
                }
                total_frames_dropped_.fetch_add(1, std::memory_order_relaxed);
                return false;
            }
            
            // Double-check reader_count with lock held
            if (slot.reader_count.load(std::memory_order_acquire) > 0) {
                // Someone started reading, drop frame
                total_frames_dropped_.fetch_add(1, std::memory_order_relaxed);
                return false;
            }
            
            // Safe to write - overwrite oldest frame
            slot.state.store(SlotState::WRITING, std::memory_order_release);
            slot.frame = std::move(frame);
            slot.state.store(SlotState::READY, std::memory_order_release);
            
            write_index_.store(target_slot + 1, std::memory_order_release);
            total_frames_written_.fetch_add(1, std::memory_order_relaxed);
            return true;
        }
        
        // All slots are being read - drop the frame
        static int all_slots_busy_count = 0;
        all_slots_busy_count++;
        if (all_slots_busy_count % 100 == 1) {
            std::cerr << "⚠️ RingBuffer write: All " << cap << " slots busy (dropped " 
                      << all_slots_busy_count << " frames)" << std::endl;
        }
        total_frames_dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ RingBuffer write exception: " << e.what() << std::endl;
        total_frames_dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
    } catch (...) {
        std::cerr << "❌ RingBuffer write unknown exception" << std::endl;
        total_frames_dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
}

FrameBox* DynamicRingBuffer::get_latest_frame() {
    try {
        // Take shared lock to prevent vector reallocation during iteration
        std::shared_lock<std::shared_mutex> shared_lock(slots_mutex_);
        
        size_t cap = capacity_.load(std::memory_order_acquire);
        if (cap == 0 || slots_.empty()) {
            return nullptr;
        }
        
        // Single-pass approach: find best candidate then acquire
        // Use timestamp (hardware time) instead of sequence_id because
        // sequence_id resets when Producer is recreated (e.g., after camera restart)
        double max_timestamp = -1.0;  // Use -1 to handle timestamp=0 case
        size_t best_slot_idx = cap; // Invalid index
        size_t ready_count = 0;
        size_t reading_count = 0;
        
        // Find the slot with the highest timestamp (most recent frame)
        for (size_t i = 0; i < cap && i < slots_.size(); ++i) {
            if (!slots_[i]) continue;
            
            RingBufferSlot& slot = *slots_[i];
            
            SlotState state = slot.state.load(std::memory_order_acquire);
            if (state == SlotState::READY) ready_count++;
            if (state == SlotState::READING) reading_count++;
            
            if (state != SlotState::READY && state != SlotState::READING) {
                continue;
            }
            
            double ts = slot.frame.timestamp;
            if (ts > max_timestamp) {
                max_timestamp = ts;
                best_slot_idx = i;
            }
        }
        
        // Debug: Log if no valid frames found (every 1000 calls to avoid spam)
        static int get_frame_calls = 0;
        get_frame_calls++;
        if (best_slot_idx >= cap && get_frame_calls % 1000 == 0) {
            std::cerr << "⚠️ get_latest_frame: No valid frame found! cap=" << cap 
                      << " ready=" << ready_count << " reading=" << reading_count << std::endl;
        }
        
        // If we found a candidate, lock it and acquire
        if (best_slot_idx < cap && best_slot_idx < slots_.size() && slots_[best_slot_idx]) {
            RingBufferSlot& slot = *slots_[best_slot_idx];
            std::lock_guard<std::mutex> lock(slot.slot_mutex);
            
            // Re-check state after acquiring lock
            SlotState state = slot.state.load(std::memory_order_acquire);
            if (state == SlotState::READY || state == SlotState::READING) {
                // Acquire the frame
                slot.reader_count.fetch_add(1, std::memory_order_acq_rel);
                slot.state.store(SlotState::READING, std::memory_order_release);
                slot.frame.acquire();
                return &slot.frame;
            }
        }
        
        return nullptr;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ RingBuffer get_latest_frame exception: " << e.what() << std::endl;
        return nullptr;
    } catch (...) {
        std::cerr << "❌ RingBuffer get_latest_frame unknown exception" << std::endl;
        return nullptr;
    }
}

void DynamicRingBuffer::release_frame(FrameBox* frame) {
    if (!frame) return;
    
    try {
        // Release the FrameBox ref_count first
        frame->release();
        
        // Take shared lock to prevent vector reallocation during slot access
        std::shared_lock<std::shared_mutex> shared_lock(slots_mutex_);
        
        // Find the slot containing this frame and decrement reader_count
        size_t cap = capacity_.load(std::memory_order_acquire);
        for (size_t i = 0; i < cap && i < slots_.size(); ++i) {
            if (!slots_[i]) continue;
            
            RingBufferSlot& slot = *slots_[i];
            
            if (&slot.frame == frame) {
                std::lock_guard<std::mutex> lock(slot.slot_mutex);
                
                int prev_count = slot.reader_count.load(std::memory_order_acquire);
                
                // Safety check: don't decrement below 0
                if (prev_count > 0) {
                    prev_count = slot.reader_count.fetch_sub(1, std::memory_order_acq_rel);
                    
                    // If this was the last reader, mark slot as READY again
                    if (prev_count == 1) {
                        slot.state.store(SlotState::READY, std::memory_order_release);
                    }
                } else {
                    // reader_count was already 0 - this shouldn't happen but handle gracefully
                    std::cerr << "⚠️ RingBuffer: release_frame called but reader_count was 0" << std::endl;
                    // Ensure state is READY if not being written
                    SlotState current_state = slot.state.load(std::memory_order_acquire);
                    if (current_state == SlotState::READING) {
                        slot.state.store(SlotState::READY, std::memory_order_release);
                    }
                }
                
                return;
            }
        }
        
        // Frame not found in buffer - this can happen if buffer grew or frame is stale
        // Not an error, just ignore
        
    } catch (const std::exception& e) {
        std::cerr << "❌ RingBuffer release_frame exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "❌ RingBuffer release_frame unknown exception" << std::endl;
    }
}

void DynamicRingBuffer::set_recording_active(bool active) {
    recording_active_.store(active, std::memory_order_release);
}

bool DynamicRingBuffer::is_recording_active() const {
    return recording_active_.load(std::memory_order_acquire);
}

void DynamicRingBuffer::clear() {
    try {
        // Wait briefly for any readers to finish
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // Take shared lock (not exclusive since we're just modifying slot contents, not vector)
        std::shared_lock<std::shared_mutex> shared_lock(slots_mutex_);
        
        size_t cap = capacity_.load(std::memory_order_acquire);
        for (size_t i = 0; i < cap && i < slots_.size(); ++i) {
            if (!slots_[i]) continue;
            
            RingBufferSlot& slot = *slots_[i];
            std::lock_guard<std::mutex> lock(slot.slot_mutex);
            
            // Only clear if no readers
            if (slot.reader_count.load(std::memory_order_acquire) == 0) {
                slot.state.store(SlotState::EMPTY, std::memory_order_release);
                slot.frame = FrameBox();
            }
        }
        write_index_.store(0, std::memory_order_release);
        
    } catch (const std::exception& e) {
        std::cerr << "❌ RingBuffer clear exception: " << e.what() << std::endl;
    }
}

size_t DynamicRingBuffer::get_capacity() const {
    return capacity_.load(std::memory_order_acquire);
}

size_t DynamicRingBuffer::get_usage() const {
    try {
        std::shared_lock<std::shared_mutex> shared_lock(slots_mutex_);
        
        size_t count = 0;
        size_t cap = capacity_.load(std::memory_order_acquire);
        for (size_t i = 0; i < cap && i < slots_.size(); ++i) {
            if (!slots_[i]) continue;
            SlotState state = slots_[i]->state.load(std::memory_order_acquire);
            if (state == SlotState::READY || state == SlotState::READING) {
                count++;
            }
        }
        return count;
    } catch (...) {
        return 0;
    }
}

size_t DynamicRingBuffer::get_memory_usage() const {
    try {
        std::shared_lock<std::shared_mutex> shared_lock(slots_mutex_);
        
        size_t bytes = 0;
        size_t cap = capacity_.load(std::memory_order_acquire);
        for (size_t i = 0; i < cap && i < slots_.size(); ++i) {
            if (!slots_[i]) continue;
            SlotState state = slots_[i]->state.load(std::memory_order_acquire);
            if (state != SlotState::EMPTY) {
                bytes += estimate_frame_size(slots_[i]->frame);
            }
        }
        return bytes;
    } catch (...) {
        return 0;
    }
}

size_t DynamicRingBuffer::estimate_frame_size(const FrameBox& frame) const {
    return frame.ir_data.size() + sizeof(FrameBox);
}

bool DynamicRingBuffer::grow_buffer() {
    try {
        // Take exclusive lock - blocks all readers during vector modification
        std::unique_lock<std::shared_mutex> exclusive_lock(slots_mutex_);
        
        size_t old_cap = capacity_.load(std::memory_order_acquire);
        size_t new_cap = old_cap * 2;
        
        // Sanity check - don't grow beyond reasonable limits
        if (new_cap > 10000) {
            std::cerr << "⚠️ RingBuffer: refusing to grow beyond 10000 slots" << std::endl;
            return false;
        }
        
        slots_.reserve(new_cap);
        for (size_t i = old_cap; i < new_cap; ++i) {
            slots_.push_back(std::make_unique<RingBufferSlot>());
        }
        
        capacity_.store(new_cap, std::memory_order_release);
        growth_count_.fetch_add(1, std::memory_order_relaxed);
        
        std::cout << "📈 RingBuffer grew: " << old_cap << " → " << new_cap << " slots" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ RingBuffer grow_buffer exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "❌ RingBuffer grow_buffer unknown exception" << std::endl;
        return false;
    }
}

size_t DynamicRingBuffer::find_available_slot() {
    size_t cap = capacity_.load(std::memory_order_acquire);
    if (cap == 0) return 0;
    return write_index_.load(std::memory_order_acquire) % cap;
}

std::vector<FrameBox*> DynamicRingBuffer::get_all_valid_frames() {
    std::vector<FrameBox*> frames;
    bool success = false;
    
    {
        // Take shared lock to prevent vector reallocation during slot access
        std::shared_lock<std::shared_mutex> shared_lock(slots_mutex_);
        
        try {
            size_t cap = capacity_.load(std::memory_order_acquire);
            frames.reserve(cap); // Pre-allocate to avoid reallocation
            
            for (size_t i = 0; i < cap && i < slots_.size(); ++i) {
                if (!slots_[i]) continue;
                
                RingBufferSlot& slot = *slots_[i];
                
                std::lock_guard<std::mutex> lock(slot.slot_mutex);
                
                SlotState state = slot.state.load(std::memory_order_acquire);
                if (state == SlotState::READY || state == SlotState::READING) {
                    // Acquire for reading
                    slot.reader_count.fetch_add(1, std::memory_order_acq_rel);
                    slot.state.store(SlotState::READING, std::memory_order_release);
                    slot.frame.acquire();
                    
                    frames.push_back(&slot.frame);
                }
            }
            
            // Sort by sequence ID
            if (frames.size() > 1) {
                std::sort(frames.begin(), frames.end(), 
                    [](const FrameBox* a, const FrameBox* b) {
                        if (!a || !b) return false;
                        return a->sequence_id < b->sequence_id;
                    });
            }
            
            success = true;
            
        } catch (const std::exception& e) {
            std::cerr << "❌ RingBuffer get_all_valid_frames exception: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "❌ RingBuffer get_all_valid_frames unknown exception" << std::endl;
        }
    } // shared_lock released here
    
    // If we failed, release frames outside the lock to avoid deadlock
    if (!success) {
        for (FrameBox* frame : frames) {
            release_frame(frame);
        }
        frames.clear();
    }
    
    return frames;
}

// Stub implementations
FrameBox* DynamicRingBuffer::get_next_frame(uint64_t) { return nullptr; }
uint64_t DynamicRingBuffer::get_total_frames_written() const { return total_frames_written_.load(std::memory_order_acquire); }
uint64_t DynamicRingBuffer::get_total_frames_dropped() const { return total_frames_dropped_.load(std::memory_order_acquire); }
size_t DynamicRingBuffer::get_growth_count() const { return growth_count_.load(std::memory_order_acquire); }

} // namespace mdai
