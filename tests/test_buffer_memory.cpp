#include "FrameBox.hpp"
#include "RingBuffer.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace mdai;

// Create mock FrameBox for testing
FrameBox create_mock_framebox(uint64_t seq_id, int width = 848, int height = 480) {
    FrameBox fb;
    fb.sequence_id = seq_id;
    
    // Create mock depth data (simulating frame memory usage)
    // In real scenario, rs2::frame holds the actual data
    // For testing, we just set the intrinsics to simulate size
    fb.depth_intrinsics.width = width;
    fb.depth_intrinsics.height = height;
    fb.color_intrinsics.width = width;
    fb.color_intrinsics.height = height;
    
    return fb;
}

void test_buffer_growth() {
    std::cout << "\n=== TEST: Buffer Growth During Recording ===" << std::endl;
    
    // Create buffer: 8 initial slots, 1GB max
    DynamicRingBuffer buffer(8, 5ULL * 1024 * 1024 * 1024);
    buffer.set_recording_active(true);
    
    std::cout << "Initial capacity: " << buffer.get_capacity() << " slots" << std::endl;
    
    // Write frames until buffer grows
    for (int i = 0; i < 20; ++i) {
        auto fb = create_mock_framebox(i);
        buffer.write(std::move(fb));
        
        if (i == 7) {
            std::cout << "After 8 frames, capacity: " << buffer.get_capacity() << std::endl;
        }
        if (i == 9) {
            std::cout << "After 10 frames, capacity: " << buffer.get_capacity() 
                     << " (should have grown to 16)" << std::endl;
        }
    }
    
    std::cout << "Final capacity: " << buffer.get_capacity() << " slots" << std::endl;
    std::cout << "Growth count: " << buffer.get_growth_count() << std::endl;
    std::cout << "Frames written: " << buffer.get_total_frames_written() << std::endl;
    std::cout << "Frames dropped: " << buffer.get_total_frames_dropped() 
             << " (should be 0 during recording)" << std::endl;
    
    if (buffer.get_capacity() > 8 && buffer.get_total_frames_dropped() == 0) {
        std::cout << "✓ PASS: Buffer grew correctly without dropping frames" << std::endl;
    } else {
        std::cout << "✗ FAIL: Buffer growth issue" << std::endl;
    }
}

void test_drop_oldest() {
    std::cout << "\n=== TEST: Drop-Oldest When Not Recording ===" << std::endl;
    
    DynamicRingBuffer buffer(8, 5ULL * 1024 * 1024 * 1024);
    buffer.set_recording_active(false);  // NOT recording
    
    std::cout << "Capacity: " << buffer.get_capacity() << " slots" << std::endl;
    
    // Write more frames than capacity
    for (int i = 0; i < 20; ++i) {
        auto fb = create_mock_framebox(i);
        buffer.write(std::move(fb));
    }
    
    std::cout << "Final capacity: " << buffer.get_capacity() 
             << " (should still be 8)" << std::endl;
    std::cout << "Frames written: " << buffer.get_total_frames_written() << std::endl;
    std::cout << "Frames dropped: " << buffer.get_total_frames_dropped() 
             << " (should be ~12)" << std::endl;
    
    if (buffer.get_capacity() == 8 && buffer.get_total_frames_dropped() > 0) {
        std::cout << "✓ PASS: Drop-oldest policy working correctly" << std::endl;
    } else {
        std::cout << "✗ FAIL: Drop-oldest issue" << std::endl;
    }
}

void test_realtime_consumer() {
    std::cout << "\n=== TEST: Realtime Consumer (Latest Frame) ===" << std::endl;
    
    DynamicRingBuffer buffer(32);
    
    // Write several frames
    for (int i = 0; i < 10; ++i) {
        auto fb = create_mock_framebox(i);
        buffer.write(std::move(fb));
    }
    
    // Get latest frame
    FrameBox* latest = buffer.get_latest_frame();
    
    if (latest) {
        std::cout << "Latest frame sequence: " << latest->sequence_id 
                 << " (should be 9)" << std::endl;
        
        if (latest->sequence_id == 9) {
            std::cout << "✓ PASS: Got latest frame correctly" << std::endl;
        } else {
            std::cout << "✗ FAIL: Wrong frame returned" << std::endl;
        }
        
        buffer.release_frame(latest);
    } else {
        std::cout << "✗ FAIL: No frame returned" << std::endl;
    }
}

void test_sequential_consumer() {
    std::cout << "\n=== TEST: Sequential Consumer (In-Order) ===" << std::endl;
    
    DynamicRingBuffer buffer(32);
    
    // Write frames
    for (int i = 0; i < 5; ++i) {
        auto fb = create_mock_framebox(i);
        buffer.write(std::move(fb));
    }
    
    // Read frames in sequence
    uint64_t last_seq = 0;
    int count = 0;
    bool in_order = true;
    
    for (int i = 0; i < 5; ++i) {
        FrameBox* frame = buffer.get_next_frame(last_seq);
        if (frame) {
            std::cout << "Got frame: " << frame->sequence_id << std::endl;
            
            if (frame->sequence_id != last_seq + 1) {
                in_order = false;
            }
            
            last_seq = frame->sequence_id;
            buffer.release_frame(frame);
            count++;
        }
    }
    
    if (in_order && count == 5) {
        std::cout << "✓ PASS: Sequential access working correctly" << std::endl;
    } else {
        std::cout << "✗ FAIL: Sequential access issue" << std::endl;
    }
}

void test_recording_mode_switch() {
    std::cout << "\n=== TEST: Recording Mode Switch ===" << std::endl;
    
    DynamicRingBuffer buffer(4, 5ULL * 1024 * 1024 * 1024);
    
    // Start with recording OFF - should drop oldest
    buffer.set_recording_active(false);
    std::cout << "Recording OFF: Writing 10 frames to 4-slot buffer..." << std::endl;
    
    for (int i = 0; i < 10; ++i) {
        auto fb = create_mock_framebox(i);
        buffer.write(std::move(fb));
    }
    
    auto dropped_before = buffer.get_total_frames_dropped();
    auto capacity_before = buffer.get_capacity();
    
    std::cout << "  Capacity: " << capacity_before << std::endl;
    std::cout << "  Dropped: " << dropped_before << std::endl;
    
    // Switch to recording ON - should grow instead of drop
    buffer.set_recording_active(true);
    std::cout << "\nRecording ON: Writing 10 more frames..." << std::endl;
    
    for (int i = 10; i < 20; ++i) {
        auto fb = create_mock_framebox(i);
        buffer.write(std::move(fb));
    }
    
    auto dropped_after = buffer.get_total_frames_dropped();
    auto capacity_after = buffer.get_capacity();
    
    std::cout << "  Capacity: " << capacity_after << std::endl;
    std::cout << "  Dropped: " << dropped_after << std::endl;
    std::cout << "  Growth count: " << buffer.get_growth_count() << std::endl;
    
    if (capacity_after > capacity_before && dropped_after == dropped_before) {
        std::cout << "✓ PASS: Recording mode switch working correctly" << std::endl;
    } else {
        std::cout << "✗ FAIL: Recording mode switch issue" << std::endl;
    }
}

int main() {
    std::cout << "╔═══════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Ring Buffer Memory Logic Test Suite     ║" << std::endl;
    std::cout << "╚═══════════════════════════════════════════╝" << std::endl;
    
    test_buffer_growth();
    test_drop_oldest();
    test_realtime_consumer();
    test_sequential_consumer();
    test_recording_mode_switch();
    
    std::cout << "\n╔═══════════════════════════════════════════╗" << std::endl;
    std::cout << "║  All Tests Complete                       ║" << std::endl;
    std::cout << "╚═══════════════════════════════════════════╝" << std::endl;
    
    return 0;
}


