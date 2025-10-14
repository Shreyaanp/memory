#include "RingBuffer.hpp"
#include <iostream>

using namespace mdai;

static int fails = 0;

static void assert_true(bool cond, const char* msg) {
    if (!cond) {
        std::cerr << "[FAIL] " << msg << std::endl;
        ++fails;
    } else {
        std::cout << "[PASS] " << msg << std::endl;
    }
}

int main() {
    std::cout << "=== DynamicRingBuffer Memory Logic Test ===" << std::endl;

    // Test 1: Buffer grows when recording is active and wrap occurs
    {
        size_t initial_capacity = 8;
        DynamicRingBuffer rb(initial_capacity, /*max_memory_bytes=*/ 5ULL * 1024 * 1024 * 1024); // 5GB cap
        rb.set_recording_active(true);

        // Fill the buffer completely
        for (size_t i = 0; i < initial_capacity; ++i) {
            FrameBox fb;
            fb.sequence_id = i;
            bool ok = rb.write(std::move(fb));
            assert_true(ok, "write() should succeed while recording active");
        }

        size_t cap_before = rb.get_capacity();
        size_t growths_before = rb.get_growth_count();
        uint64_t dropped_before = rb.get_total_frames_dropped();

        // Next write forces wrap; with recording active, buffer should grow
        FrameBox extra;
        extra.sequence_id = 999;
        bool ok = rb.write(std::move(extra));
        assert_true(ok, "write() should succeed after growth");

        size_t cap_after = rb.get_capacity();
        size_t growths_after = rb.get_growth_count();
        uint64_t dropped_after = rb.get_total_frames_dropped();

        assert_true(cap_after == cap_before * 2, "capacity doubles on growth");
        assert_true(growths_after == growths_before + 1, "growth count increments");
        assert_true(dropped_after == dropped_before, "no drops while recording active");
    }

    // Test 2: Buffer drops frames when recording is inactive
    {
        size_t initial_capacity = 4;
        DynamicRingBuffer rb(initial_capacity, /*max_memory_bytes=*/ 5ULL * 1024 * 1024 * 1024);
        // recording_active defaults to false

        // Write more than capacity to trigger drops
        size_t writes = initial_capacity + 3; // 7 writes, expect 3 drops
        for (size_t i = 0; i < writes; ++i) {
            FrameBox fb;
            fb.sequence_id = i;
            bool ok = rb.write(std::move(fb));
            assert_true(ok, "write() should return true even when dropping");
        }

        uint64_t drops = rb.get_total_frames_dropped();
        assert_true(drops == writes - initial_capacity, "drops equal writes beyond capacity when not recording");
    }

    // Test 3: Clear resets capacity to 32 and empties buffer
    {
        DynamicRingBuffer rb(64, /*max_memory_bytes=*/ 1024ULL * 1024 * 1024);
        // Fill a few frames
        for (size_t i = 0; i < 10; ++i) {
            FrameBox fb;
            fb.sequence_id = i;
            rb.write(std::move(fb));
        }
        assert_true(rb.get_usage() > 0, "usage > 0 before clear");
        rb.clear();
        assert_true(rb.get_capacity() == 32, "clear() resets capacity to 32");
        assert_true(rb.get_usage() == 0, "usage is 0 after clear");
    }

    if (fails == 0) {
        std::cout << "\nAll tests passed." << std::endl;
        return 0;
    } else {
        std::cerr << "\nTests failed: " << fails << std::endl;
        return 1;
    }
}

