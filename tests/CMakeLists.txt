cmake_minimum_required(VERSION 3.10)

# Ring buffer memory logic test (no camera required)
add_executable(ring_buffer_test ring_buffer_test.cpp)
target_link_libraries(ring_buffer_test mdai_realsense)

# Buffer memory test
add_executable(test_buffer_memory test_buffer_memory.cpp)
target_link_libraries(test_buffer_memory mdai_realsense)

# Camera capture test (requires camera)
add_executable(test_camera_capture test_camera_capture.cpp)
target_link_libraries(test_camera_capture mdai_realsense)

# Camera test with consumer (requires camera)
add_executable(test_with_consumer test_with_consumer.cpp)
target_link_libraries(test_with_consumer mdai_realsense)

# Recording mode switching test (requires camera)
add_executable(test_recording_modes test_recording_modes.cpp)
target_link_libraries(test_recording_modes mdai_realsense)

# Multiple consumers test (requires camera)
add_executable(test_multiple_consumers test_multiple_consumers.cpp)
target_link_libraries(test_multiple_consumers mdai_realsense)

# Buffer stress test (requires camera)
add_executable(test_buffer_stress test_buffer_stress.cpp)
target_link_libraries(test_buffer_stress mdai_realsense)

# Camera initialization test (requires camera)
add_executable(test_camera_initialization test_camera_initialization.cpp)
target_link_libraries(test_camera_initialization mdai_realsense)

# Unified access system test (no camera required)
add_executable(test_unified_access test_unified_access.cpp)
target_link_libraries(test_unified_access mdai_realsense)
