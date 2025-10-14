# Examples

# Basic capture example
add_executable(basic_capture basic_capture.cpp)
target_link_libraries(basic_capture mdai_realsense)

# Pipeline example with mock anti-spoofing
add_executable(pipeline_example pipeline_example.cpp)
target_link_libraries(pipeline_example mdai_realsense)

# Frame saver example
add_executable(frame_saver frame_saver.cpp)
target_link_libraries(frame_saver mdai_realsense)

# Anti-spoofing test suite (kept)
add_executable(anti_spoofing_test_suite anti_spoofing_test_suite.cpp)
target_link_libraries(anti_spoofing_test_suite mdai_realsense)

# Human face detection test
add_executable(human_face_detection_test human_face_detection_test.cpp)
target_link_libraries(human_face_detection_test mdai_realsense)

# ASCII visual test
add_executable(ascii_visual_test ascii_visual_test.cpp)
target_link_libraries(ascii_visual_test mdai_realsense)

## Removed redundant executables to focus on the viewer
## add_executable(attack_scenario_test attack_scenario_test.cpp)
## target_link_libraries(attack_scenario_test mdai_realsense)

# Visual face detection test (requires OpenCV)
add_executable(visual_face_detection_test visual_face_detection_test.cpp)
target_link_libraries(visual_face_detection_test mdai_realsense ${OpenCV_LIBS})
target_include_directories(visual_face_detection_test PRIVATE ${OpenCV_INCLUDE_DIRS})

## add_executable(gui_attack_tester gui_attack_tester.cpp)
## target_link_libraries(gui_attack_tester mdai_realsense ${OpenCV_LIBS})
## target_include_directories(gui_attack_tester PRIVATE ${OpenCV_INCLUDE_DIRS})

# Simple Liveness Viewer (requires OpenCV)
add_executable(simple_liveness_viewer simple_liveness_viewer.cpp)
target_link_libraries(simple_liveness_viewer mdai_realsense ${OpenCV_LIBS})
target_include_directories(simple_liveness_viewer PRIVATE ${OpenCV_INCLUDE_DIRS})

## add_executable(test_real_anti_spoofing test_real_anti_spoofing.cpp)
## target_link_libraries(test_real_anti_spoofing mdai_realsense ${OpenCV_LIBS} realsense2)
## target_include_directories(test_real_anti_spoofing PRIVATE ${OpenCV_INCLUDE_DIRS})

## add_executable(debug_anti_spoofing debug_anti_spoofing.cpp)
## target_link_libraries(debug_anti_spoofing mdai_realsense ${OpenCV_LIBS} realsense2)
## target_include_directories(debug_anti_spoofing PRIVATE ${OpenCV_INCLUDE_DIRS})

