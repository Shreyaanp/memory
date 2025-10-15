cmake_minimum_required(VERSION 3.10)
project(MDaiRealSense VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

# Find dependencies
find_package(realsense2 REQUIRED)
find_package(OpenCV QUIET)  # Optional - for Utils and fallback face detection
find_package(Threads REQUIRED)

# MediaPipe C++ GPU face detection (DISABLED - using OpenCV DNN for portability)
# MediaPipe has complex linking dependencies and is not easily portable to Jetson
# Using OpenCV DNN with automatic CUDA/CPU backend selection instead
set(MEDIAPIPE_AVAILABLE FALSE)
message(STATUS "Face detection: OpenCV DNN (portable Desktop + Jetson)")

# Check if OpenCV was found
if(OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
    add_definitions(-DHAVE_OPENCV)
else()
    message(WARNING "OpenCV not found - Utils functionality will be limited")
endif()

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${realsense2_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

# No additional includes needed - using OpenCV DNN

# Source files
set(SOURCES
    src/FrameBox.cpp
    src/RingBuffer.cpp
    src/Producer.cpp
    src/Pipeline.cpp
    src/CameraInitializer.cpp
    src/AntiSpoofing.cpp
    src/FaceDetector.cpp
)

# Add Utils only if OpenCV is available
if(OpenCV_FOUND)
    list(APPEND SOURCES src/Utils.cpp)
endif()

# Create shared library
add_library(mdai_realsense SHARED ${SOURCES})
target_link_libraries(mdai_realsense 
    ${realsense2_LIBRARY}
    Threads::Threads
)

if(OpenCV_FOUND)
    target_link_libraries(mdai_realsense ${OpenCV_LIBS})
endif()

# Create static library for Python bindings and tests
add_library(mdai_realsense_static STATIC ${SOURCES})

target_link_libraries(mdai_realsense_static
    ${realsense2_LIBRARY}
    Threads::Threads
)

if(OpenCV_FOUND)
    target_link_libraries(mdai_realsense_static ${OpenCV_LIBS})
endif()

# Install targets
install(TARGETS mdai_realsense mdai_realsense_static
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)

# Examples (require OpenCV since they include Utils)
if(OpenCV_FOUND)
    add_subdirectory(examples)
else()
    message(WARNING "Skipping examples build: OpenCV not found")
    # Build only anti-spoofing examples without OpenCV
    add_executable(quick_anti_spoofing_test examples/quick_anti_spoofing_test.cpp)
    target_link_libraries(quick_anti_spoofing_test mdai_realsense)
    
    add_executable(anti_spoofing_test_suite examples/anti_spoofing_test_suite.cpp)
    target_link_libraries(anti_spoofing_test_suite mdai_realsense)
    
    add_executable(human_face_detection_test examples/human_face_detection_test.cpp)
    target_link_libraries(human_face_detection_test mdai_realsense)
    
    add_executable(ascii_visual_test examples/ascii_visual_test.cpp)
    target_link_libraries(ascii_visual_test mdai_realsense)
    
    add_executable(attack_scenario_test examples/attack_scenario_test.cpp)
    target_link_libraries(attack_scenario_test mdai_realsense)
    
    # Visual test (requires OpenCV)
    if(OpenCV_FOUND)
        add_executable(visual_face_detection_test examples/visual_face_detection_test.cpp)
        target_link_libraries(visual_face_detection_test mdai_realsense ${OpenCV_LIBS})
        target_include_directories(visual_face_detection_test PRIVATE ${OpenCV_INCLUDE_DIRS})
        
        add_executable(gui_attack_tester examples/gui_attack_tester.cpp)
        target_link_libraries(gui_attack_tester mdai_realsense ${OpenCV_LIBS})
        target_include_directories(gui_attack_tester PRIVATE ${OpenCV_INCLUDE_DIRS})
    else()
        message(WARNING "Visual tests require OpenCV - install libopencv-dev to enable")
    endif()
endif()

# Tests
add_subdirectory(tests)

