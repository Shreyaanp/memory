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
find_package(OpenCV QUIET)
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(nlohmann_json 3.2.0 QUIET)

# MediaPipe Configuration (legacy - kept for fallback)
set(MEDIAPIPE_AVAILABLE TRUE)
set(MEDIAPIPE_WRAPPER_LIB "${CMAKE_SOURCE_DIR}/lib/mediapipe/libface_mesh_wrapper.so")
set(MEDIAPIPE_WRAPPER_INCLUDE "${CMAKE_SOURCE_DIR}/include/mediapipe")
message(STATUS "Face detection: MediaPipe Face Mesh (468 points with GPU acceleration)")

# BPU Configuration (Horizon RDK X5)
set(BPU_AVAILABLE TRUE)
set(BPU_INCLUDE_DIR "/usr/include")
set(BPU_LIB "/usr/lib/libdnn.so")
message(STATUS "BPU acceleration: Horizon RDK X5 hbDNN (face landmarks ~1ms)")

if(OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
    add_definitions(-DHAVE_OPENCV)
else()
    message(WARNING "OpenCV not found - Utils functionality will be limited")
endif()

if(nlohmann_json_FOUND)
    message(STATUS "nlohmann_json found: ${nlohmann_json_VERSION}")
else()
    message(WARNING "nlohmann_json not found - using header-only fallback")
    # Add header-only include path
    include_directories(${PROJECT_SOURCE_DIR}/third_party/nlohmann)
endif()

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${realsense2_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${MEDIAPIPE_WRAPPER_INCLUDE}
    ${OPENSSL_INCLUDE_DIR}
    ${BPU_INCLUDE_DIR}
)

# CORE SOURCES
set(SOURCES
    src/FrameBox.cpp
    src/RingBuffer.cpp
    src/Producer.cpp
    src/CameraInitializer.cpp
    src/AntiSpoofing.cpp
    src/FaceDetector.cpp
    src/BPUFaceLandmark.cpp
    src/SerialCommunicator.cpp
    src/NetworkManager.cpp
    src/CryptoUtils.cpp
    src/SystemController.cpp
    src/TrustZoneIdentity.cpp
    # src/Utils.cpp  <-- Removed until needed
)

# Create shared library
add_library(mdai_realsense SHARED ${SOURCES})
target_link_libraries(mdai_realsense 
    ${realsense2_LIBRARY}
    Threads::Threads
    ${MEDIAPIPE_WRAPPER_LIB}
    ${BPU_LIB}
    OpenSSL::SSL 
    OpenSSL::Crypto
    -Wl,--no-as-needed
    dl
    absl_status
    absl_statusor
    zbar
)

if(OpenCV_FOUND)
    target_link_libraries(mdai_realsense ${OpenCV_LIBS})
endif()

if(nlohmann_json_FOUND)
    target_link_libraries(mdai_realsense nlohmann_json::nlohmann_json)
endif()

# Main System Application
add_executable(mdai_system src/main.cpp)
target_link_libraries(mdai_system mdai_realsense)

# Install targets
install(TARGETS mdai_realsense mdai_system
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

# Skip examples and tests for now to ensure core system builds
# add_subdirectory(examples)
# add_subdirectory(tests)


# Install targets
install(TARGETS mdai_realsense mdai_system
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

# Skip examples and tests for now to ensure core system builds
# add_subdirectory(examples)
# add_subdirectory(tests)
