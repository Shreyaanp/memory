cmake_minimum_required(VERSION 3.10)
project(MDaiRealSense VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3 -march=armv8-a")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

# Find dependencies
find_package(realsense2 REQUIRED)
find_package(OpenCV QUIET)
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(nlohmann_json 3.2.0 QUIET)

# Native MediaPipe Configuration (no Python!)
set(MEDIAPIPE_NATIVE_DIR "$ENV{HOME}/mediapipe_arm64_final")
set(MEDIAPIPE_NATIVE_LIB "${MEDIAPIPE_NATIVE_DIR}/lib/libface_landmarker.so")
set(MEDIAPIPE_NATIVE_INCLUDE "${MEDIAPIPE_NATIVE_DIR}/include")
message(STATUS "Face detection: Native MediaPipe Face Landmarker (478 points, ~44 FPS, no Python)")

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
    ${MEDIAPIPE_NATIVE_INCLUDE}
    ${PROJECT_SOURCE_DIR}/include/mediapipe
    ${OPENSSL_INCLUDE_DIR}
)

# CORE SOURCES (IR-only mode - no anti-spoofing)
set(SOURCES
    src/FrameBox.cpp
    src/RingBuffer.cpp
    src/Producer.cpp
    src/CameraInitializer.cpp
    # src/AntiSpoofing.cpp  # REMOVED - IR-only mode
    src/FaceDetector.cpp
    src/NativeFaceLandmarker.cpp
    src/SerialCommunicator.cpp
    src/NetworkManager.cpp
    src/CryptoUtils.cpp
    src/SystemController.cpp
    src/TrustZoneIdentity.cpp
)

# Create shared library
add_library(mdai_realsense SHARED ${SOURCES})
target_link_libraries(mdai_realsense 
    ${realsense2_LIBRARY}
    Threads::Threads
    ${MEDIAPIPE_NATIVE_LIB}
    OpenSSL::SSL 
    OpenSSL::Crypto
    -Wl,--no-as-needed
    dl
    absl_status
    absl_statusor
    /usr/local/lib/libquirc.a
)

# Set RPATH for native MediaPipe library
set_target_properties(mdai_realsense PROPERTIES
    BUILD_RPATH "${MEDIAPIPE_NATIVE_DIR}/lib"
    INSTALL_RPATH "${MEDIAPIPE_NATIVE_DIR}/lib"
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

# Set RPATH for executable too
set_target_properties(mdai_system PROPERTIES
    BUILD_RPATH "${MEDIAPIPE_NATIVE_DIR}/lib"
    INSTALL_RPATH "${MEDIAPIPE_NATIVE_DIR}/lib"
)

# Install targets
install(TARGETS mdai_realsense mdai_system
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
