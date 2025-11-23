cmake_minimum_required(VERSION 3.10)
project(MediaPipeFaceMeshWrapper VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3 -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

# Find Python3
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

message(STATUS "Python3 found:")
message(STATUS "  Version: ${Python3_VERSION}")
message(STATUS "  Include dirs: ${Python3_INCLUDE_DIRS}")
message(STATUS "  Libraries: ${Python3_LIBRARIES}")
message(STATUS "  NumPy include: ${Python3_NumPy_INCLUDE_DIRS}")

# Find OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
message(STATUS "OpenCV found: ${OpenCV_VERSION}")

# Find abseil (required for status types)
find_package(absl QUIET)
if(NOT absl_FOUND)
    message(STATUS "Abseil not found via find_package, checking pkg-config...")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ABSL REQUIRED absl_status absl_statusor)
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# Source files
set(SOURCES
    face_mesh_wrapper.cpp
)

# Create shared library
add_library(face_mesh_wrapper SHARED ${SOURCES})

# Link libraries
target_link_libraries(face_mesh_wrapper
    ${Python3_LIBRARIES}
    ${OpenCV_LIBS}
)

# Set output name to match original
set_target_properties(face_mesh_wrapper PROPERTIES
    OUTPUT_NAME "face_mesh_wrapper"
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
)

# Install targets
install(TARGETS face_mesh_wrapper
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES face_mesh_wrapper.h DESTINATION include/mediapipe)

# Print build summary
message(STATUS "")
message(STATUS "========================================")
message(STATUS "MediaPipe Face Mesh Wrapper Build Config")
message(STATUS "========================================")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "C++ flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "")
message(STATUS "Dependencies:")
message(STATUS "  - Python ${Python3_VERSION}")
message(STATUS "  - OpenCV ${OpenCV_VERSION}")
message(STATUS "  - NumPy ${Python3_NumPy_VERSION}")
message(STATUS "========================================")
message(STATUS "")


