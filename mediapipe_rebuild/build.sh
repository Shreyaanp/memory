#!/bin/bash

set -e

echo "=========================================="
echo "MediaPipe Face Mesh Wrapper - Build Script"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check Python dependencies
echo -e "${YELLOW}[1/5]${NC} Checking Python dependencies..."
python3 -c "import mediapipe; import numpy; import cv2" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} Python dependencies OK"
    python3 -c "import mediapipe; print('  - MediaPipe:', mediapipe.__version__)"
    python3 -c "import numpy; print('  - NumPy:', numpy.__version__)"
    python3 -c "import cv2; print('  - OpenCV:', cv2.__version__)"
else
    echo -e "${RED}✗${NC} Missing Python dependencies"
    echo "Please install: pip3 install mediapipe numpy opencv-python"
    exit 1
fi

# Check system dependencies
echo ""
echo -e "${YELLOW}[2/5]${NC} Checking system dependencies..."

MISSING_DEPS=()

# Check for Python dev
if ! pkg-config --exists python3; then
    MISSING_DEPS+=("python3-dev")
fi

# Check for OpenCV
if ! pkg-config --exists opencv4; then
    if ! pkg-config --exists opencv; then
        MISSING_DEPS+=("libopencv-dev")
    fi
fi

# Check for abseil
if ! pkg-config --exists absl_status 2>/dev/null; then
    MISSING_DEPS+=("libabsl-dev")
fi

if [ ${#MISSING_DEPS[@]} -ne 0 ]; then
    echo -e "${RED}✗${NC} Missing system dependencies:"
    for dep in "${MISSING_DEPS[@]}"; do
        echo "  - $dep"
    done
    echo ""
    echo "Install with: sudo apt-get install ${MISSING_DEPS[*]}"
    exit 1
fi

echo -e "${GREEN}✓${NC} System dependencies OK"

# Create build directory
echo ""
echo -e "${YELLOW}[3/5]${NC} Setting up build directory..."
BUILD_DIR="build"
if [ -d "$BUILD_DIR" ]; then
    echo "  Cleaning existing build directory..."
    rm -rf "$BUILD_DIR"
fi
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
echo -e "${GREEN}✓${NC} Build directory ready"

# Configure with CMake
echo ""
echo -e "${YELLOW}[4/5]${NC} Configuring with CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release
if [ $? -ne 0 ]; then
    echo -e "${RED}✗${NC} CMake configuration failed"
    exit 1
fi
echo -e "${GREEN}✓${NC} CMake configuration complete"

# Build
echo ""
echo -e "${YELLOW}[5/5]${NC} Building library..."
make -j$(nproc)
if [ $? -ne 0 ]; then
    echo -e "${RED}✗${NC} Build failed"
    exit 1
fi
echo -e "${GREEN}✓${NC} Build complete"

# Summary
echo ""
echo "=========================================="
echo -e "${GREEN}BUILD SUCCESSFUL${NC}"
echo "=========================================="
echo ""
echo "Output library: build/libface_mesh_wrapper.so"
echo ""
echo "To install system-wide:"
echo "  sudo cp build/libface_mesh_wrapper.so /usr/local/lib/"
echo "  sudo cp face_mesh_wrapper.h /usr/local/include/mediapipe/"
echo "  sudo ldconfig"
echo ""
echo "Or copy to your project:"
echo "  cp build/libface_mesh_wrapper.so ../lib/mediapipe/"
echo ""


