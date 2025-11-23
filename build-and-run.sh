#!/bin/bash

# ═══════════════════════════════════════════════════════════════════════════════
# Build and Run MDAI System on RDK X5
# ═══════════════════════════════════════════════════════════════════════════════
# This script:
# 1. Builds the C++ application with all integrations
# 2. Ensures device is registered with EC2 server
# 3. Runs the system controller
# ═══════════════════════════════════════════════════════════════════════════════

set -e

PROJECT_ROOT="/home/mercleDev/codebase"
BUILD_DIR="$PROJECT_ROOT/build"
BINARY_NAME="mdai_system"

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║          MDAI RDK X5 Build & Run Script                      ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

# ==============================================================================
# Step 1: Check Device Registration
# ==============================================================================
echo "[1/5] Checking device registration..."
if [ ! -f "/etc/mdai/device_config.json" ]; then
    echo "⚠️  Device not registered!"
    echo "Please run: cd $PROJECT_ROOT/deployment && sudo ./deploy-complete-system.sh"
    echo "Or register manually with the EC2 server."
    exit 1
fi

echo "✅ Device registered"
cat /etc/mdai/device_config.json | python3 -m json.tool || true
echo ""

# ==============================================================================
# Step 2: Check Dependencies
# ==============================================================================
echo "[2/5] Checking dependencies..."

# Check for required libraries
MISSING_DEPS=()

if ! ldconfig -p | grep -q "librealsense2"; then
    MISSING_DEPS+=("librealsense2")
fi

if ! ldconfig -p | grep -q "libopencv"; then
    MISSING_DEPS+=("libopencv")
fi

if ! ldconfig -p | grep -q "libssl"; then
    MISSING_DEPS+=("libssl")
fi

if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    echo "⚠️  Missing dependencies: ${MISSING_DEPS[@]}"
    echo "Installing missing dependencies..."
    sudo apt-get update
    sudo apt-get install -y \
        librealsense2-dev \
        libopencv-dev \
        libssl-dev \
        nlohmann-json3-dev \
        libabsl-dev
fi

echo "✅ Dependencies satisfied"

# ==============================================================================
# Step 3: Build Project
# ==============================================================================
echo "[3/5] Building project..."

# Clean build
if [ "$1" == "--clean" ]; then
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi

# Create build directory
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Run CMake
echo "Running CMake..."
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DHAVE_OPENCV=ON \
    -DENABLE_MEDIAPIPE=ON

# Build
echo "Compiling..."
make -j$(nproc)

echo "✅ Build complete"
echo ""

# ==============================================================================
# Step 4: Check Serial Connection
# ==============================================================================
echo "[4/5] Checking serial connection to LilyGo..."

if [ -e "/dev/ttyUSB0" ]; then
    echo "✅ Serial device found: /dev/ttyUSB0"
elif [ -e "/dev/ttyACM0" ]; then
    echo "✅ Serial device found: /dev/ttyACM0"
else
    echo "⚠️  No serial device found. LilyGo display may not update."
    echo "   Check USB connection and device permissions."
fi

# Set permissions if device exists
if [ -e "/dev/ttyUSB0" ]; then
    sudo chmod 666 /dev/ttyUSB0 || true
fi
if [ -e "/dev/ttyACM0" ]; then
    sudo chmod 666 /dev/ttyACM0 || true
fi

echo ""

# ==============================================================================
# Step 5: Run System
# ==============================================================================
echo "[5/5] Starting MDAI System..."
echo ""
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║                    MDAI SYSTEM STARTED                        ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""
echo "📊 System Status:"
echo "   • Device ID: $(cat /etc/mdai/device_config.json | python3 -c 'import json, sys; print(json.load(sys.stdin)[\"device_id\"])')"
echo "   • EC2 Server: mdai.mercle.ai"
echo "   • Serial Port: $(ls /dev/ttyUSB0 2>/dev/null || ls /dev/ttyACM0 2>/dev/null || echo 'Not found')"
echo ""
echo "📹 Camera:"
echo "   • RGB: 640x480 @ 30fps (Idle)"
echo "   • RGB+IR+Depth: 1280x720 + 848x480 @ 30fps (Tracking)"
echo ""
echo "🔄 Flow:"
echo "   Boot → Check WiFi → Scan QR → Warmup → Align → Process → Result"
echo ""
echo "Press Ctrl+C to stop"
echo "─────────────────────────────────────────────────────────────────"
echo ""

# Run the binary
cd "$BUILD_DIR"
if [ -f "./$BINARY_NAME" ]; then
    ./"$BINARY_NAME"
else
    echo "❌ Binary not found: $BINARY_NAME"
    echo "Available binaries:"
    ls -la
    exit 1
fi

