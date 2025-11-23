#!/bin/bash
# Script to run simple_liveness_viewer with proper environment

# Set library paths
export LD_LIBRARY_PATH=$PWD/build:$PWD/lib/mediapipe:$LD_LIBRARY_PATH

# Ensure display is set
export DISPLAY=${DISPLAY:-:0}

# Check if camera is connected
echo "Checking RealSense camera..."
if ! rs-enumerate-devices 2>&1 | grep -q "Intel RealSense"; then
    echo "❌ No RealSense camera detected!"
    echo "Please connect your RealSense D435/D435I camera."
    exit 1
fi

echo "✅ Camera detected!"
echo ""
echo "Starting Simple Liveness Viewer..."
echo "=================================="
echo ""
echo "Controls:"
echo "  C - Start/Stop camera"
echo "  D - Toggle debug info"
echo "  R - Reset statistics"
echo "  Q - Quit"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Run the viewer
./build/examples/simple_liveness_viewer
