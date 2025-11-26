#!/bin/bash

# Face Tracking GUI Test Launcher
# Run this and view via FreeRDP

echo "============================================"
echo "  MDai Face Tracking GUI Test"
echo "============================================"
echo ""

# Check dependencies
echo "Checking dependencies..."

python3 -c "import cv2" 2>/dev/null || {
    echo "❌ OpenCV not found. Install: pip3 install opencv-python"
    exit 1
}
echo "✓ OpenCV"

python3 -c "import numpy" 2>/dev/null || {
    echo "❌ NumPy not found. Install: pip3 install numpy"
    exit 1
}
echo "✓ NumPy"

python3 -c "import mediapipe" 2>/dev/null || {
    echo "❌ MediaPipe not found. Install: pip3 install mediapipe"
    exit 1
}
echo "✓ MediaPipe"

# Check RealSense (optional)
python3 -c "import pyrealsense2" 2>/dev/null && {
    echo "✓ pyrealsense2 (RealSense camera)"
} || {
    echo "⚠ pyrealsense2 not found (will use webcam)"
}

echo ""
echo "Starting GUI test..."
echo "View via FreeRDP or local display"
echo ""
echo "Controls:"
echo "  Q/ESC  - Quit"
echo "  R      - Reset progress"
echo "  SPACE  - Pause/Resume"
echo ""

# Set display if needed
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi

# Run the test
cd "$(dirname "$0")"
python3 face_tracking_gui_test.py

