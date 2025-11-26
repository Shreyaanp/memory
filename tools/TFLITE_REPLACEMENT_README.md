# TFLite Replacement for MediaPipe

## Overview

This document describes the replacement of MediaPipe with TFLite (`face_landmark.tflite`) for face mesh detection.

## Files Created

### 1. `face_tracking_gui_test_tflite.py`
A drop-in replacement for `face_tracking_gui_test.py` that uses TFLite instead of MediaPipe.

**Key Features:**
- Uses `face_landmark.tflite` model (468 landmarks)
- Compatible interface with MediaPipe version
- Uses OpenCV Haar cascade for face detection (required before TFLite inference)
- Same visualization and tracking features

**Usage:**
```bash
cd /home/mercleDev/codebase/tools
python3 face_tracking_gui_test_tflite.py
```

**Differences from MediaPipe version:**
- Requires face detection step (Haar cascade) before landmark detection
- Model expects 192x192 RGB input (normalized to [0,1])
- Outputs 468 landmarks in same format as MediaPipe

## Model Details

**Model:** `/home/mercleDev/codebase/tools/face_landmark.tflite`

**Input:**
- Shape: `[1, 192, 192, 3]`
- Type: `float32`
- Format: RGB normalized to [0, 1]

**Output:**
- Output 0: `[1, 1, 1, 1404]` - 468 landmarks × 3 (x, y, z)
- Output 1: `[1, 1, 1, 1]` - Confidence score

## Testing

### Test the TFLite model:
```bash
python3 test_face_landmark_tflite.py
```

### Test with image:
```bash
python3 test_landmark_correct.py
```

### Run GUI test:
```bash
python3 face_tracking_gui_test_tflite.py
```

## Screen0 Test

Screen0 is a UI test screen in the C++ application (`UI_Application`) that displays face tracking results. It receives serial data from the backend.

**Current Status:**
- Python GUI test: ✅ Replaced with TFLite
- C++ Backend: ⚠️ Still uses MediaPipe wrapper

## Next Steps for C++ Backend

To replace MediaPipe in the C++ backend (`/home/mercleDev/codebase/src/FaceDetector.cpp`):

1. **Option A: Use TensorFlow Lite C++ API**
   - Requires TensorFlow Lite C++ library
   - More complex integration
   - Better performance

2. **Option B: Create Python bridge (similar to MediaPipe wrapper)**
   - Use existing Python TFLite code
   - Create C++ wrapper that calls Python
   - Easier but less efficient

3. **Option C: Use OpenCV DNN with TFLite**
   - OpenCV can load TFLite models
   - Simpler integration
   - May have limitations

## Dependencies

**Python:**
- `tflite_runtime` or `tensorflow`
- `opencv-python`
- `numpy`
- `pyrealsense2` (optional, for RealSense camera)

**Install:**
```bash
pip3 install tflite-runtime opencv-python numpy pyrealsense2
# OR
pip3 install tensorflow opencv-python numpy pyrealsense2
```

## Performance Improvements (v2)

The TFLite version has been improved to match MediaPipe's stability:

### 1. **ROI Tracking & Smoothing**
- Tracks face ROI across frames
- Smooths ROI position and size (70% new, 30% old)
- Falls back to previous ROI if detection fails temporarily

### 2. **Temporal Landmark Smoothing**
- Applies exponential moving average to landmarks
- Reduces jitter: 60% new landmarks, 40% previous
- Z-coordinate smoothed less (depth changes faster)

### 3. **Better Face Detection**
- **Primary**: OpenCV DNN face detector (if available)
  - More stable than Haar cascade
  - Better accuracy
- **Fallback**: Improved Haar cascade parameters
  - Smaller scale steps (1.05 vs 1.1)
  - Higher min neighbors (7 vs 5)
  - Larger minimum size (80x80 vs 50x50)

### 4. **Detection Failure Handling**
- Uses previous ROI for up to 3 frames if detection fails
- Prevents sudden jumps when face temporarily occluded

**Result**: Much smoother, more stable tracking similar to MediaPipe!

## Troubleshooting

### Issue: "No face detected"
- Check that face detector (DNN or Haar cascade) is loading correctly
- Try adjusting face detection parameters
- Ensure good lighting and face visibility
- Check if DNN model files exist: `/home/mercleDev/codebase/models/res10_300x300_ssd_iter_140000.caffemodel`

### Issue: "Model not found"
- Verify model path: `/home/mercleDev/codebase/tools/face_landmark.tflite`
- Check file permissions

### Issue: "TFLite runtime not found"
- Install: `pip3 install tflite-runtime`
- Or: `pip3 install tensorflow`

### Issue: Still glitchy
- The script will automatically use DNN if available (more stable)
- If using Haar cascade, ensure good lighting
- Adjust smoothing factors in code if needed:
  - `roi_alpha`: ROI smoothing (default 0.7)
  - `landmark_alpha`: Landmark smoothing (default 0.6)

## Notes

- The TFLite model only does landmark detection, not face detection
- Face detection is done separately using OpenCV Haar cascade
- Landmarks are mapped from 192x192 space back to original image coordinates
- All 468 landmarks follow MediaPipe topology (same indices)

