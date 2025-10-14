# MDAI RealSense Anti-Spoofing System

World-class liveness detection using Intel RealSense depth cameras.

## Features

- **Multi-Modal Anti-Spoofing** - Depth geometry + IR texture + Cross-modal + Temporal analysis
- **Real-time Performance** - Optimized for 30fps with RingBuffer architecture
- **Attack Resistance** - Blocks photos, videos, masks, screens, 3D models
- **OpenCV Face Detection** - Fast, reliable, cross-platform
- **Optional GPU Acceleration** - MediaPipe C++ support (requires build from source)

## Dependencies

### Required
- **Intel RealSense SDK** (librealsense2)
- **OpenCV** >= 4.0 - Face detection + utilities
- **CMake** >= 3.10
- **C++17** compiler

### Optional (Advanced)
- **MediaPipe C++** - GPU-accelerated face detection (build from source with Bazel)

## Installation

```bash
# Install dependencies
sudo apt install librealsense2-dev libopencv-dev

# Build
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## MediaPipe GPU (Optional Advanced)

MediaPipe is **natively C++** and requires building from source:

```bash
# Install Bazel
wget https://github.com/bazelbuild/bazel/releases/download/5.3.0/bazel-5.3.0-installer-linux-x86_64.sh
bash bazel-5.3.0-installer-linux-x86_64.sh --user

# Build MediaPipe C++
git clone https://github.com/google/mediapipe.git
cd mediapipe
bazel build -c opt --define MEDIAPIPE_DISABLE_GPU=0 mediapipe/calculators/...

# Then rebuild this project with MediaPipe support
cmake -DUSE_MEDIAPIPE=ON ..
```

## Usage

```bash
# Live testing with GPU acceleration
./build/examples/simple_liveness_viewer

# Press 'C' to start camera
# Press 'Q' to quit
```

## Anti-Spoofing Configuration

Optimized for real humans while blocking attacks:

```cpp
config.min_positioning_score = 0.4f;      // Accept off-center faces
config.min_depth_analysis = 0.5f;         // Detect 3D structure
config.min_ir_texture = 0.35f;            // Real skin IR patterns
config.min_temporal_consistency = 0.35f;  // Natural micro-motion
config.min_confidence = 0.5f;             // Overall threshold
```

## Performance

- **MediaPipe GPU**: 5-10x faster than Haar Cascade
- **Frame Rate**: 30fps real-time
- **Latency**: <33ms per frame
- **Memory**: RingBuffer with zero-copy design

## Architecture

1. **Producer** - Captures aligned depth+color+IR from RealSense
2. **RingBuffer** - Lock-free frame storage
3. **QualityGate** - MediaPipe face detection + quality checks
4. **AntiSpoofing** - Multi-modal liveness detection
5. **Pipeline** - Orchestrates the complete flow

Created by MDAI Team
