# MDai RealSense Face Detection System

A high-performance, production-ready face detection system for Intel RealSense D435 depth camera with non-blocking pipeline architecture.

## 🎯 Features

- **Non-blocking Producer**: Captures frames at 30fps without dropping during recording
- **Dynamic Ring Buffer**: Starts at 32 slots, grows up to 5GB as needed
- **Pipeline Architecture**: 
  - Blocking within pipelines (sequential stages)
  - Non-blocking across pipelines (independent execution)
- **Extensible Metadata**: Add custom processing results without modifying core structure
- **Python Support**: Python consumers can process frames alongside C++ pipelines
- **Utility Functions**: Save frames as PNG/JPEG, export point clouds, visualization

## 📁 Project Structure

```
mdairealsense/
├── include/                 # C++ headers
│   ├── FrameBox.hpp        # Core data structure with extensible metadata
│   ├── RingBuffer.hpp      # Dynamic ring buffer (32 slots → 5GB)
│   ├── Producer.hpp        # Camera capture (non-blocking poll_for_frames)
│   ├── Pipeline.hpp        # Pipeline orchestrator
│   └── Utils.hpp           # Utility functions (save, export, visualize)
├── src/                    # C++ implementations
├── python/                 # Python bindings
│   └── mdai_realsense.py   # Python API and example stages
├── examples/               # Example applications
│   ├── basic_capture.cpp   # Simple capture example
│   ├── pipeline_example.cpp # Full pipeline with anti-spoofing
│   └── frame_saver.cpp     # Save frames to disk
└── CMakeLists.txt         # Build configuration
```

## 🏗️ Architecture

### System Overview

```
┌─────────────────────────────────────────────────┐
│  PRODUCER (C++)                                 │
│  - Non-blocking poll_for_frames()               │
│  - Never drops during recording                 │
│  - Packages frames + metadata → FrameBox        │
└────────────────┬────────────────────────────────┘
                 ↓
┌─────────────────────────────────────────────────┐
│  DYNAMIC RING BUFFER                            │
│  - 32 slots initially, grows to 5GB max         │
│  - Lock-free, reference-counted slots           │
│  - Drop-oldest only when recording inactive     │
└────────────────┬────────────────────────────────┘
                 ↓
┌─────────────────────────────────────────────────┐
│  PIPELINES (C++ and Python)                     │
│                                                  │
│  Pipeline X: Anti-Spoofing (REALTIME)           │
│    Stage 1 → Stage 2 → Stage 3 (BLOCKING)      │
│                                                  │
│  Pipeline Y: Logging (SEQUENTIAL)               │
│    Stage 1 → Stage 2 (BLOCKING)                │
│                                                  │
│  Pipelines run INDEPENDENTLY of each other      │
└─────────────────────────────────────────────────┘
```

### Key Design Principles

1. **Producer Never Blocks**: Uses `poll_for_frames()` and dynamic buffer growth
2. **Recording Mode**: When active, buffer grows instead of dropping frames
3. **Pipeline Independence**: Each pipeline runs in its own thread
4. **Stage Dependencies**: Pipelines can wait for other pipeline results via metadata
5. **Extensible Metadata**: Add custom fields without changing core structures

## 🔧 Dependencies

- **RealSense SDK** 2.x (`librealsense2`)
- **OpenCV** 4.x (for image processing and saving)
- **C++17** compiler
- **CMake** 3.10+
- **Python 3.6+** (for Python bindings, optional)

## 🚀 Building

```bash
cd /home/ichiro/mdairealsense

# Create build directory
mkdir -p build && cd build

# Configure
cmake ..

# Build
make -j$(nproc)

# Install (optional)
sudo make install
```

## 📖 Usage Examples

### Basic Capture (C++)

```cpp
#include "Producer.hpp"
#include "RingBuffer.hpp"

// Create ring buffer
DynamicRingBuffer ring_buffer(32);

// Configure camera
CameraConfig config;
config.depth_width = 848;
config.depth_height = 480;
config.depth_fps = 30;

// Create and start producer
Producer producer(config, &ring_buffer);
producer.start();

// Get frames
FrameBox* frame = ring_buffer.get_latest_frame();
if (frame) {
    // Process frame...
    ring_buffer.release_frame(frame);
}
```

### Pipeline with Anti-Spoofing (C++)

```cpp
// Create pipeline
auto pipeline = std::make_unique<Pipeline>(
    "AntiSpoofing", 
    &ring_buffer, 
    ConsumerMode::REALTIME
);

// Add stages
pipeline->add_stage(std::make_unique<AntiSpoofingStage>());
pipeline->add_stage(std::make_unique<FaceDetectionStage>());

// Start
pipeline->start();
```

### Python Consumer

```python
from python.mdai_realsense import (
    SimplePipeline, AntiSpoofingStage, FaceRecognitionStage
)

# Create pipeline
pipeline = SimplePipeline()
pipeline.add_stage(AntiSpoofingStage(threshold=0.5))
pipeline.add_stage(FaceRecognitionStage())

# Start
pipeline.start()

# Process frames (from ring buffer)
for frame in consumer.get_frames():
    pipeline.process_frame(frame)
```

## 🎬 Running Examples

```bash
# Basic capture
./build/examples/basic_capture

# Full pipeline example
./build/examples/pipeline_example

# Frame saver
./build/examples/frame_saver ./output_directory
```

## 🔬 FrameBox Structure

```cpp
struct FrameBox {
    // Core data
    uint64_t sequence_id;
    rs2::depth_frame depth;
    rs2::video_frame color;
    rs2::video_frame ir_left, ir_right;
    
    // Camera parameters (queried at runtime)
    rs2_intrinsics depth_intrinsics;
    rs2_intrinsics color_intrinsics;
    rs2_extrinsics extrinsics_depth_to_color;
    float depth_scale;
    
    // Extensible metadata
    FrameBoxMetadata metadata {
        float anti_spoofing_score;
        bool is_valid_frame;
        string detected_face_id;
        // ... add custom fields
    };
};
```

## 🔄 Pipeline Dependencies

Pipelines can wait for other pipeline results:

```cpp
// In Stage 2 of Pipeline Y
bool process(FrameBox* frame) {
    // Wait for Pipeline X to complete anti-spoofing
    if (!frame->wait_for_stage("anti_spoofing", 1000)) {
        return false; // Timeout
    }
    
    // Check result
    if (frame->metadata.is_valid_frame) {
        // Process only if anti-spoofing passed
    }
}
```

## 📊 Performance Characteristics

- **Capture Rate**: 30 fps (depth + color + IR)
- **Latency**: ~33ms (realtime mode)
- **Memory**: 32 slots = ~64MB, grows to 5GB max
- **Buffer Growth**: 2x when full during recording
- **No Frame Drops**: During active recording (buffer grows instead)

## 🛠️ Camera Configuration

```cpp
CameraConfig config;

// Streams
config.depth_width = 848;
config.depth_height = 480;
config.depth_fps = 30;

// Sensor options
config.internal_queue_size = 2;        // Low latency
config.emitter_enabled = 1;            // Laser on
config.laser_power = 150.0f;           // mW

// Exposure
config.auto_exposure = true;           // Or manual
config.manual_exposure = 16000.0f;     // μs
config.manual_gain = 64.0f;

// Post-processing
config.enable_spatial_filter = true;
config.enable_temporal_filter = true;
config.align_to_color = true;
```

## 🧪 Testing Without Camera

The Python module includes mock frame generation:

```python
from python.mdai_realsense import create_mock_frame

frame = create_mock_frame(sequence_id=0)
# Returns FrameData with synthetic depth and color
```

## 📝 Creating Custom Stages

### C++ Stage

```cpp
class MyCustomStage : public PipelineStage {
public:
    bool process(FrameBox* frame) override {
        // Your processing logic
        frame->metadata.scores["my_score"] = 0.95f;
        frame->notify_stage_complete("my_stage");
        return true;
    }
    
    std::string get_name() const override {
        return "MyCustomStage";
    }
};
```

### Python Stage

```python
class MyCustomStage(PipelineStage):
    def process(self, frame: FrameData) -> bool:
        # Your processing logic
        frame.metadata['my_score'] = 0.95
        return True
```

## 🔍 Debugging & Monitoring

The system provides real-time statistics:

```cpp
// Producer stats
float fps = producer.get_fps();
uint64_t total = producer.get_total_frames_captured();

// Ring buffer stats
size_t capacity = ring_buffer.get_capacity();
size_t usage = ring_buffer.get_usage();
size_t memory_mb = ring_buffer.get_memory_usage() / (1024*1024);
uint64_t dropped = ring_buffer.get_total_frames_dropped();

// Pipeline stats
uint64_t processed = pipeline->get_frames_processed();
uint64_t dropped = pipeline->get_frames_dropped();
```

## 🐛 Troubleshooting

### Camera not found
```bash
# Check RealSense devices
rs-enumerate-devices

# Check permissions
sudo usermod -a -G video $USER
```

### Build errors
```bash
# Check dependencies
pkg-config --modversion realsense2
pkg-config --modversion opencv4
```

### Memory issues
- Reduce `max_memory_bytes` in RingBuffer constructor
- Decrease buffer initial capacity
- Enable frame drops when not recording

## 📚 API Reference

See header files in `include/` for complete API documentation:
- `FrameBox.hpp` - Frame data structure
- `RingBuffer.hpp` - Ring buffer API
- `Producer.hpp` - Camera capture API
- `Pipeline.hpp` - Pipeline orchestrator
- `Utils.hpp` - Utility functions

## 🤝 Contributing

This is based on the Intel RealSense D435 specification document.
See `Face_Detection_D435_Textbook_Latest.pdf` for hardware details.

## 📄 License

[Add your license here]

## 🙏 Acknowledgments

- Intel RealSense SDK
- OpenCV
- Design based on LMAX Disruptor pattern

# memory
