# IR-Only System Migration Plan

## 🎯 Goal
Simplify the MDAI system to use **single IR camera only** - no RGB, no depth, no laser, no anti-spoofing.

---

## 📊 Current vs Target Architecture

### BEFORE (Complex)
```
RealSense Camera
├── Color Stream (848x480 RGB)
├── Depth Stream (848x480 Z16)
├── IR Left Stream (848x480 Y8)
├── IR Right Stream (848x480 Y8)
└── Laser Emitter (on/off)

Processing Pipeline:
├── Producer → FrameBox (all 4 streams)
├── FaceDetector (on RGB)
├── QualityGate (multi-modal)
├── AntiSpoofing (depth + IR + temporal)
└── SystemController (2 camera modes)
```

### AFTER (Simple)
```
RealSense Camera (or any IR camera)
└── IR Stream (848x480 Y8) - passive, no laser

Processing Pipeline:
├── Producer → FrameBox (IR only)
├── FaceDetector (IR→BGR conversion)
└── SystemController (single mode)
```

---

## 📋 Phase-by-Phase Implementation

### Phase 1: Delete Anti-Spoofing ❌
**Files to DELETE:**
- `src/AntiSpoofing.cpp` (~4700 lines)
- `include/AntiSpoofing.hpp` (~330 lines)

**Why:** Entire anti-spoofing system is being removed.

---

### Phase 2: Simplify FrameBox 📦
**File:** `include/FrameBox.hpp`, `src/FrameBox.cpp`

**REMOVE:**
```cpp
// Frame data vectors
std::vector<uint16_t> depth_data;      // ❌ Remove
std::vector<uint8_t> color_data;       // ❌ Remove  
std::vector<uint8_t> ir_right_data;    // ❌ Remove

// Dimensions
int depth_width, depth_height;         // ❌ Remove
int color_width, color_height;         // ❌ Remove

// Camera parameters
rs2_intrinsics depth_intrinsics;       // ❌ Remove
rs2_intrinsics color_intrinsics;       // ❌ Remove
rs2_extrinsics extrinsics_depth_to_color; // ❌ Remove
float depth_scale;                     // ❌ Remove

// Settings
int emitter_state;                     // ❌ Remove
float laser_power;                     // ❌ Remove

// Metadata structs
QualityGateResults quality_gate;       // ❌ Remove
AntiSpoofingResults anti_spoofing;     // ❌ Remove
```

**KEEP:**
```cpp
// IR data (rename from ir_left_data to ir_data)
std::vector<uint8_t> ir_data;          // ✅ Keep (renamed)
int ir_width, ir_height;               // ✅ Keep

// Face detection results
bool face_detected;                    // ✅ Keep
int face_x, face_y, face_w, face_h;   // ✅ Keep
float face_detection_confidence;       // ✅ Keep
std::vector<Landmark> landmarks;       // ✅ Keep (478 from MediaPipe)

// Timestamps
double time_ir;                        // ✅ Keep (rename from time_ir_left)
uint64_t sequence_id;                  // ✅ Keep
```

---

### Phase 3: Simplify Producer 📹
**Files:** `include/Producer.hpp`, `src/Producer.cpp`

**REMOVE:**
```cpp
// Stream configuration
bool enable_depth;                     // ❌ Remove
bool enable_ir;                        // ❌ Remove (always IR)
int depth_width, depth_height;         // ❌ Remove
int color_width, color_height;         // ❌ Remove

// Sensor options
int emitter_enabled;                   // ❌ Remove
float laser_power;                     // ❌ Remove

// Post-processing filters
rs2::spatial_filter spatial_filter_;   // ❌ Remove
rs2::temporal_filter temporal_filter_; // ❌ Remove
rs2::hole_filling_filter hole_filling_filter_; // ❌ Remove
rs2::align* aligner_;                  // ❌ Remove

// configure_pipeline() complexity
enable_stream(RS2_STREAM_DEPTH, ...);  // ❌ Remove
enable_stream(RS2_STREAM_COLOR, ...);  // ❌ Remove
enable_stream(RS2_STREAM_INFRARED, 2, ...); // ❌ Remove (right IR)
```

**SIMPLIFIED configure_pipeline():**
```cpp
bool Producer::configure_pipeline() {
    // Only enable single IR stream
    rs_config_.enable_stream(
        RS2_STREAM_INFRARED, 1,  // Left IR only
        config_.ir_width,        // 848
        config_.ir_height,       // 480
        RS2_FORMAT_Y8,           // Grayscale
        config_.ir_fps           // 30
    );
    
    // Disable laser emitter for passive IR
    // (done in configure_sensor_options)
    
    return true;
}
```

**NEW CameraConfig (simplified):**
```cpp
struct CameraConfig {
    int ir_width = 848;
    int ir_height = 480;
    int ir_fps = 30;
    std::string device_serial = "";  // Optional device selection
};
```

---

### Phase 4: Update FaceDetector 👤
**Files:** `include/FaceDetector.hpp`, `src/FaceDetector.cpp`

**CHANGE:** Accept IR grayscale input, convert internally

```cpp
// OLD: expects color Mat
bool MediaPipeFaceDetector::detect(FrameBox* frame) {
    cv::Mat color_mat = frame->get_color_mat();  // ❌ OLD
    // ...
}

// NEW: accepts IR grayscale, converts to BGR
bool MediaPipeFaceDetector::detect(FrameBox* frame) {
    cv::Mat ir_gray = frame->get_ir_mat();       // ✅ NEW
    if (ir_gray.empty()) return false;
    
    // Convert IR grayscale to BGR for MediaPipe
    cv::Mat bgr;
    cv::cvtColor(ir_gray, bgr, cv::COLOR_GRAY2BGR);
    
    // Feed to MediaPipe (rest unchanged)
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    // ...
}
```

**Add to FrameBox:**
```cpp
cv::Mat FrameBox::get_ir_mat() const {
    if (ir_data.empty() || ir_width <= 0 || ir_height <= 0) {
        return cv::Mat();
    }
    return cv::Mat(ir_height, ir_width, CV_8UC1, 
                   const_cast<uint8_t*>(ir_data.data()));
}
```

---

### Phase 5: Simplify SystemController 🎮
**Files:** `include/SystemController.hpp`, `src/SystemController.cpp`

**REMOVE:**
```cpp
// Camera modes
enum class CameraMode { UNINITIALIZED, RGB_ONLY, FULL }; // ❌ Remove entire enum
std::atomic<CameraMode> current_camera_mode_;            // ❌ Remove

// configure_camera_for_state() - entire function  // ❌ Remove
void configure_camera_for_state(SystemState state);

// Anti-spoofing processing
void handle_processing();                          // ❌ Simplify heavily
std::unique_ptr<StatsService> stats_;             // ❌ Remove

// Quality gate checks
bool quality_gate_passed = ...                    // ❌ Remove
```

**SIMPLIFY handle_processing():**
```cpp
// OLD: Complex batch anti-spoofing analysis
void SystemController::handle_processing() {
    // ~500 lines of anti-spoofing code
}

// NEW: Simple - just send result
void SystemController::handle_processing() {
    // Get best frame from buffer
    FrameBox* best = ring_buffer_->get_frame_with_best_face();
    
    if (best && best->metadata.face_detected) {
        // Encode image and send to server
        // (face image upload for server-side processing)
        set_state(SystemState::SUCCESS);
    } else {
        set_state(SystemState::ERROR);
    }
}
```

**SIMPLIFY camera initialization:**
```cpp
// OLD: Different modes for different states
configure_camera_for_state(SystemState::IDLE);      // RGB_ONLY
configure_camera_for_state(SystemState::ALIGN);     // FULL

// NEW: Single initialization at boot
void SystemController::initialize_camera() {
    CameraConfig config;
    config.ir_width = 848;
    config.ir_height = 480;
    config.ir_fps = 30;
    
    producer_ = std::make_unique<Producer>(config, ring_buffer_.get());
    producer_->start();
}
```

---

### Phase 6: Update CMakeLists.txt 🔧
**File:** `CMakeLists.txt`

```cmake
# REMOVE AntiSpoofing.cpp from sources
set(SOURCES
    src/FrameBox.cpp
    src/RingBuffer.cpp
    src/Producer.cpp
    src/CameraInitializer.cpp
    # src/AntiSpoofing.cpp      # ❌ REMOVE
    src/FaceDetector.cpp
    src/NativeFaceLandmarker.cpp
    src/SerialCommunicator.cpp
    src/NetworkManager.cpp
    src/CryptoUtils.cpp
    src/SystemController.cpp
    src/TrustZoneIdentity.cpp
)
```

---

### Phase 7: Update CameraConfig Struct 📝
**File:** `include/Producer.hpp`

```cpp
// OLD (complex)
struct CameraConfig {
    bool enable_depth = true;
    int depth_width = 848;
    int depth_height = 480;
    int depth_fps = 30;
    
    int color_width = 848;
    int color_height = 480;
    int color_fps = 30;
    
    bool enable_ir = true;
    int ir_width = 848;
    int ir_height = 480;
    int ir_fps = 30;
    
    int emitter_enabled = 1;
    float laser_power = 200.0f;
    bool auto_exposure = false;
    float manual_exposure = 8000.0f;
    float manual_gain = 32.0f;
    
    bool enable_spatial_filter = true;
    float spatial_alpha = 0.5f;
    float spatial_delta = 20.0f;
    bool enable_temporal_filter = true;
    float temporal_alpha = 0.4f;
    float temporal_delta = 20.0f;
    bool enable_hole_filling = true;
    bool align_to_color = true;
    
    std::string device_serial = "";
    bool quick_init = false;
};

// NEW (simple)
struct CameraConfig {
    int ir_width = 848;
    int ir_height = 480;
    int ir_fps = 30;
    std::string device_serial = "";  // Optional
};
```

---

### Phase 8: Test & Verify ✅

1. **Build test:**
   ```bash
   cd /home/mercleDev/codebase/build
   cmake .. && make -j$(nproc)
   ```

2. **Unit test:**
   ```bash
   cd /home/mercleDev/codebase/tests
   python test_ir_face_detection.py --live
   ```

3. **Integration test:**
   ```bash
   ./build/mdai_system
   # Verify:
   # - Camera starts with IR only
   # - Face detection works
   # - Nose tracking works
   # - No errors about missing depth/color
   ```

---

## 📊 Code Reduction Summary

| Component | Before | After | Reduction |
|-----------|--------|-------|-----------|
| AntiSpoofing.cpp | 4700 lines | 0 lines | -4700 |
| AntiSpoofing.hpp | 330 lines | 0 lines | -330 |
| FrameBox.hpp | 378 lines | ~150 lines | -228 |
| Producer.cpp | 670 lines | ~300 lines | -370 |
| SystemController.cpp | 2369 lines | ~1500 lines | -869 |
| **TOTAL** | ~8500 lines | ~2000 lines | **~6500 lines removed** |

---

## ⚠️ Breaking Changes

1. **No depth data** - Any code expecting `frame->depth_data` will fail
2. **No color data** - Any code expecting `frame->get_color_mat()` will fail
3. **No anti-spoofing** - `frame->metadata.anti_spoofing` struct gone
4. **No camera modes** - `configure_camera_for_state()` gone
5. **No quality gates** - `frame->metadata.quality_gate` struct gone

---

## 🚀 Ready to Proceed?

Review this plan and confirm to start implementation. I'll work through each phase sequentially, testing after each major change.

