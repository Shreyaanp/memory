# Screen 7 Placeholder Implementation

## 📋 Overview
Screen 7 has been implemented as a **simple dummy placeholder** as requested. This document explains the implementation details.

---

## 🎯 Implementation Details

### State Definition
```cpp
enum class SystemState {
    // ... other states ...
    PLACEHOLDER_SCREEN_7,    // Screen 7: [PLACEHOLDER - TO BE IMPLEMENTED LATER]
    READY,                   // Connected, waiting for user to start
    // ... more states ...
};
```

### Code Location
**File**: `/home/mercleDev/codebase/src/SystemController.cpp`

**Lines 148-169**: Placeholder implementation in `set_state()` method

```cpp
case SystemState::PLACEHOLDER_SCREEN_7:
    // ═══════════════════════════════════════════════════════════════════
    // SCREEN 7: PLACEHOLDER - TO BE IMPLEMENTED LATER
    // ═══════════════════════════════════════════════════════════════════
    // This is a dummy placeholder as requested. No complex logic here.
    // Just displays a screen and waits for 3 seconds before continuing.
    serial_comm_->send_state(7); // Screen 7: Placeholder screen
    std::cout << "📍 [PLACEHOLDER] Screen 7 displayed (dummy implementation)" << std::endl;
    
    // Auto-transition after 3 seconds (dummy behavior)
    std::thread([this]() {
        std::this_thread::sleep_for(std::chrono::seconds(3));
        if (current_state_ == SystemState::PLACEHOLDER_SCREEN_7) {
            std::cout << "📍 [PLACEHOLDER] Auto-transitioning from Screen 7 → READY" << std::endl;
            set_state(SystemState::READY);
        }
    }).detach();
    break;
```

---

## 🔄 Flow Integration

### Trigger Point
Screen 7 is entered after successful WebSocket authentication:

**File**: `/home/mercleDev/codebase/src/SystemController.cpp`
**Lines 716-723**: In `on_websocket_message()` method

```cpp
if (msg_type == "auth_success") {
    // WebSocket authentication successful
    std::cout << "✅ WebSocket authenticated successfully" << std::endl;
    
    // Server already notifies mobile with "device_connected"
    // No need to send redundant "device_ready" message
    
    // Enter PLACEHOLDER_SCREEN_7 first (dummy screen)
    std::cout << "📍 Entering PLACEHOLDER Screen 7 (will auto-transition)" << std::endl;
    set_state(SystemState::PLACEHOLDER_SCREEN_7);
}
```

### Complete Flow
```
User scans QR (Screen 6: IDLE)
   ↓
QR decoded and decrypted
   ↓
WebSocket connection established
   ↓
Authentication message sent
   ↓
Server responds with "auth_success"
   ↓
**PLACEHOLDER_SCREEN_7** (3 seconds) ⚠️ DUMMY
   ↓
Auto-transition to READY state
   ↓
Wait for mobile to send "start_verification"
   ↓
WARMUP → ALIGN → PROCESSING → etc.
```

---

## ⚙️ Behavior Characteristics

### What It Does
- ✅ Displays screen via serial: `send_state(7)`
- ✅ Logs placeholder status to console
- ✅ Auto-transitions after **3 seconds**
- ✅ Moves to READY state automatically
- ✅ No frame processing (camera idle)
- ✅ No complex logic

### What It Does NOT Do
- ❌ No anti-spoofing logic
- ❌ No high power camera activation
- ❌ No user interaction required
- ❌ No WebSocket communication
- ❌ No face detection
- ❌ No depth analysis
- ❌ No liveness verification

### Camera State
- Camera remains in **LOW POWER mode** during Screen 7
- No camera mode change occurs
- Camera will switch to HIGH POWER when entering READY state

---

## 🔧 Frame Processing

**File**: `/home/mercleDev/codebase/src/SystemController.cpp`
**Lines 301-312**: In `process_frame()` method

```cpp
void SystemController::process_frame(FrameBox* frame) {
    SystemState state = current_state_.load();
    switch (state) {
        case SystemState::AWAIT_ADMIN_QR: handle_await_admin_qr(frame); break;
        case SystemState::IDLE: handle_idle(frame); break;
        case SystemState::PLACEHOLDER_SCREEN_7: break;  // Placeholder - no processing
        case SystemState::READY: handle_idle(frame); break;
        case SystemState::WARMUP: handle_warmup(frame); break;
        case SystemState::ALIGN: handle_align(frame); break;
        default: break;
    }
}
```

**Note**: The placeholder state explicitly does **nothing** during frame processing (line 305).

---

## 📊 Serial Communication

### Screen ID
- **Serial Command**: `send_state(7)`
- **Screen ID**: `7`
- **LilyGo Display**: Should show a simple placeholder screen

### Expected Display
The LilyGo ESP32 AMOLED display should show:
- A simple screen indicating the device is ready
- No interactive elements required
- Can be as simple as a text message like:
  - "Connecting..." or
  - "Ready..." or
  - "Preparing..." or
  - Any placeholder text/graphic

---

## 🚀 How to Replace with Actual Implementation

When ready to implement the actual Screen 7, follow these steps:

### Step 1: Modify State Transition
Replace the auto-transition with actual logic:

```cpp
case SystemState::PLACEHOLDER_SCREEN_7:
    serial_comm_->send_state(7);
    
    // YOUR IMPLEMENTATION HERE
    // - Add camera mode changes if needed
    // - Add WebSocket communication if needed
    // - Add user interaction if needed
    // - Add any other logic
    
    // Remove or modify the auto-transition
    // set_state(SystemState::NEXT_STATE);
    break;
```

### Step 2: Add Frame Processing (if needed)
If Screen 7 needs to process camera frames:

```cpp
void SystemController::process_frame(FrameBox* frame) {
    SystemState state = current_state_.load();
    switch (state) {
        case SystemState::PLACEHOLDER_SCREEN_7: 
            handle_screen_7(frame);  // Add your handler
            break;
        // ... other cases ...
    }
}

// Add new handler method
void SystemController::handle_screen_7(FrameBox* frame) {
    // Your implementation
}
```

### Step 3: Update Camera Configuration (if needed)
If Screen 7 requires high power camera:

```cpp
case SystemState::PLACEHOLDER_SCREEN_7:
    serial_comm_->send_state(7);
    configure_camera_for_state(SystemState::PLACEHOLDER_SCREEN_7);  // Add this
    // ... your logic ...
    break;
```

And update `configure_camera_for_state()`:

```cpp
void SystemController::configure_camera_for_state(SystemState state) {
    // ... existing code ...
    
    if (state == SystemState::PLACEHOLDER_SCREEN_7) {
        // Configure camera for Screen 7
        target_mode = CameraMode::FULL;  // or RGB_ONLY
        // ... camera config ...
    }
    
    // ... rest of method ...
}
```

---

## 🎭 Current vs Future State

| Aspect | Current (Placeholder) | Future (To Implement) |
|--------|----------------------|----------------------|
| Duration | 3 seconds (fixed) | TBD |
| Transition | Auto (timer) | TBD (user action, event, etc.) |
| Camera | LOW POWER | TBD |
| Frame Processing | None | TBD |
| WebSocket | No messages | TBD |
| User Interaction | None | TBD |
| Anti-spoofing | No | TBD |
| Purpose | Placeholder | Actual functionality |

---

## ✅ Compilation Status

The placeholder implementation has been compiled and tested:

```bash
cd /home/mercleDev/codebase
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

**Result**: ✅ **Compilation successful** with no errors.

**Output**: 
- Binary: `/home/mercleDev/codebase/build/mdai_system`
- Library: `/home/mercleDev/codebase/build/libmdai_realsense.so`

---

## 📝 Testing Checklist

When testing the placeholder:

- [ ] Device boots and shows Screen 1-6 correctly
- [ ] User scans QR code from mobile app
- [ ] WebSocket connects and authenticates
- [ ] Screen 7 displays on LilyGo for **3 seconds**
- [ ] Console shows: `📍 [PLACEHOLDER] Screen 7 displayed (dummy implementation)`
- [ ] Auto-transition to READY state occurs
- [ ] Console shows: `📍 [PLACEHOLDER] Auto-transitioning from Screen 7 → READY`
- [ ] Camera remains in LOW POWER mode during Screen 7
- [ ] No crashes or errors during transition

---

## 🔗 Related Files

| File | Purpose |
|------|---------|
| `/home/mercleDev/codebase/include/SystemController.hpp` | State enum definition (line 19-35) |
| `/home/mercleDev/codebase/src/SystemController.cpp` | Full implementation |
| `/home/mercleDev/codebase/BOOT_SEQUENCE_STATUS.md` | Complete boot sequence documentation |

---

## 💡 Summary

Screen 7 is currently a **minimal dummy placeholder** that:
1. Displays a screen via serial command
2. Waits 3 seconds
3. Auto-transitions to READY state
4. Does nothing else

This allows the system to continue functioning while Screen 7's actual requirements are being finalized. The placeholder can be easily replaced with full implementation when ready.

**Status**: ✅ **PLACEHOLDER COMPLETE** - Ready for future enhancement

