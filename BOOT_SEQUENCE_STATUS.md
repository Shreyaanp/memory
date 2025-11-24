# RDK Boot Sequence Implementation Status

## 📋 Overview
This document tracks the implementation status of the RDK boot sequence as per the plan.

---

## ✅ FULLY IMPLEMENTED SCREENS (1-6)

### Screen 1: Booting Screen (Logo) ✅
- **State**: `SystemState::BOOT`
- **Serial Command**: `send_state(1)`
- **Duration**: 3 seconds (preemptive)
- **Implementation**: `SystemController::handle_boot()` line 849-851
- **Behavior**: 
  - Displays Mercle logo
  - Kernel-level control loop
  - Can be preemptive

---

### Screen 2: Actual Booting Phase ✅
- **State**: `SystemState::BOOT` (continuation)
- **Serial Command**: `send_state(2)`
- **Duration**: Variable (based on service startup time)
- **Implementation**: `SystemController::handle_boot()` line 854-879
- **Behavior**:
  - **NOT preemptive** (as specified)
  - Starts services sequentially:
    1. **WiFi Service**: Checks for known connections (last connected prioritized)
    2. **Camera Service**: Initializes in LOW POWER mode (RGB only, 640x480@30fps)
    3. **Display Service**: Verifies serial communication
  - Progress bar UI (low priority - can be added to LilyGo screen later)
  - **Error Handling**:
    - If camera fails → Display error text → **STOP HERE** ✅
    - If display fails → Continue with warning (non-fatal)

---

### Screen 3: Looking for WiFi QR ✅
- **State**: `SystemState::AWAIT_ADMIN_QR`
- **Serial Command**: `send_state(3)`
- **Duration**: Until WiFi connected
- **Implementation**: `SystemController::handle_boot()` line 893-898 + `handle_await_admin_qr()`
- **Behavior**:
  - **Only displays if WiFi is NOT already connected** ✅
  - If WiFi is connected on boot → **Skip this screen** ✅
  - Stays on this page until WiFi connection established
  - Camera scans for WiFi QR codes continuously
  - Uses encrypted challenge-response authentication

---

### Screen 4: Connected to WiFi ✅
- **State**: `SystemState::PROVISIONED`
- **Serial Command**: `send_state(4)`
- **Duration**: 3 seconds minimum ✅
- **Implementation**: `SystemController::set_state()` line 132-140
- **Behavior**:
  - Displays WiFi name (SSID) ✅
  - **Directly displayed if WiFi was already connected** ✅
  - Auto-transitions to Screen 5 after 3 seconds

---

### Screen 5: mDai Ready Text Display ✅
- **State**: `SystemState::PROVISIONED` (transition)
- **Serial Command**: `send_state(5)`
- **Duration**: 2 seconds ✅
- **Implementation**: `SystemController::set_state()` line 136-138
- **Behavior**:
  - Shows "mDai Ready" text
  - **Preemptive for 2 seconds** ✅
  - Auto-transitions to Screen 6 (IDLE)

---

### Screen 6: Idle Phase ✅
- **State**: `SystemState::IDLE`
- **Serial Command**: `send_state(6)`
- **Duration**: Until QR scanned
- **Implementation**: `SystemController::set_state()` line 142-145 + `handle_idle()` line 337-493
- **Behavior**:
  - Waiting for **User QR** or **WiFi Change QR** ✅
  - **Modular QR System** (extensible for future QR types) ✅:
    - **User QR**: Contains session token for WebSocket connection
    - **WiFi QR**: Contains encrypted WiFi credentials
  - QR Detection:
    - Uses ZBar library (more robust than OpenCV)
    - Processes every 5th frame for efficiency
    - Differentiates QR types by JSON structure
  - QR Decryption:
    - **Session QR**: AES-256-CBC with shared key
    - **WiFi QR**: Device-key encryption with challenge-response
  - After successful Session QR scan → Connects to WebSocket → Screen 7

---

## 📍 PLACEHOLDER IMPLEMENTATION

### Screen 7: [PLACEHOLDER - TO BE IMPLEMENTED LATER] ⚠️
- **State**: `SystemState::PLACEHOLDER_SCREEN_7`
- **Serial Command**: `send_state(7)`
- **Duration**: 3 seconds (auto-transition)
- **Implementation**: `SystemController::set_state()` line 148-169
- **Behavior**:
  - **DUMMY IMPLEMENTATION** as requested ✅
  - No complex logic - just displays a screen
  - Auto-transitions to READY state after 3 seconds
  - Camera remains in LOW POWER mode during this screen
  - **To be replaced later** with actual Screen 7 implementation
- **Notes**:
  - Currently triggered after successful WebSocket authentication
  - Placeholder for future functionality
  - Does not process frames (no camera logic)

---

## 🔄 SUBSEQUENT SCREENS (After Screen 7)

### READY State
- Waits for mobile app to send "start_verification" command
- Camera switches from LOW POWER → HIGH POWER mode
- 60-second timeout (returns to IDLE if user doesn't start)

### WARMUP State (Screen 8)
- Camera warming up in HIGH POWER mode
- Duration: 3 seconds
- No user interaction required

### ALIGN State (Screen 9)
- Active liveness detection
- User performs circular head motion
- Nose tracking displayed on screen
- Recording frames for anti-spoofing

### PROCESSING State (Screen 10)
- Batch anti-spoofing analysis
- Progress bar shown
- Requires 5+ consecutive passes for success

### SUCCESS State (Screen 11)
- Verification successful
- Display for 5 seconds
- Transitions to LOGOUT

### ERROR State (Screen 12)
- Verification failed
- Display error message
- Returns to IDLE after 5 seconds

### LOGOUT State (Screen 13)
- "Thank you" message
- Camera switches HIGH POWER → LOW POWER
- WebSocket disconnected
- Returns to IDLE

---

## 🎥 CAMERA MODE MANAGEMENT

### Low Power Mode (RGB Only)
- **Resolution**: 640x480@30fps
- **Features**: RGB camera only
- **Power**: No IR, no depth, no laser (0mW)
- **States**: BOOT, AWAIT_ADMIN_QR, IDLE, PLACEHOLDER_SCREEN_7
- **Behavior**: **Persistent - never closed once started** ✅
- **Purpose**: QR scanning, idle monitoring

### High Power Mode (Full Sensors)
- **Resolution**: RGB 1280x720@30fps, Depth 848x480
- **Features**: RGB + Depth + IR + Laser (300mW)
- **States**: READY, WARMUP, ALIGN, PROCESSING
- **Behavior**: Only activated during liveness verification
- **Purpose**: Anti-spoofing, depth analysis

### Camera Transition Logic
```cpp
LOW POWER → HIGH POWER: Safe restart (line 274-277)
HIGH POWER → LOW POWER: Safe downgrade (line 278-281)
SAME MODE: No reconfiguration needed (line 267-270)
```

---

## 🔐 QR CODE TYPES (Modular Design)

### 1. User QR (Session Token)
- **Format**: JSON with `qr_encrypted` field
- **Encryption**: AES-256-CBC
- **Payload**: 
  ```json
  {
    "session_id": "uuid",
    "token": "jwt_token",
    "timestamp": 1234567890
  }
  ```
- **Purpose**: Connect RDK to mobile app via WebSocket
- **Validation**: 5-minute expiry check
- **State Transition**: IDLE → PLACEHOLDER_SCREEN_7 → READY

### 2. WiFi QR (Admin Provisioning)
- **Format**: Raw encrypted challenge string or JSON without `qr_encrypted`
- **Encryption**: Device-key encryption (RSA + AES)
- **Payload**: Challenge string validated with server
- **Purpose**: Configure WiFi on RDK device
- **Validation**: Challenge-response with EC2 server
- **State Transition**: AWAIT_ADMIN_QR → WIFI_CHANGE_CONNECTING → PROVISIONED → IDLE

### Future QR Types (Extensible)
- Easy to add more QR types due to modular detection in `handle_idle()`
- Detection logic: JSON field checking (line 392-411)
- Each type has its own decryption + validation path

---

## 📊 IMPLEMENTATION QUALITY METRICS

| Feature | Status | Code Location | Notes |
|---------|--------|---------------|-------|
| Screen 1 (Logo) | ✅ Complete | `handle_boot()` L849 | 3s preemptive |
| Screen 2 (Booting) | ✅ Complete | `handle_boot()` L854 | Service checks |
| Screen 3 (WiFi QR) | ✅ Complete | `handle_boot()` L893 | Conditional display |
| Screen 4 (WiFi Connected) | ✅ Complete | `set_state()` L132 | Shows SSID |
| Screen 5 (mDai Ready) | ✅ Complete | `set_state()` L136 | 2s preemptive |
| Screen 6 (Idle/Scan QR) | ✅ Complete | `handle_idle()` L337 | Dual QR types |
| Screen 7 (Placeholder) | ⚠️ Dummy | `set_state()` L148 | To be implemented |
| Low Power Camera | ✅ Complete | `configure_camera()` L225 | Persistent |
| High Power Camera | ✅ Complete | `configure_camera()` L253 | On-demand |
| Error Handling | ✅ Complete | Throughout | Stops on fatal |
| WiFi Auto-detect | ✅ Complete | `check_wifi_on_boot()` L838 | Skips Screen 3 |
| Modular QR System | ✅ Complete | `handle_idle()` L386 | Extensible |

---

## 🎯 CURRENT STATE SUMMARY

### ✅ COMPLETE (As Per Plan)
- All screens 1-6 fully implemented
- Low power camera mode (persistent, never closed)
- High power camera mode (only during liveness)
- Service startup with health checks
- Error handling (stops on fatal errors)
- WiFi auto-detection (skips Screen 3 if connected)
- Modular QR system (User QR + WiFi QR)
- Screen 7 as dummy placeholder (as requested)

### 📍 PLACEHOLDER
- Screen 7: Simple dummy implementation (3s auto-transition)
  - No complex logic
  - No anti-spoofing logic
  - No high power camera activation
  - Just displays a screen and moves to READY

### 🔮 FUTURE WORK
- Replace Screen 7 placeholder with actual implementation
- Add progress bar UI to Screen 2 (low priority)
- Extend QR system with additional types (if needed)

---

## 🔧 TECHNICAL NOTES

### Boot Flow
```
Power On
   ↓
Screen 1: Logo (3s)
   ↓
Screen 2: Booting (services starting)
   ├─ Service 1: Camera (LOW POWER) ✅
   ├─ Service 2: Display ✅
   └─ Check WiFi ✅
   ↓
   ├─ WiFi Connected? YES → Screen 4 (skip Screen 3)
   └─ WiFi Connected? NO → Screen 3 (scan WiFi QR)
   ↓
Screen 4: WiFi Connected (3s, show SSID)
   ↓
Screen 5: mDai Ready (2s)
   ↓
Screen 6: Idle (scan User QR)
   ↓
   [User scans QR from mobile app]
   ↓
WebSocket Connected
   ↓
Screen 7: [PLACEHOLDER] (3s auto-transition) ⚠️
   ↓
READY: Wait for mobile to send "start"
   ↓
   [Camera: LOW POWER → HIGH POWER]
   ↓
WARMUP → ALIGN → PROCESSING → SUCCESS/ERROR → LOGOUT → Back to IDLE
   ↓
   [Camera: HIGH POWER → LOW POWER]
```

### State Transitions
- All state changes go through `set_state()` for centralized management
- Each state has its own screen display + camera configuration
- Thread-safe atomic state management
- Timeout protections on all waiting states

---

## 📝 CODE REFERENCES

### Key Files
- **Header**: `/home/mercleDev/codebase/include/SystemController.hpp`
  - Line 19-35: SystemState enum with PLACEHOLDER_SCREEN_7
- **Implementation**: `/home/mercleDev/codebase/src/SystemController.cpp`
  - Line 848-899: `handle_boot()` - Screens 1-5
  - Line 115-223: `set_state()` - All screen transitions
  - Line 337-493: `handle_idle()` - Screen 6 + QR scanning
  - Line 225-299: `configure_camera_for_state()` - Power management

### Screen Mapping
| Screen | State | Serial ID | Description |
|--------|-------|-----------|-------------|
| 1 | BOOT | 1 | Logo (3s) |
| 2 | BOOT | 2 | Booting services |
| 3 | AWAIT_ADMIN_QR | 3 | Scan WiFi QR (conditional) |
| 4 | PROVISIONED | 4 | WiFi Connected (3s, SSID) |
| 5 | PROVISIONED | 5 | mDai Ready (2s) |
| 6 | IDLE | 6 | Scan User/WiFi QR |
| 7 | PLACEHOLDER_SCREEN_7 | 7 | **[PLACEHOLDER]** (3s) ⚠️ |
| 8 | WARMUP | 8 | Camera warmup |
| 9 | ALIGN | 9 | Nose tracking |
| 10 | PROCESSING | 10 | Processing |
| 11 | SUCCESS | 11 | Success |
| 12 | ERROR | 12 | Error |
| 13 | LOGOUT | 13 | Thank you |

---

## ✅ CHECKLIST (As Per Original Plan)

- [x] Screen 1: Logo (3s, preemptive)
- [x] Screen 2: Booting with service checks (NOT preemptive)
- [x] WiFi service starts and checks known connections
- [x] Last connected WiFi prioritized
- [x] Camera starts in LOW POWER mode
- [x] Screen check validation
- [x] Camera check validation
- [x] Display error text and stop if services fail
- [x] Screen 3: Only display if WiFi NOT connected
- [x] Skip Screen 3 if WiFi already connected
- [x] Stay on Screen 3 until WiFi connected
- [x] Screen 4: Show WiFi SSID for 3 seconds minimum
- [x] Screen 4: Directly displayed if WiFi was connected on boot
- [x] Screen 5: mDai Ready (2s preemptive)
- [x] Screen 6: Idle phase with dual QR types (User + WiFi)
- [x] Modular QR system (can accommodate additions)
- [x] User QR with session token for WebSocket
- [x] Mobile app generates QR from JWT
- [x] RDK scans QR and connects to WebSocket
- [x] Ready state signal sent to mobile and RDK
- [x] Screen 7: Placeholder/dummy implementation ⚠️
- [x] Low power vs high power camera differentiation
- [x] Low power camera is persistent (never closed)
- [x] High power camera only for liveness verification
- [ ] Screen 7: Full implementation (deferred as requested)
- [ ] Anti-spoofing logic for Screen 7 (deferred as requested)
- [ ] Progress bar UI for Screen 2 (low priority, optional)

---

## 🎉 RESULT

**Status**: ✅ **SCREENS 1-6 FULLY COMPLETE** | ⚠️ **SCREEN 7 IS PLACEHOLDER**

All requirements from the plan have been implemented for Screens 1-6. Screen 7 is a simple dummy placeholder as requested, ready to be replaced with actual implementation later.

The system is production-ready for the boot sequence and QR scanning flow. Screen 7 placeholder can be expanded when requirements are finalized.

