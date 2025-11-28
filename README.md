# MDAI System - Face Verification & Liveness Detection

A production-ready face verification system with anti-spoofing capabilities for embedded RDK devices.

## Overview

MDAI (Mercle Device AI) performs secure face verification using:
- **Intel RealSense** depth camera (RGB + Depth + IR)
- **MediaPipe Face Mesh** for 468-point face tracking
- **Multi-modal anti-spoofing** (depth, IR, temporal analysis)
- **LilyGo T-Display** ESP32 for user interface
- **WebSocket** communication with EC2 backend

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    RDK DEVICE                               │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────┐     ┌─────────────────┐                  │
│  │ LilyGo ESP32 │◄────│ Intel RealSense │                  │
│  │   Display    │UART │  Depth Camera   │                  │
│  └──────────────┘     └─────────────────┘                  │
│         │                     │                             │
│         └──────────┬──────────┘                             │
│                    ▼                                        │
│           ┌────────────────┐                               │
│           │  mdai_system   │ ◄── C++ Application           │
│           │  (Main App)    │                               │
│           └───────┬────────┘                               │
│                   │ WiFi                                    │
└───────────────────┼─────────────────────────────────────────┘
                    │
                    ▼
           ┌────────────────┐
           │  EC2 Server    │ ◄── FastAPI + WebSocket
           │ mdai.mercle.ai │
           └────────────────┘
```

## Quick Start

### Build
```bash
cd /home/mercleDev/codebase
mkdir -p build && cd build
cmake .. && make -j$(nproc)
```

### Run
```bash
# Development (foreground)
./build-and-run.sh

# Production (service)
./mdai-service.sh start
```

### Check Production Readiness
```bash
./production_ready_check.sh
```

## Project Structure

```
codebase/
├── src/                    # C++ source files
│   ├── main.cpp           # Entry point
│   ├── SystemController.cpp # Main state machine
│   ├── FaceDetector.cpp   # MediaPipe face detection
│   ├── AntiSpoofing.cpp   # Spoof detection algorithms
│   ├── NetworkManager.cpp # WebSocket client
│   ├── SerialCommunicator.cpp # ESP32 UART
│   └── ...
├── include/               # Header files
├── lib/mediapipe/        # MediaPipe wrapper library
├── models/               # ML models (fallback)
├── tools/                # Utility scripts
│   ├── generate_device_label.py
│   ├── register_device.py
│   └── admin_portal*.html
├── deployment/           # Deployment scripts
└── build/               # Build output
```

## State Machine

```
BOOT → AWAIT_ADMIN_QR (if no WiFi)
  ↓
PROVISIONED → IDLE (Screen 6: Scan QR)
  ↓
READY (Screen 7) → COUNTDOWN → ALIGN (Screen 9)
  ↓
PROCESSING → SUCCESS/ERROR → LOGOUT → IDLE
```

## Key Scripts

| Script | Purpose |
|--------|---------|
| `build-and-run.sh` | Full build with checks, runs in foreground |
| `rebuild-and-restart.sh` | Quick rebuild and restart |
| `mdai-service.sh` | Service management (start/stop/status) |
| `production_ready_check.sh` | Verify system readiness |
| `start_mdai.sh` | Systemd exec script |

## Configuration

Device config stored at `/opt/mdai/device_config.json`:
```json
{
  "device_id": "dev_xxx",
  "hardware_id": "b4:2f:03:31:9a:35",
  "device_key": "..."
}
```

## Documentation

- [System Architecture](COMPLETE_SYSTEM_ARCHITECTURE.md) - Detailed architecture
- [Quick Reference](QUICK_REFERENCE.md) - Commands and API endpoints
- [EC2 Credentials](EC2_SSH_CREDENTIALS.md) - Server access

## Dependencies

- librealsense2
- OpenCV 4.x
- OpenSSL
- nlohmann/json
- zbar (QR scanning)
- MediaPipe (via Python bridge)
- abseil

## License

Proprietary - Mercle AI



