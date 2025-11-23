# MDAI Complete System Architecture

## 🏗️ Physical Hardware Setup

```
┌─────────────────────────────────────────────────────────────────────┐
│                         PHYSICAL DEVICE (RDK)                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌──────────────────┐        ┌───────────────────┐                 │
│  │  LilyGo T-Display │        │ Intel RealSense   │                 │
│  │  AMOLED (ESP32)   │◄──────►│ Depth Camera      │                 │
│  │  240x536 pixels   │  UART  │ D435/D455         │                 │
│  │  (Screen Display) │        │ (Face Detection)  │                 │
│  └──────────────────┘        └───────────────────┘                 │
│         │                             │                              │
│         │                             │                              │
│         │                             ▼                              │
│         │                    ┌─────────────────┐                    │
│         │                    │ Main RDK Board  │                    │
│         └───────────────────►│ (ARM/x86 Linux) │                    │
│                              │ - WiFi Module   │                    │
│                              │ - TPM Chip      │                    │
│                              │ - CPU/RAM       │                    │
│                              └─────────────────┘                    │
│                                      │                               │
└──────────────────────────────────────┼───────────────────────────────┘
                                       │ WiFi/Network
                                       │
                              ┌────────▼────────┐
                              │   Internet      │
                              └────────┬────────┘
                                       │
                          ┌────────────┴────────────┐
                          │                         │
                    ┌─────▼──────┐          ┌──────▼──────┐
                    │ EC2 Server │          │ Mobile App  │
                    │            │          │ (User Phone)│
                    └────────────┘          └─────────────┘
```

## 📡 Software Components Running WHERE

### 1️⃣ **ESP32 (LilyGo Display) - `/dev/ttyUSB0`**
**Location:** Connected via USB to RDK main board
**Runs:** PlatformIO firmware (Arduino/ESP-IDF)
**Code:** `/home/mercleDev/codebase/UI_Application/`

**Responsibilities:**
- Display UI screens (1-13)
- Receive state commands via UART
- Show progress bars, face alignment guide
- Display WiFi setup screens
- Boot logo and animations

**Communication:**
```
RDK Board ──► [UART /dev/ttyUSB0] ──► ESP32
Commands: state changes (1-13), progress updates (0-100)
```

**LVGL Screens:**
```
Screen 1:  Boot Logo
Screen 2:  Initializing
Screen 3:  WiFi QR Scan
Screen 4:  Welcome Message (3s)
Screen 5:  Waiting for User QR (IDLE)
Screen 6:  Ready? (User must click Start)
Screen 7:  Face Alignment Guide
Screen 8:  Success
Screen 9:  Failed
Screen 10: Scan Again
Screen 11-13: WiFi Change Flow
```

---

### 2️⃣ **RDK Main Board (Linux ARM/x86)**
**Location:** Main compute unit with WiFi
**Runs:** C++ application (`mdai_system`)
**Code:** `/home/mercleDev/codebase/src/` + `/home/mercleDev/codebase/include/`

**Binary Location:** `/home/mercleDev/codebase/build/mdai_system`

**Responsibilities:**
- Camera management (RealSense)
- Face detection and liveness
- QR code scanning (both WiFi and Session)
- WebSocket client to EC2
- State machine controller
- TPM identity management
- UART communication with ESP32

**Processes Running:**
```bash
./mdai_system              # Main application (runs continuously)
├── Camera Thread          # RealSense frame capture
├── Processing Thread      # Face detection/liveness
├── Network Thread         # WebSocket client
└── Serial Thread          # UART to ESP32
```

**Device Configuration:**
```
/opt/mdai/device_config.json:
{
  "device_id": "dev_xyz...",
  "hardware_id": "b4:2f:03:31:9a:35",
  "device_key": "abc123...",
  "tpm_wrapped_key": "..."
}
```

**State Machine:**
```
BOOT → AWAIT_ADMIN_QR (WiFi setup)
    ↓
PROVISIONING (connecting to WiFi)
    ↓
PROVISIONED (3s confirmation)
    ↓
IDLE (Screen 5 - waiting for user QR)
    ↓
READY (Screen 6 - device connected, waiting for "Start")
    ↓
WARMUP (camera warmup)
    ↓
ALIGN (face alignment)
    ↓
VERIFY (liveness check)
    ↓
SUCCESS/FAILED (Screen 8/9)
    ↓
IDLE (back to Screen 5)
```

---

### 3️⃣ **EC2 Server (AWS Cloud)**
**Location:** `mdai.mercle.ai` (public IP)
**Runs:** Python FastAPI server
**Code:** `/opt/mdai/server.py` (on EC2)

**Deployed From:** `/home/mercleDev/codebase/ec2-server/server.py`

**Processes Running:**
```bash
sudo systemctl status mdai-server

# Shows:
/opt/mdai/venv/bin/python3 /opt/mdai/server.py
├── Uvicorn ASGI Server (port 8000)
│   ├── WebSocket /ws/mobile (for phones)
│   ├── WebSocket /ws/device (for RDK)
│   └── REST API endpoints
└── Nginx reverse proxy (port 80)
    └── http://mdai.mercle.ai → localhost:8000
```

**Database:**
```
/var/lib/mdai/mdai_server.db (SQLite)
Tables:
- mobile_sessions
- device_connections
- paired_sessions
```

**Responsibilities:**
- Bearer token validation (calls your backend)
- Generate encrypted QR codes (AES-256)
- WebSocket message routing (mobile ↔ device)
- Session management (3 minute expiry)
- Platform ID privacy (never sent to device)
- Multi-scan detection

**REST API Endpoints:**
```
GET  /api/health
GET  /api/status
GET  /api/mobile/create-session
     → Header: Authorization: Bearer <token>
     → Returns: session_id, ws_token, qr_encrypted, ws_url
```

**WebSocket Channels:**
```
ws://mdai.mercle.ai/ws/mobile
  ↓ Mobile app connects with ws_token
  
ws://mdai.mercle.ai/ws/device  
  ↓ RDK connects with decrypted session_id + jwt_token
  
Server pairs them and routes messages bidirectionally
```

---

### 4️⃣ **Mobile App (User's Phone)**
**Location:** User's smartphone
**Platform:** React Native / Flutter / Native iOS/Android
**Code:** Your mobile app (not in this codebase)

**Responsibilities:**
1. Get bearer token from your auth system
2. Call `GET /api/mobile/create-session` with bearer token
3. Display encrypted QR code
4. Connect to WebSocket: `ws://mdai.mercle.ai/ws/mobile`
5. Send auth: `{type: "auth", bearer_token: "..."}`
6. Wait for device ready notification
7. User clicks "Start"
8. Send: `{type: "to_device", data: {command: "start_verification"}}`
9. Receive real-time progress updates
10. Display final result (success/fail)

---

### 5️⃣ **Admin Portal (Optional Testing Tool)**
**Location:** Local HTML file
**Files:** 
- `/home/mercleDev/codebase/tools/admin_portal.html` (WiFi setup QR)
- `/home/mercleDev/codebase/ec2-server/mobile_session_portal.html` (Session QR)

**Purpose:** Testing/debugging tool for admins
**Opens in:** Web browser (Chrome/Firefox)

---

## 🔄 Complete Flow - Step by Step

### Phase 1: Initial Device Setup (One-time)

```
1. Admin runs device registration:
   cd /home/mercleDev/codebase/deployment
   ./deploy-complete-system.sh
   
   → Generates device keys with TPM
   → Registers device with EC2 backend
   → Creates /opt/mdai/device_config.json
   → Creates physical label with device_key QR

2. Admin uses admin_portal.html to generate WiFi QR
   → Device scans QR → decrypts with device_key
   → Connects to WiFi
   → Device enters IDLE state (Screen 5)
```

### Phase 2: User Authentication Flow (Every Use)

```
┌─────────────┐                 ┌──────────────┐                 ┌─────────────┐
│ Mobile App  │                 │  EC2 Server  │                 │  RDK Device │
└──────┬──────┘                 └──────┬───────┘                 └──────┬──────┘
       │                               │                                │
       │ 1. POST /api/mobile/          │                                │
       │    create-session             │                                │
       │    Bearer: user_token         │                                │
       ├──────────────────────────────►│                                │
       │                               │ 2. Validate token with         │
       │                               │    newapi.mercle.ai            │
       │                               │    (get platform_id)           │
       │                               │                                │
       │ 3. Returns:                   │                                │
       │    - session_id               │                                │
       │    - qr_encrypted             │                                │
       │    - ws_token                 │                                │
       │◄──────────────────────────────┤                                │
       │                               │                                │
       │ 4. Display encrypted QR       │                                │
       │                               │                                │
       │                               │                5. User shows QR│
       │                               │                   to camera    │
       │                               │                                │
       │                               │                6. Scan QR      │
       │                               │                   Decrypt with │
       │                               │                   device_key   │
       │                               │                                │
       │                               │ 7. Connect WebSocket           │
       │                               │    ws://mdai.../ws/device      │
       │                               │◄───────────────────────────────┤
       │                               │                                │
       │                               │ 8. Validate JWT                │
       │                               │    Check session exists        │
       │                               │                                │
       │                               │ 9. Device authenticated        │
       │                               ├───────────────────────────────►│
       │                               │    {success: true}             │
       │                               │                                │
       │ 10. Connect WebSocket         │                                │
       │     ws://mdai.../ws/mobile    │                                │
       ├──────────────────────────────►│                                │
       │                               │                                │
       │ 11. Auth with ws_token        │                                │
       ├──────────────────────────────►│                                │
       │                               │                                │
       │                               │ 12. Pair mobile+device         │
       │                               │     Store platform_id          │
       │                               │     (never sent to device!)    │
       │                               │                                │
       │                               │ 13. Send device_ready          │
       │                               ├───────────────────────────────►│
       │◄──────────────────────────────┤ (Device shows Screen 6)        │
       │ 14. device_ready notification │                                │
       │                               │                                │
       │ 15. User clicks "Start"       │                                │
       │     on mobile                 │                                │
       │                               │                                │
       │ 16. Send start_verification   │                                │
       ├──────────────────────────────►│                                │
       │                               │ 17. Route to device            │
       │                               ├───────────────────────────────►│
       │                               │    {command: start_verification}
       │                               │                                │
       │                               │ 18. READY → WARMUP → ALIGN    │
       │                               │                (Screen 7)      │
       │                               │                                │
       │                               │ 19. Progress updates           │
       │◄──────────────────────────────┼────────────────────────────────┤
       │    {type: progress, value: X} │                                │
       │                               │                                │
       │                               │ 20. Face verified              │
       │                               │                                │
       │                               │ 21. POST result to backend     │
       │                               │     with platform_id           │
       │                               │     (device doesn't know it!)  │
       │                               │                                │
       │ 22. Success/Fail to mobile    │                                │
       │◄──────────────────────────────┤                                │
       │                               │ 23. Success/Fail to device     │
       │                               ├───────────────────────────────►│
       │                               │                (Screen 8/9)    │
       │                               │                                │
       │ 24. Device returns to IDLE    │                                │
       │                               │                (Screen 5)      │
       │                               │                                │
```

---

## 🔌 Communication Protocols

### UART Communication (RDK ↔ ESP32)
```
Protocol: Custom binary protocol
Baud: 115200
Direction: Mostly RDK → ESP32 (commands)

Commands:
- STATE:<1-13>     → Change screen
- PROGRESS:<0-100> → Update progress bar
```

### WebSocket Protocol (Device ↔ Server)
```json
// Device → Server (authentication)
{
  "type": "auth",
  "session_id": "sess_xyz",
  "jwt_token": "eyJ..."
}

// Device → Server (status updates)
{
  "type": "to_mobile",
  "data": {
    "state": "ALIGN",
    "progress": 45
  }
}

// Server → Device (commands)
{
  "type": "to_device", 
  "data": {
    "command": "start_verification"
  }
}
```

### WebSocket Protocol (Mobile ↔ Server)
```json
// Mobile → Server (authentication)
{
  "type": "auth",
  "bearer_token": "abc123..."
}

// Mobile → Server (commands)
{
  "type": "to_device",
  "data": {
    "command": "start_verification"
  }
}

// Server → Mobile (notifications)
{
  "type": "device_ready",
  "session_id": "sess_xyz"
}
```

---

## 🗂️ File Locations Summary

### On Development Machine (`/home/mercleDev/`)
```
codebase/
├── src/                          # C++ RDK application
│   ├── SystemController.cpp      # Main state machine
│   └── NetworkManager.cpp        # WebSocket client
├── include/                      # Headers
├── UI_Application/               # ESP32 PlatformIO project
│   └── src/main.cpp              # ESP32 firmware
├── ec2-server/
│   ├── server.py                 # FastAPI server (deploy to EC2)
│   └── mobile_session_portal.html # Testing portal
├── tools/
│   ├── admin_portal.html         # WiFi QR generator
│   └── generate_device_label.py  # Device provisioning
├── deployment/
│   └── deploy-complete-system.sh # One-time setup script
└── build/
    └── mdai_system               # Compiled RDK binary
```

### On RDK Device (Embedded Linux)
```
/opt/mdai/
├── mdai_system                   # Main application
├── device_config.json            # Device credentials
└── models/                       # AI models
    └── face_landmarker.task

/dev/ttyUSB0                      # Serial connection to ESP32
```

### On EC2 Server (`mdai.mercle.ai`)
```
/opt/mdai/
├── server.py                     # FastAPI application
├── venv/                         # Python virtual environment
└── requirements.txt

/var/lib/mdai/
└── mdai_server.db               # SQLite database

/etc/systemd/system/
└── mdai-server.service          # System service

/etc/nginx/sites-available/
└── mdai                         # Nginx config (port 80 → 8000)
```

---

## 🎯 Answer to Your Questions

### Q1: "Can we create QR from phone flow too?"

**YES! Here's what exists:**

#### Current HTML Portal (for testing):
- `mobile_session_portal.html` - Web-based QR generator
- User pastes bearer token → generates QR
- Good for testing, but not for production

#### For Production Mobile App:

**Option A: Server-Side QR Generation (RECOMMENDED)**
```javascript
// Mobile app calls API
const response = await fetch('http://mdai.mercle.ai/api/mobile/create-session', {
  headers: { 'Authorization': `Bearer ${userToken}` }
});

const { qr_encrypted, ws_token, session_id } = await response.json();

// Display qr_encrypted as QR code using any QR library
import QRCode from 'react-native-qrcode-svg';
<QRCode value={qr_encrypted} size={300} />
```

**Option B: Client-Side QR Generation**
Mobile app could generate QR locally IF you share the AES encryption key
- ⚠️ **NOT RECOMMENDED** - encryption key would be in mobile app code
- Server-side is more secure

#### Recommendation:
Your mobile app should:
1. Call `/api/mobile/create-session` API
2. Get back `qr_encrypted` string
3. Use any React Native QR library to display it
4. No need for HTML portal in production

The HTML portal is just for **admin testing** - your real mobile app will do this natively!

---

## 🚀 What's Running Where - Quick Summary

| Component | Location | What Runs | Communication |
|-----------|----------|-----------|---------------|
| **ESP32 Display** | Connected to RDK via USB | PlatformIO firmware (C++) | ← UART from RDK |
| **RDK Main Board** | Standalone device with WiFi | `mdai_system` (C++) | WiFi → EC2, USB → ESP32, Camera |
| **EC2 Server** | AWS Cloud `mdai.mercle.ai` | FastAPI Python + Nginx | Internet (HTTP/WS) |
| **Mobile App** | User's phone | Your React Native app | WiFi/4G → EC2 |
| **Your Backend** | `newapi.mercle.ai` | Your existing API | ← EC2 validates tokens |

---

## 🔐 Security Notes

### Current Security:
✅ AES-256 encrypted QR codes
✅ Device-specific encryption keys
✅ JWT WebSocket authentication
✅ Bearer token validation with your backend
✅ Platform ID never sent to device
✅ TPM-based device identity

### TODO (Production):
⚠️ Add HTTPS/WSS (currently HTTP/WS)
⚠️ Change default JWT secret
⚠️ Add rate limiting
⚠️ Add session cleanup
⚠️ Add device certificate pinning

---

## 📞 Quick Commands

### Build RDK Application:
```bash
cd /home/mercleDev/codebase/build
cmake .. && make -j$(nproc)
```

### Deploy to EC2:
```bash
cd /home/mercleDev/codebase/ec2-server
./deploy-to-ec2.sh
```

### Check Server Status:
```bash
ssh ubuntu@mdai.mercle.ai
sudo systemctl status mdai-server
sudo journalctl -u mdai-server -f
```

### Flash ESP32 Display:
```bash
cd /home/mercleDev/codebase/UI_Application
pio run -t upload
```

---

**Everything is connected and working! 🎉**

The RDK device has a camera, a display, runs Linux, connects to WiFi, 
talks to your EC2 server via WebSocket, and coordinates with the mobile app 
to perform secure face verification. The platform_id stays private on the server!


