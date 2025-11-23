# MDAI System - Quick Reference Guide

## 🚀 System Status: OPERATIONAL

**Server:** http://mdai.mercle.ai (port 80 via nginx)  
**Version:** v1.0-beta  
**Last Updated:** 2025-11-21

---

## 📡 API Endpoints

### REST API
```bash
# Health check
curl http://mdai.mercle.ai/api/health

# Server status
curl http://mdai.mercle.ai/api/status

# Create mobile session (requires bearer token)
curl http://mdai.mercle.ai/api/mobile/create-session \
  -H "Authorization: Bearer YOUR_TOKEN_HERE"
```

### WebSocket
- **Mobile:** `ws://mdai.mercle.ai/ws/mobile`
- **Device:** `ws://mdai.mercle.ai/ws/device`

---

## 🔑 Key Files & Locations

### Local Development
```
/home/mercleDev/codebase/
├── ec2-server/
│   ├── server.py                 # FastAPI server
│   ├── requirements.txt          # Python dependencies
│   └── deploy-to-ec2.sh         # Deployment script
├── src/
│   ├── SystemController.cpp      # RDK main logic
│   └── TrustZoneIdentity.cpp    # Hardware security
├── include/
│   └── SystemController.hpp      # State machine definitions
├── UI_Application/
│   └── include/screens/         # LilyGo display screens
├── TODO_CHECKLIST.md            # Complete TODO list
└── QUICK_REFERENCE.md           # This file
```

### EC2 Production
```
/opt/mdai/
├── server.py                     # Running server
├── requirements.txt
├── venv/                         # Python virtual environment
└── .env (create this)           # Environment config

/var/lib/mdai/
└── mdai_server.db               # SQLite database

/etc/nginx/sites-available/
└── mdai                         # Nginx reverse proxy config

/etc/systemd/system/
└── mdai-server.service          # Systemd service
```

---

## 🎯 Complete Flow (Updated)

### Step-by-Step
1. **Mobile** creates session: `GET /api/mobile/create-session`
2. **Mobile** displays encrypted QR code
3. **RDK** scans QR → decrypts → connects to WebSocket
4. **RDK** sends `device_ready` → enters **READY state** (Screen 6 "Ready?")
5. **Mobile** receives `device_ready` notification
6. **User clicks "Start"** on mobile
7. **Mobile** sends: `{type: "to_device", data: {command: "start_verification"}}`
8. **RDK** receives command → **READY → WARMUP → ALIGN**
9. Face verification proceeds with real-time progress updates
10. Results sent to both mobile and device
11. Device returns to IDLE (Screen 5)

---

## 🛠️ Common Commands

### EC2 Server Management
```bash
# SSH to EC2
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai

# Check server status
sudo systemctl status mdai-server

# View logs
sudo journalctl -u mdai-server -f

# Restart server
sudo systemctl restart mdai-server

# Deploy updates
cd /home/mercleDev/codebase/ec2-server
scp -i ~/.ssh/mdaiws.pem server.py ubuntu@mdai.mercle.ai:/opt/mdai/
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai 'sudo systemctl restart mdai-server'
```

### RDK Build & Deploy
```bash
# Build
cd /home/mercleDev/codebase
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# Deploy (when ready)
scp RDK_Application root@<RDK_IP>:/usr/local/bin/
```

---

## 🎨 Screen Flow

```
Screen 1: Boot Logo
Screen 2: Initializing
Screen 3: WiFi Check (skip if connected)
Screen 4: Welcome (3s) - ❓ verify if used
Screen 5: Waiting for QR (IDLE state)
Screen 6: Ready? (READY state - waiting for user to start) ✨ NEW
Screen 7: Face Alignment (ALIGN state)
Screen 8: Success
Screen 9: Failed
Screen 10: Scan Again
Screen 11-13: WiFi Change Flow
```

---

## 🔐 Security Notes

### Current Setup
- ✅ Bearer token validation with backend
- ✅ AES-256 encrypted QR codes
- ✅ JWT for WebSocket authentication
- ✅ Platform ID privacy (never sent to device)
- ✅ Multi-scan detection
- ⚠️ HTTP only (no HTTPS yet) - **TODO**
- ⚠️ Simple JWT secret - **TODO: Change**
- ⚠️ Simple QR key - **TODO: Change**

### Ports Open on EC2
- Port 22: SSH
- Port 80: HTTP (nginx → FastAPI)
- Port 443: HTTPS (nginx, but no SSL certs yet)

---

## 📊 Critical TODOs (Do First)

1. ✅ **DONE:** Fix ws_url in server response
2. ⚠️ Add HTTPS/WSS support (get SSL certificates)
3. ⚠️ Change default JWT secret
4. ⚠️ Test full flow with real RDK device

---

## 🐛 Known Issues

### Critical
- No HTTPS (using HTTP only)
- JWT secret is predictable
- No session cleanup (will grow indefinitely)

### Important  
- Camera config hardcoded in multiple places
- No retry logic for backend API calls
- No graceful shutdown handling

### Nice to Have
- No progress indicator during WARMUP
- No admin dashboard
- No monitoring/metrics

---

## 📞 Backend Integration

### Your Backend URLs
```python
BACKEND_VALIDATE_URL = "https://newapi.mercle.ai/api/auth/verify-mdai-user"
BACKEND_API_URL = "https://newapi.mercle.ai/api/admin/hardware/next"
BACKEND_API_KEY = "mercle"
```

### Expected Responses
```json
// Token validation (GET with Authorization header)
{
  "success": true,
  "platform_id": "user_platform_id_here"
}

// Verification result (POST with platform_id as auth)
{
  "success": true,
  "confidence": 0.95
}
```

---

## 🧪 Testing

### Test Mobile Session Creation
```bash
curl http://mdai.mercle.ai/api/mobile/create-session \
  -H "Authorization: Bearer YOUR_TOKEN" \
  | jq
```

### Test WebSocket (with Python)
```python
import asyncio
import websockets
import json

async def test():
    uri = 'ws://mdai.mercle.ai/ws/mobile'
    async with websockets.connect(uri) as ws:
        await ws.send(json.dumps({
            'type': 'auth',
            'bearer_token': 'YOUR_WS_TOKEN'
        }))
        response = await ws.recv()
        print(json.loads(response))

asyncio.run(test())
```

---

## 📚 Documentation

- **Full TODO List:** `/home/mercleDev/codebase/TODO_CHECKLIST.md`
- **Server Code:** `/home/mercleDev/codebase/ec2-server/server.py`
- **RDK Controller:** `/home/mercleDev/codebase/src/SystemController.cpp`

---

## 🆘 Troubleshooting

### Server Not Responding
```bash
ssh ubuntu@mdai.mercle.ai
sudo systemctl status mdai-server
sudo journalctl -u mdai-server -n 50
```

### WebSocket Connection Failed
- Check if server is running
- Verify ws:// (not wss://) until SSL added
- Check bearer token is valid

### RDK Build Failed
- Ensure all dependencies installed
- Check CMakeLists.txt for required packages
- Verify camera SDK (RealSense) installed

---

**Quick Start:** Everything is deployed and working! Test with your mobile app, then deploy to RDK device. 🚀

