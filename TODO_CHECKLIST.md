# MDAI System - Complete TODO & Cleanup Checklist

## ✅ COMPLETED (Done)
- [x] EC2 Server converted to FastAPI
- [x] READY state added to RDK flow
- [x] Start verification command handler
- [x] Multi-scan detection implemented
- [x] Platform ID privacy (never sent to device)
- [x] Bearer token validation with backend
- [x] Encrypted QR codes (AES-256)
- [x] WebSocket bidirectional communication
- [x] EC2 deployment with nginx reverse proxy
- [x] Server running on mdai.mercle.ai

---

## 🔧 CODE CLEANUPS NEEDED

### 1. **Server.py Issues**
- [ ] Fix `ws_url` in session response (currently shows localhost:8000, should be mdai.mercle.ai)
  - Location: `/home/mercleDev/codebase/ec2-server/server.py` line ~310
  - Change: `"ws_url": "ws://mdai.mercle.ai/ws/mobile"`
  
- [ ] Remove old aiohttp backup file
  - File: `/home/mercleDev/codebase/ec2-server/server_aiohttp_backup.py`
  - Action: Delete it

- [ ] Clean up test files
  - File: `/home/mercleDev/codebase/ec2-server/test_server.py`
  - Action: Delete if not needed

- [ ] Update SERVER_PORT constant (currently hardcoded in startup message)
  - Make it read from environment or config

### 2. **EC2 Instance Cleanup**
- [ ] Remove old middleware files on EC2
  ```bash
  ssh ubuntu@mdai.mercle.ai
  rm ~/middleware_server*.py
  rm ~/sessions.db
  rm ~/http_server.log
  rm ~/middleware.log
  ```

- [ ] Remove old HTTP test server directory
  - Check: `/home/ubuntu/mdai-server/` vs `/opt/mdai/`
  - Keep only `/opt/mdai/`

### 3. **RDK C++ Code Issues**

#### a. **TrustZone Error Handling**
- [ ] Add better error messages in `TrustZoneIdentity.cpp`
  - Location: `/home/mercleDev/codebase/src/TrustZoneIdentity.cpp`
  - Add: Try-catch blocks and meaningful error messages
  - Add: Fallback if TrustZone not available

#### b. **Camera Configuration**
- [ ] Verify WARMUP needs RGB-only (currently set to RGB-only)
  - Location: `SystemController.cpp` line ~192
  - Question: Should WARMUP switch to RGB+IR+Depth early?

#### c. **WebSocket URL Hardcoded**
- [ ] Make middleware_host_ configurable
  - Currently: Hardcoded to "mdai.mercle.ai" somewhere
  - Should: Read from config file `/etc/mdai/device_config.json`

#### d. **QR Encryption Key Hardcoded**
- [ ] Move QR encryption key to config
  - Location: `SystemController.cpp` line ~270
  - Currently: `"mdai_qr_encryption_key_32byte!"`
  - Should: Match server's key from config

### 4. **UI Application (LilyGo Display)**

#### Screen Updates Needed:
- [ ] Verify Screen 6 text matches READY state intent
  - Current: "Ready?"
  - Consider: "Ready - Waiting to start" or "Connected"

- [ ] Check if we need a WARMUP screen
  - Screen 5 was supposed to be warmup
  - But now READY uses Screen 6
  - Clarify screen flow

#### Screen Flow Verification:
```
Screen 1: Boot logo ✅
Screen 2: Initializing ✅
Screen 3: WiFi check (skip if connected) ✅
Screen 4: Welcome (3 seconds) - IS THIS USED? ❓
Screen 5: Waiting for QR (IDLE state) ✅
Screen 6: Ready? (READY state - waiting for start) ✅
Screen 7: Face alignment (ALIGN state) ✅
Screen 8: Success ✅
Screen 9: Failed ✅
Screen 10: "Scan again" ✅
Screen 11-13: WiFi change flow ✅
```

- [ ] Clarify if Screen 4 is used or should be removed
- [ ] Add visual indicator for Screen 6 (Ready state waiting)

### 5. **Configuration Files Missing**

#### Device Config Template:
- [ ] Create `/etc/mdai/device_config.json.template`
  ```json
  {
    "device_id": "",
    "hardware_id": "",
    "middleware_host": "mdai.mercle.ai",
    "middleware_port": 443,
    "middleware_path": "/ws/device",
    "qr_encryption_key": "mdai_qr_encryption_key_32byte!",
    "camera": {
      "serial_number": "",
      "rgb_only_width": 640,
      "rgb_only_height": 480,
      "rgb_only_fps": 30,
      "full_width": 640,
      "full_height": 480,
      "full_fps": 30
    }
  }
  ```

#### Server Config:
- [ ] Create environment variable file for EC2 server
  - File: `/opt/mdai/.env`
  ```
  SERVER_HOST=0.0.0.0
  SERVER_PORT=8000
  DB_PATH=/var/lib/mdai/mdai_server.db
  QR_ENCRYPTION_KEY=mdai_qr_encryption_key_32byte!
  JWT_SECRET=your_jwt_secret_here_change_in_production
  BACKEND_VALIDATE_URL=https://newapi.mercle.ai/api/auth/verify-mdai-user
  BACKEND_API_URL=https://newapi.mercle.ai/api/admin/hardware/next
  BACKEND_API_KEY=mercle
  ```

---

## 🧪 TESTING NEEDED

### 1. **Unit Tests Missing**
- [ ] Test QR encryption/decryption matches between server and RDK
- [ ] Test JWT token generation and validation
- [ ] Test multi-scan detection logic
- [ ] Test session expiry (3 minutes)
- [ ] Test timeout scenarios (READY state 60s timeout)

### 2. **Integration Tests Needed**
- [ ] Full flow: Mobile → QR → RDK → WebSocket → Verification
- [ ] Test WiFi provisioning QR flow
- [ ] Test WiFi change flow (while already provisioned)
- [ ] Test connection loss scenarios
- [ ] Test multiple sessions simultaneously
- [ ] Test RDK reconnection after crash

### 3. **Security Tests**
- [ ] Verify platform_id never exposed to RDK
- [ ] Test bearer token validation with invalid tokens
- [ ] Test QR tampering detection
- [ ] Test WebSocket authentication failures
- [ ] Test SQL injection (should be safe with parameterized queries)

### 4. **Performance Tests**
- [ ] Test with 10+ simultaneous sessions
- [ ] Monitor memory usage during long runs
- [ ] Check WebSocket connection stability over hours
- [ ] Test camera stream performance (RGB vs RGB+IR+Depth)

---

## 📝 DOCUMENTATION NEEDED

### 1. **API Documentation for Mobile Team**
- [ ] Document `/api/mobile/create-session` endpoint
  - Headers required
  - Response format
  - Error codes
  
- [ ] Document WebSocket protocol
  - Connection URL
  - Authentication message format
  - All message types (device_ready, to_device, progress, etc.)
  - Error handling

- [ ] Create Postman/Insomnia collection for testing

### 2. **RDK Deployment Guide**
- [ ] Document device registration process
  - Run `register_device_to_ec2.sh`
  - Verify device_config.json created
  
- [ ] Document build process
  - CMake flags needed
  - Dependencies (MediaPipe, OpenCV, RealSense, etc.)
  
- [ ] Document deployment steps
  - Copy binary to RDK
  - Set up systemd service
  - Configure camera permissions

### 3. **Admin Portal Documentation**
- [ ] How to generate WiFi QR codes
- [ ] How to monitor active sessions
- [ ] How to view logs
- [ ] How to handle multi-scan events

### 4. **Troubleshooting Guide**
- [ ] Common issues and solutions
  - "Device not registered" error
  - WebSocket connection failures
  - Camera not found
  - TrustZone unavailable
  - QR decryption failures

---

## 🔐 SECURITY HARDENING

### 1. **Server Security**
- [ ] Change default JWT secret (currently predictable)
- [ ] Change QR encryption key (currently simple string)
- [ ] Add rate limiting to API endpoints
- [ ] Add CORS configuration (currently open)
- [ ] Enable HTTPS (SSL certificates) instead of HTTP
- [ ] Add API key authentication for device registration

### 2. **Database Security**
- [ ] Current SQLite is fine for small scale
- [ ] Consider PostgreSQL for production
- [ ] Add database backups
- [ ] Add session cleanup cron job (delete old sessions)

### 3. **Network Security**
- [ ] Configure firewall rules properly
- [ ] Use WSS (WebSocket Secure) instead of WS
- [ ] Add request size limits
- [ ] Add connection throttling

---

## 🏗️ ARCHITECTURE IMPROVEMENTS

### 1. **Error Recovery**
- [ ] Add automatic reconnection logic in RDK
  - If WebSocket drops, retry connection
  - If camera fails, attempt restart
  
- [ ] Add graceful degradation
  - If TrustZone unavailable, use software keys with warning
  - If MediaPipe fails, use simpler face detection

### 2. **Logging Improvements**
- [ ] Structured logging (JSON format)
- [ ] Log rotation configuration
- [ ] Separate log levels (DEBUG, INFO, ERROR)
- [ ] Add correlation IDs for tracking requests

### 3. **Monitoring**
- [ ] Add Prometheus metrics endpoint
- [ ] Add health check with detailed status
- [ ] Add alerting for critical errors
- [ ] Add session duration tracking

### 4. **Database Schema Improvements**
Current schema works but could add:
- [ ] Index on `session_id` for faster lookups
- [ ] Index on `expires_at` for cleanup queries
- [ ] Add `created_at` timestamp to all tables
- [ ] Add `last_activity` to track stale sessions

---

## 🐛 KNOWN ISSUES TO FIX

### 1. **Critical**
- [ ] `ws_url` in session response shows `localhost:8000` instead of `mdai.mercle.ai`
- [ ] No SSL/TLS (using HTTP instead of HTTPS)
- [ ] JWT secret is hardcoded and simple

### 2. **Important**
- [ ] No cleanup of expired sessions (will grow indefinitely)
- [ ] No graceful shutdown handling
- [ ] Camera config hardcoded in multiple places
- [ ] No retry logic for backend API calls

### 3. **Nice to Have**
- [ ] No progress indicator during WARMUP
- [ ] No battery level monitoring on RDK
- [ ] No network quality indicator
- [ ] No admin dashboard UI

---

## 📦 BUILD & DEPLOYMENT

### 1. **RDK Build**
- [ ] Test CMake build on clean system
- [ ] Create build script `build.sh`
- [ ] Document all dependencies
- [ ] Create Docker container for cross-compilation (optional)

### 2. **EC2 Deployment**
- [ ] Create deployment script for updates
- [ ] Add rollback capability
- [ ] Add blue-green deployment (optional)
- [ ] Set up CI/CD pipeline (optional)

### 3. **Packaging**
- [ ] Create RDK .deb package
- [ ] Create installation script
- [ ] Version numbering scheme
- [ ] Release notes template

---

## 🎨 POLISH & UX

### 1. **RDK User Experience**
- [ ] Add sound feedback (beeps) on QR scan
- [ ] Add LED indicators for status
- [ ] Improve error messages on display
- [ ] Add progress animations

### 2. **Mobile App UX**
- [ ] Clear error messages
- [ ] Retry button on failures
- [ ] Session timeout warning
- [ ] Better progress visualization

### 3. **Admin Portal**
- [ ] Real-time session monitoring
- [ ] Device status dashboard
- [ ] Multi-scan event log
- [ ] QR code generator UI

---

## 🔄 REFACTORING OPPORTUNITIES

### 1. **Code Organization**
- [ ] Split `SystemController.cpp` (850+ lines)
  - Extract QR handling
  - Extract WebSocket logic
  - Extract camera management
  
- [ ] Split `server.py` (711 lines)
  - Separate routes
  - Separate WebSocket handlers
  - Separate database logic

### 2. **Reduce Code Duplication**
- [ ] WiFi QR and Session QR have similar decrypt logic
- [ ] Multiple places configure camera
- [ ] Error handling repeated in many places

### 3. **Type Safety**
- [ ] Add TypeScript for mobile app
- [ ] Use Pydantic models consistently in server
- [ ] Add more C++ const correctness

---

## 📊 PRIORITY LEVELS

### 🔴 **CRITICAL (Do Now)**
1. Fix `ws_url` in server response
2. Add HTTPS/WSS support
3. Change default JWT secret
4. Test full end-to-end flow with real devices

### 🟡 **IMPORTANT (Do Soon)**
1. Add session cleanup (cron job)
2. Create deployment documentation
3. Add error recovery logic
4. Create config file templates

### 🟢 **NICE TO HAVE (Do Later)**
1. Add monitoring and metrics
2. Create admin dashboard
3. Add sound/LED feedback
4. Performance optimizations

---

## ✅ VERIFICATION CHECKLIST

Before considering the system "production ready":

- [ ] All critical issues fixed
- [ ] Full flow tested with physical devices
- [ ] Documentation complete
- [ ] Security hardening done
- [ ] Monitoring in place
- [ ] Backup strategy defined
- [ ] Disaster recovery plan
- [ ] User training completed

---

## 📝 NOTES

- Keep this file updated as items are completed
- Add new issues as they're discovered
- Prioritize based on user needs and risk
- Review monthly for new technical debt

---

**Last Updated:** 2025-11-21
**System Version:** v1.0-beta
**Status:** Functional, needs hardening for production

