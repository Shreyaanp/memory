# EC2 Server Message Protocol

## Overview
The EC2 middleware server handles bidirectional WebSocket communication between:
- **Mobile App** (user interface)
- **RDK Device** (face detection hardware)

---

## Mobile App → EC2 Server

### 1. Authentication Message
```json
{
  "type": "auth",
  "bearer_token": "eyJhbGc...",  // ws_token from QR code
  "session_id": "6922e054..."
}
```
**Response:**
```json
{
  "type": "auth_success",
  "session_id": "6922e054...",
  "message": "Mobile connected successfully"
}
```

### 2. Send Message to Device
```json
{
  "type": "to_device",
  "data": {
    // Any custom data to send to device
    "action": "start_capture",
    "params": {...}
  }
}
```
**Response:** Data is forwarded to device

### 3. Ping (Keepalive)
```json
{
  "type": "ping"
}
```
**Response:**
```json
{
  "type": "pong"
}
```

---

## RDK Device → EC2 Server

### 1. Authentication Message
```json
{
  "type": "auth",
  "bearer_token": "eyJhbGc...",  // ws_token from QR code
  "device_id": "TEST_RDK_b4_2f_03_31_9a_35",
  "session_id": "6922e054..."
}
```
**Response:**
```json
{
  "type": "auth_success",
  "session_id": "6922e054...",
  "device_id": "TEST_RDK_b4_2f_03_31_9a_35",
  "message": "Device connected successfully"
}
```

### 2. Send Message to Mobile
```json
{
  "type": "to_mobile",
  "data": {
    // Any custom data to send to mobile
    "status": "ready",
    "info": {...}
  }
}
```
**Response:** Data is forwarded to mobile

### 3. Submit Verification Result
```json
{
  "type": "submit_result",
  "image": "base64_encoded_image...",
  "consecutive_passes": 5,
  "confidence": 0.95,
  "liveness_score": 0.92,
  "face_match_score": 0.88,
  "anti_spoof_score": 0.90
}
```
**Server Actions:**
- Checks if `consecutive_passes >= 5`
- If YES: Forwards image + platform_id to backend API
- If NO: Sends failure to mobile
- Closes both WebSocket connections

**Response to Mobile:**
```json
{
  "type": "verification_result",
  "status": "success" | "failed",
  "reason": "..."  // if failed
}
```

### 4. Ping (Keepalive)
```json
{
  "type": "ping"
}
```
**Response:**
```json
{
  "type": "pong"
}
```

---

## EC2 Server → Mobile App

### 1. Device Connected Notification
```json
{
  "type": "device_connected",
  "device_id": "TEST_RDK_b4_2f_03_31_9a_35",
  "message": "Device connected and ready"
}
```

### 2. Verification Result (Final)
```json
{
  "type": "verification_result",
  "status": "success",
  "message": "Verification successful"
}
```
OR
```json
{
  "type": "verification_result",
  "status": "failed",
  "reason": "Insufficient liveness frames"
}
```

### 3. Custom Messages from Device
```json
{
  "type": "to_mobile",
  "data": {
    // Whatever device sent via to_mobile
  }
}
```

### 5. Error Messages
```json
{
  "type": "error",
  "error": "Device not connected"
}
```

---

## EC2 Server → RDK Device

### 1. Custom Messages from Mobile
```json
{
  "type": "to_device",
  "data": {
    // Whatever mobile sent via to_device
  }
}
```

### 2. Verification Result Confirmation
```json
{
  "type": "verification_result",
  "status": "success" | "failed",
  "reason": "..."
}
```

### 3. Error Messages
```json
{
  "type": "error",
  "error": "Mobile not connected"
}
```

---

## Backend API Integration

### EC2 → Backend (After Successful Verification)

**Endpoint:** `https://newapi.mercle.ai/api/admin/hardware/next`

**Request:**
```json
{
  "platform_id": "PLT_j5LdgA3OB5R5",
  "image_base64": "base64_encoded_image..."
}
```

**Headers:**
```
x-api-key: mercle
```

**Response:** Backend processes and stores the result

---

## Connection Flow

```
1. Mobile App
   ↓
2. GET /api/mobile/create-session
   ← Returns: session_id, ws_token, qr_encrypted
   ↓
3. Mobile connects to WebSocket
   ← Sends: auth with ws_token
   ← Receives: auth_success
   ↓
4. Mobile displays QR code
   ↓
5. RDK Device scans QR
   ← Decrypts: session_id, ws_token
   ↓
6. Device connects to WebSocket
   ← Sends: auth with ws_token + device_id
   ← Receives: auth_success
   ↓
7. Mobile receives: device_connected
   ↓
8. Device performs liveness detection
   ↓
9. Device sends: submit_result
   ↓
10. EC2 Server validates consecutive_passes
    ↓
11. If valid:
    ← EC2 → Backend API (with platform_id + image)
    ← EC2 → Mobile: verification_result (success)
    ← EC2 → Device: verification_result (success)
    
    If invalid:
    ← EC2 → Mobile: verification_result (failed)
    ← EC2 → Device: verification_result (failed)
    ↓
12. Both WebSocket connections close
```

---

## Key Security Features

1. **Token Validation:**
   - Mobile must provide valid backend user token
   - EC2 generates short-lived ws_token (3 min expiry)
   - Both mobile and device authenticate with ws_token

2. **Session Management:**
   - Each session has unique ID
   - Sessions expire after 3 minutes
   - Multi-scan detection (same device, different sessions)

3. **Privacy:**
   - Device NEVER knows platform_id
   - EC2 stores platform_id from backend validation
   - EC2 injects platform_id when forwarding to backend

4. **Encrypted QR:**
   - QR payload encrypted with AES-256-CBC
   - Contains: session_id, ws_token, timestamp

---

## Message Types Summary

| Source | Message Type | Target | Purpose |
|--------|-------------|--------|---------|
| Mobile | `auth` | EC2 | Authenticate WebSocket |
| Mobile | `to_device` | Device | Send custom data to device |
| Mobile | `ping` | EC2 | Keepalive |
| Device | `auth` | EC2 | Authenticate WebSocket |
| Device | `to_mobile` | Mobile | Send custom data to mobile |
| Device | `submit_result` | EC2 | Submit verification result |
| Device | `ping` | EC2 | Keepalive |
| EC2 | `auth_success` | Mobile/Device | Confirm authentication |
| EC2 | `device_connected` | Mobile | Notify device is ready |
| EC2 | `verification_result` | Mobile/Device | Final verification outcome |
| EC2 | `error` | Mobile/Device | Error notifications |
| EC2 | `pong` | Mobile/Device | Keepalive response |

---

## Current Implementation Status

✅ **Implemented:**
- Session creation with backend validation
- QR encryption/decryption
- WebSocket authentication (mobile + device)
- Device-mobile pairing notification
- Result submission with consecutive_passes check
- Backend API integration
- Multi-scan detection
- Session expiry
- Error handling

✅ **Tested:**
- Complete end-to-end flow
- Mobile simulation
- RDK device simulation
- All message types
- Success and failure paths

