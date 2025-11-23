# QR Code Encryption Analysis - MDAI System

## 🔍 Your Questions ANSWERED

### Q1: Are WiFi QRs and Session QRs encrypted with DIFFERENT keys?

**ANSWER: ❌ NO - Currently BOTH use the SAME shared key!**

This is a **SECURITY ISSUE** that needs to be fixed.

---

## 📊 Current Implementation Analysis

### WiFi QR Codes (Admin Portal)
**Location:** `tools/admin_portal.html`

```javascript
// Line 408-409: WiFi QR generation
const deviceKey = document.getElementById('device-key').value.trim();
const encrypted = aes256Encrypt(JSON.stringify(wifiData), deviceKey);
```

**Expected Behavior:** Should use device-specific key
**Actual Reality:** User manually enters `device_key`, but this is NOT enforced by RDK!

---

### Session QR Codes (Server)
**Location:** `ec2-server/server.py`

```python
# Line 56: Shared encryption key
QR_ENCRYPTION_KEY = os.getenv('QR_ENCRYPTION_KEY', 'mdai_qr_encryption_key_32byte!')[:32]

# Line 333-342: Session QR encryption
encrypted_payload = aes_encrypt(json.dumps(payload_data), QR_ENCRYPTION_KEY)
```

**Uses:** Hardcoded shared key for ALL devices

---

### RDK Decryption (Device Side)
**Location:** `src/SystemController.cpp`

```cpp
// Line 286-291: Session QR decryption
if (qr_json.contains("qr_encrypted") || qr_json.contains("e")) {
    std::string qr_key = "mdai_qr_encryption_key_32byte!"; // Same as server
    std::string decrypted_json = CryptoUtils::aes256_decrypt(encrypted_data, qr_key);
    // ... process session
}
```

**PROBLEM:** Device ALWAYS uses the same hardcoded key!

---

## 🚨 CRITICAL SECURITY ISSUE

### Current Reality:

```
WiFi QR:    AES-256 with "mdai_qr_encryption_key_32byte!"  (hardcoded)
Session QR: AES-256 with "mdai_qr_encryption_key_32byte!"  (hardcoded)

BOTH USE THE SAME KEY! ❌
```

### What You Probably INTENDED:

```
WiFi QR:    AES-256 with device_key (unique per device) ✅
Session QR: AES-256 with shared key (same for all)      ✅
```

---

## 🔧 How RDK Currently "Knows" Which Key to Use

### Answer: **IT DOESN'T!**

The RDK uses the **SAME hardcoded key** for everything:

```cpp
// SystemController.cpp line 288
std::string qr_key = "mdai_qr_encryption_key_32byte!";
```

### The RDK Never Tries Multiple Keys

Looking at the code flow:

```cpp
void SystemController::handle_idle(FrameBox* frame) {
    // Scan QR code
    std::string decoded_info = qr_decoder.detectAndDecode(gray);
    
    if (!decoded_info.empty()) {
        nlohmann::json qr_json = nlohmann::json::parse(decoded_info);
        
        // Check if encrypted
        if (qr_json.contains("qr_encrypted")) {
            // ALWAYS uses same key - no fallback!
            std::string qr_key = "mdai_qr_encryption_key_32byte!";
            std::string decrypted = CryptoUtils::aes256_decrypt(encrypted_data, qr_key);
            // ...
        }
    }
}
```

**No key selection logic exists!**

---

## 🎯 What SHOULD Happen (Recommended Fix)

### Option 1: Different Keys with Type Indicator (RECOMMENDED)

```json
// WiFi QR format (encrypted with device_key)
{
  "type": "wifi",
  "encrypted": "base64_encrypted_data_here",
  "device_id": "MDAI-b42f03319a35"
}

// Session QR format (encrypted with shared key)
{
  "type": "session",
  "qr_encrypted": "base64_encrypted_data_here"
}
```

**RDK Logic:**
```cpp
if (qr_json.contains("type")) {
    std::string qr_type = qr_json["type"];
    
    if (qr_type == "wifi") {
        // Use device-specific key (from TPM)
        std::string device_key = CryptoUtils::get_device_key();
        decrypted = aes256_decrypt(encrypted_data, device_key);
        
    } else if (qr_type == "session") {
        // Use shared key
        std::string shared_key = "mdai_qr_encryption_key_32byte!";
        decrypted = aes256_decrypt(encrypted_data, shared_key);
    }
}
```

---

### Option 2: Try Multiple Keys (Less Secure, Not Recommended)

```cpp
// Try device key first
try {
    std::string device_key = CryptoUtils::get_device_key();
    decrypted = aes256_decrypt(encrypted_data, device_key);
    // Success with device key
} catch (...) {
    // Failed, try shared key
    try {
        std::string shared_key = "mdai_qr_encryption_key_32byte!";
        decrypted = aes256_decrypt(encrypted_data, shared_key);
        // Success with shared key
    } catch (...) {
        // Both failed - invalid QR
    }
}
```

**Why this is bad:**
- Performance overhead (2x decryption attempts)
- Security risk (attacker can craft QR to succeed with wrong key)
- No clear intent/purpose

---

## 🔐 Proper Security Architecture

### Device Key Generation
**Location:** `src/CryptoUtils.cpp`

```cpp
// Line 199-207: Device key derivation
std::string CryptoUtils::derive_device_key(
    const std::string& master_secret, 
    const std::string& hardware_id
) {
    // HMAC-SHA256(master_secret, hardware_id)
    return hmac_sha256(hardware_id, master_secret);
}

std::string CryptoUtils::get_device_key() {
    std::string hardware_id = get_hardware_id();  // MAC address
    return derive_device_key(MASTER_SECRET, hardware_id);
}
```

**MASTER_SECRET Location:** `include/CryptoUtils.hpp`
```cpp
static constexpr const char* MASTER_SECRET = "mdai-master-secret-key-v1-change-in-production";
```

**Formula:**
```
device_key = HMAC-SHA256(MASTER_SECRET, MAC_ADDRESS)
```

Each device gets a **unique** 256-bit key derived from its MAC address.

---

## 📋 Current QR Format Comparison

### Session QR (from EC2 Server)

**Python Server Code:**
```python
# Line 333-348
payload_data = {
    "session_id": session_id,
    "token": jwt_token,
    "timestamp": int(time.time()),
    "ws_url": f"ws://{SERVER_HOST}/ws/device"
}

encrypted_payload = aes_encrypt(json.dumps(payload_data), QR_ENCRYPTION_KEY)

qr_data = {
    "qr_encrypted": encrypted_payload
}
```

**Encryption Key:** `QR_ENCRYPTION_KEY` (shared, hardcoded)

**Final QR JSON:**
```json
{
  "qr_encrypted": "AjK9x... (base64 encrypted data)"
}
```

---

### WiFi QR (from Admin Portal HTML)

**JavaScript Code:**
```javascript
const wifiData = {
    type: "wifi_setup",
    ssid: "MyNetwork",
    password: "password123",
    security: "WPA2",
    timestamp: "2025-11-21T10:30:00Z"
};

const encrypted = aes256Encrypt(JSON.stringify(wifiData), deviceKey);

// QR contains just the encrypted string (no JSON wrapper)
// QR data = "AjK9x..." (base64 directly)
```

**Encryption Key:** User manually enters `device_key`

**Final QR:** Just base64 string (no JSON)

---

## ⚠️ Security Vulnerabilities

### Issue 1: Same Key for Both QR Types
**Risk:** If shared key is compromised, attacker can:
- Decrypt ALL session QRs for ALL devices
- Create fake WiFi QRs
- Man-in-the-middle attacks

**Severity:** 🔴 HIGH

---

### Issue 2: WiFi QR Doesn't Actually Use Device Key
**Risk:** Admin portal CLAIMS to use device_key, but:
- RDK ignores it and uses shared key
- Device_key input is meaningless
- No per-device encryption

**Severity:** 🔴 HIGH

---

### Issue 3: Hardcoded Shared Key in Source Code
**Risk:** 
- Key visible in GitHub repo
- Anyone can decrypt QRs
- Cannot be changed without recompiling

**Location:**
- `ec2-server/server.py` line 56
- `src/SystemController.cpp` line 288

**Severity:** 🔴 CRITICAL

---

### Issue 4: No QR Type Differentiation
**Risk:**
- Can't tell WiFi QR from Session QR
- Both processed identically
- Prone to mistakes

**Severity:** 🟡 MEDIUM

---

## ✅ Recommended Solution

### 1. Add Type Field to All QRs

```json
// WiFi QR (encrypted with device_key)
{
  "type": "wifi",
  "device_id": "MDAI-b42f03319a35",
  "encrypted": "base64_encrypted_payload"
}

// Encrypted payload contains:
{
  "ssid": "MyNetwork",
  "password": "password123",
  "security": "WPA2",
  "timestamp": 1732192800
}
```

```json
// Session QR (encrypted with shared_key)
{
  "type": "session",
  "qr_encrypted": "base64_encrypted_payload"
}

// Encrypted payload contains:
{
  "session_id": "sess_xyz",
  "token": "eyJ...",
  "timestamp": 1732192800,
  "ws_url": "ws://mdai.mercle.ai/ws/device"
}
```

---

### 2. Update RDK Decryption Logic

```cpp
void SystemController::handle_idle(FrameBox* frame) {
    std::string decoded_info = qr_decoder.detectAndDecode(gray);
    
    if (!decoded_info.empty()) {
        nlohmann::json qr_json = nlohmann::json::parse(decoded_info);
        
        // Determine QR type
        std::string qr_type = qr_json.value("type", "unknown");
        std::string encrypted_data;
        std::string decryption_key;
        
        if (qr_type == "wifi") {
            // WiFi QR - use device-specific key
            encrypted_data = qr_json["encrypted"];
            decryption_key = CryptoUtils::get_device_key();
            std::cout << "🔐 WiFi QR detected (device key)" << std::endl;
            
        } else if (qr_type == "session") {
            // Session QR - use shared key
            encrypted_data = qr_json["qr_encrypted"];
            decryption_key = QR_SHARED_KEY;  // From config
            std::cout << "🔐 Session QR detected (shared key)" << std::endl;
            
        } else {
            // Legacy format - assume session QR
            if (qr_json.contains("qr_encrypted")) {
                encrypted_data = qr_json["qr_encrypted"];
                decryption_key = QR_SHARED_KEY;
            } else {
                std::cerr << "⚠ Unknown QR format" << std::endl;
                return;
            }
        }
        
        // Decrypt with appropriate key
        try {
            std::string decrypted = CryptoUtils::aes256_decrypt(
                encrypted_data, 
                decryption_key
            );
            
            // Process decrypted data based on type
            if (qr_type == "wifi") {
                process_wifi_credentials(decrypted);
            } else {
                process_session_data(decrypted);
            }
            
        } catch (const std::exception& e) {
            std::cerr << "⚠ QR decryption failed: " << e.what() << std::endl;
        }
    }
}
```

---

### 3. Move Shared Key to Config File

**Create:** `/opt/mdai/qr_keys.conf`

```ini
[encryption]
# Shared key for session QRs (must match server)
shared_key=generate_random_256bit_key_here

# Device key is derived from hardware (MAC address + master secret)
# See device_config.json for device_id
```

**Load in RDK:**
```cpp
class SystemController {
private:
    std::string qr_shared_key_;  // Loaded from config
    
    void load_qr_config() {
        // Load from /opt/mdai/qr_keys.conf
        // Or from device_config.json
    }
};
```

---

### 4. Update Server to Support Both Keys

```python
# ec2-server/server.py

# Shared key for session QRs (all devices)
QR_SHARED_KEY = os.getenv('QR_SHARED_KEY', generate_secure_key())

# Function to get device-specific key
def get_device_key(hardware_id: str) -> str:
    """Derive device key from hardware ID (same formula as RDK)"""
    master_secret = os.getenv('MASTER_SECRET', 'mdai-master-secret-key-v1')
    return hmac_sha256(hardware_id, master_secret)

# For WiFi QR generation endpoint
@app.post("/api/admin/generate-wifi-qr")
async def generate_wifi_qr(
    hardware_id: str,
    ssid: str,
    password: str
):
    # Use device-specific key
    device_key = get_device_key(hardware_id)
    
    payload = {
        "type": "wifi",
        "ssid": ssid,
        "password": password,
        "timestamp": int(time.time())
    }
    
    encrypted = aes_encrypt(json.dumps(payload), device_key)
    
    return {
        "type": "wifi",
        "device_id": f"MDAI-{hardware_id}",
        "encrypted": encrypted
    }

# For session QR (existing)
@app.get("/api/mobile/create-session")
async def create_session():
    # Use shared key
    payload = {...}
    encrypted = aes_encrypt(json.dumps(payload), QR_SHARED_KEY)
    
    return {
        "type": "session",
        "qr_encrypted": encrypted
    }
```

---

## 📊 Summary Table

| QR Type | Current Key | Should Use | Risk Level |
|---------|------------|------------|------------|
| **WiFi QR** | Hardcoded shared key `mdai_qr_encryption_key_32byte!` | Device-specific key (derived from MAC) | 🔴 HIGH |
| **Session QR** | Hardcoded shared key `mdai_qr_encryption_key_32byte!` | Shared key (but in config, not code) | 🟡 MEDIUM |

---

## 🎯 Action Items

### Priority 1: CRITICAL (Do Immediately)
- [ ] Move shared key from source code to environment variable
- [ ] Generate strong random 256-bit key for production
- [ ] Update EC2 server to use new key from env
- [ ] Update RDK to load key from config file

### Priority 2: HIGH (Next Week)
- [ ] Add `type` field to all QR codes
- [ ] Implement device_key decryption for WiFi QRs
- [ ] Update admin portal to use correct device keys
- [ ] Add key selection logic in RDK

### Priority 3: MEDIUM (This Month)
- [ ] Add key rotation mechanism
- [ ] Implement QR expiry checking
- [ ] Add device_id validation for WiFi QRs
- [ ] Log QR decryption attempts for security audit

---

## 🧪 Testing Commands

### Test Current Shared Key Decryption

```bash
# Create test QR with Python
python3 << 'EOF'
import json
import base64
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.backends import default_backend
import secrets

key = b"mdai_qr_encryption_key_32byte!"[:32]
plaintext = json.dumps({"session_id": "test123", "token": "abc"})

iv = secrets.token_bytes(16)
cipher = Cipher(algorithms.AES(key), modes.CBC(iv), backend=default_backend())
encryptor = cipher.encryptor()

# Pad
pad_len = 16 - (len(plaintext) % 16)
padded = plaintext + (chr(pad_len) * pad_len)

ciphertext = encryptor.update(padded.encode()) + encryptor.finalize()
encrypted = base64.b64encode(iv + ciphertext).decode()

qr_data = {"qr_encrypted": encrypted}
print(json.dumps(qr_data))
EOF
```

---

## 📚 References

- **Server Code:** `/home/mercleDev/codebase/ec2-server/server.py` (line 56, 333-348)
- **RDK Code:** `/home/mercleDev/codebase/src/SystemController.cpp` (line 286-291)
- **Crypto Utils:** `/home/mercleDev/codebase/src/CryptoUtils.cpp` (line 199-207, 315-369)
- **Admin Portal:** `/home/mercleDev/codebase/tools/admin_portal.html` (line 408-409)

---

**Last Updated:** 2025-11-21
**Security Status:** 🔴 CRITICAL ISSUES FOUND

