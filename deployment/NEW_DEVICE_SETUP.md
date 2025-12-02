# MDAI System - New RDK Device Setup

Complete guide for deploying `mdai_system` to a new RDK X5 device.

---

## Quick Reference: SD Card Image Creation

### Your Workflow (From Ichiro → SD Card)

```
┌─────────────┐      SSH       ┌─────────────┐      Copy      ┌─────────────┐
│   Ichiro    │ ────────────► │  Source RDK │ ────────────► │  SD Card    │
│  (MacBook)  │               │ (Working)   │               │  (Image)    │
└─────────────┘               └─────────────┘               └─────────────┘
                                    │
                                    ▼
                          Only copy SHARED files
                          NOT device-specific configs
```

### Files to Copy FROM Source RDK (Static/Shared)

| Category | Path | Copy? |
|----------|------|-------|
| **Application** | `/home/mercleDev/codebase/build/mdai_system` | ✅ YES |
| **Application** | `/home/mercleDev/codebase/build/libmdai_realsense.so` | ✅ YES |
| **MediaPipe** | `/home/mercleDev/mediapipe_arm64_final/` | ✅ YES |
| **Models** | `/home/mercleDev/codebase/models/` | ✅ YES |
| **Libraries** | `/usr/local/lib/librealsense2.so*` | ✅ YES |
| **Libraries** | `/usr/local/lib/libquirc.a` | ✅ YES |
| **Services** | `/etc/systemd/system/wifi-always-on.service` | ✅ YES |
| **Services** | `/etc/systemd/system/tailscale-watchdog.service` | ✅ YES |
| **Services** | `/etc/systemd/system/tailscale-dns-monitor.service` | ✅ YES |
| **Scripts** | `/usr/local/bin/wifi-always-on.sh` | ✅ YES |
| **Scripts** | `/usr/local/bin/tailscale-watchdog.sh` | ✅ YES |
| **Scripts** | `/usr/local/bin/tailscale-dns-monitor.sh` | ✅ YES |
| **Deployment** | `/home/mercleDev/codebase/deployment/` | ✅ YES |

### Files to NOT Copy (Device-Specific)

| Path | Why NOT |
|------|---------|
| `/opt/mdai/device_config.json` | Contains THIS device's secret |
| `/etc/mdai/device_config.json` | Contains THIS device's ID |
| `/opt/mdai/.registered` | Flag prevents re-registration |
| `/opt/mdai/device_private.pem` | THIS device's private key |
| `/opt/mdai/device_public.pem` | THIS device's public key |
| `/var/lib/tailscale/` | Tailscale device identity |

### Files to ADD (Not from this RDK)

| File | Source | Purpose |
|------|--------|---------|
| `ec2_register.pem` | Your secure storage | SSH key for first-boot registration (deleted after use) |

---

## SD Card Image Checklist

Before flashing SD card, verify:

- [ ] **Base OS**: RDK X5 Ubuntu 22.04 image
- [ ] **Application files copied**: `mdai_system`, `libmdai_realsense.so`
- [ ] **MediaPipe copied**: `mediapipe_arm64_final/` folder
- [ ] **Models copied**: `face_landmarker.task`, etc.
- [ ] **Libraries copied**: `librealsense2.so*`, `libquirc.a` → `/usr/local/lib/`
- [ ] **Services copied**: All `.service` files
- [ ] **Scripts copied**: All self-healing `.sh` scripts
- [ ] **Registration key added**: `ec2_register.pem` → `/home/mercleDev/.ssh/`
- [ ] **No device configs**: `/opt/mdai/` should be empty or non-existent
- [ ] **No /etc/mdai/**: This folder should NOT exist

---

## Copy Commands (Run from Ichiro)

```bash
# Variables
RDK_IP="<source-rdk-ip>"  # Your working RDK
LOCAL_DIR="./rdk_image_files"

# Create local staging directory
mkdir -p $LOCAL_DIR/{home/mercleDev,usr/local/lib,etc/systemd/system,usr/local/bin}

# Copy application
scp mercleDev@$RDK_IP:~/codebase/build/mdai_system $LOCAL_DIR/home/mercleDev/codebase/build/
scp mercleDev@$RDK_IP:~/codebase/build/libmdai_realsense.so $LOCAL_DIR/home/mercleDev/codebase/build/

# Copy MediaPipe (entire folder)
scp -r mercleDev@$RDK_IP:~/mediapipe_arm64_final $LOCAL_DIR/home/mercleDev/

# Copy models
scp -r mercleDev@$RDK_IP:~/codebase/models $LOCAL_DIR/home/mercleDev/codebase/

# Copy deployment scripts
scp -r mercleDev@$RDK_IP:~/codebase/deployment $LOCAL_DIR/home/mercleDev/codebase/

# Copy libraries
scp mercleDev@$RDK_IP:/usr/local/lib/librealsense2.so* $LOCAL_DIR/usr/local/lib/
scp mercleDev@$RDK_IP:/usr/local/lib/libquirc.a $LOCAL_DIR/usr/local/lib/

# Copy self-healing services
scp mercleDev@$RDK_IP:/etc/systemd/system/wifi-always-on.service $LOCAL_DIR/etc/systemd/system/
scp mercleDev@$RDK_IP:/etc/systemd/system/tailscale-watchdog.service $LOCAL_DIR/etc/systemd/system/
scp mercleDev@$RDK_IP:/etc/systemd/system/tailscale-dns-monitor.service $LOCAL_DIR/etc/systemd/system/

# Copy self-healing scripts
scp mercleDev@$RDK_IP:/usr/local/bin/wifi-always-on.sh $LOCAL_DIR/usr/local/bin/
scp mercleDev@$RDK_IP:/usr/local/bin/tailscale-watchdog.sh $LOCAL_DIR/usr/local/bin/
scp mercleDev@$RDK_IP:/usr/local/bin/tailscale-dns-monitor.sh $LOCAL_DIR/usr/local/bin/

# ADD registration key (from YOUR secure storage, NOT the RDK)
cp /path/to/your/ec2_register.pem $LOCAL_DIR/home/mercleDev/.ssh/
```

---

## First Boot Sequence (What Happens Automatically)

```
BOOT
  │
  ├─► NetworkManager connects WiFi
  │
  ├─► mdai-register.service runs (if /opt/mdai/.registered doesn't exist)
  │     │
  │     ├─► Generates unique device_id from /etc/machine-id
  │     ├─► SSHs to EC2 using ec2_register.pem
  │     ├─► EC2 creates device entry, returns device_secret
  │     ├─► Saves /opt/mdai/device_config.json
  │     ├─► Saves /etc/mdai/device_config.json
  │     ├─► DELETES ec2_register.pem (one-time use)
  │     └─► Creates /opt/mdai/.registered flag
  │
  └─► mdai-system.service starts (needs registration complete)
```

---

## Post-Flash Manual Steps

After first boot, you still need to:

1. **Set hostname**: `sudo hostnamectl set-hostname rdk-prod-XXX`
2. **Join Tailscale**: `sudo tailscale up --authkey=<your-key>`
3. **Verify registration**: `cat /opt/mdai/device_config.json`

---

## What You Need to Install

The RDK X5 comes with Ubuntu 22.04 and basic system services pre-installed.
This guide covers **only what you need to add** to run mdai_system.

---

## Services to Install

### 1. MDAI Application Services (MUST INSTALL)

| Service | Purpose | Source File |
|---------|---------|-------------|
| `mdai-system.service` | Main face detection app | `deployment/mdai-system.service` |
| `mdai-watchdog.service` | Auto-restarts mdai-system | `deployment/mdai-watchdog.service` |

**Scripts:**
- `deployment/watchdog.sh` - Watchdog script

---

### 2. Self-Healing Services (MUST INSTALL)

| Service | Purpose | Files to Copy |
|---------|---------|---------------|
| `wifi-always-on.service` | Ensures WiFi stays enabled on boot | Service + Script |
| `tailscale-watchdog.service` | Auto-restarts Tailscale if down | Service + Script |
| `tailscale-dns-monitor.service` | Fixes MagicDNS issues | Service + Script |

**Files to copy:**
```
/etc/systemd/system/wifi-always-on.service
/usr/local/bin/wifi-always-on.sh

/etc/systemd/system/tailscale-watchdog.service
/usr/local/bin/tailscale-watchdog.sh

/etc/systemd/system/tailscale-dns-monitor.service
/usr/local/bin/tailscale-dns-monitor.sh
```

---

### 3. Tailscale (MUST INSTALL)

Tailscale is NOT pre-installed on RDK. Install it:

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo systemctl enable --now tailscaled
sudo tailscale up --authkey=<your-key>
```

---

### 4. Additional Packages (MUST INSTALL)

These are NOT on the base RDK image:

```bash
# NTP for time sync
sudo apt install -y ntp

# Hardware watchdog daemon
sudo apt install -y watchdog
sudo systemctl enable watchdog
```

---

## Pre-Installed on RDK X5 (NO ACTION NEEDED)

These come with the RDK X5 factory image - **do not reinstall**:

| Service | Purpose |
|---------|---------|
| `NetworkManager.service` | WiFi management |
| `wpa_supplicant.service` | WiFi authentication |
| `ssh.service` | SSH access |
| `dbus.service` | System bus |
| `systemd-journald.service` | Logging |
| `systemd-udevd.service` | Device/USB management |
| `systemd-resolved.service` | DNS |
| `rtkit-daemon.service` | Real-time scheduling |
| `polkit.service` | Authorization |
| `apparmor.service` | Security |
| `hobot-*.service` | Hardware drivers |

---

## Libraries to Install

### From APT (if not present)

```bash
sudo apt update
sudo apt install -y \
    libopencv-dev \
    libopencv-contrib-dev \
    libssl-dev \
    libtbb-dev \
    libabsl-dev \
    libusb-1.0-0-dev \
    libprotobuf-dev \
    libcurl4-openssl-dev \
    nlohmann-json3-dev \
    ntp \
    watchdog
```

### From Source (MUST COPY)

These are built from source and must be copied from the working RDK:

| Library | Location | Purpose |
|---------|----------|---------|
| `librealsense2.so*` | `/usr/local/lib/` | Intel RealSense camera |
| `libface_landmarker.so` | `~/mediapipe_arm64_final/lib/` | MediaPipe face detection |
| `libquirc.a` | `/usr/local/lib/` | QR code detection |

---

## Device Registration (CRITICAL)

Each RDK device must be registered with the EC2 server before it can operate.

### How Registration Works

```
FIRST BOOT:
                                    
  New RDK Device                     EC2 Server
  ──────────────                     ──────────
       │                                  │
       │  1. Generate device_id           │
       │     (from /etc/machine-id)       │
       │                                  │
       │  2. SSH into EC2 ───────────────►│
       │     (using pre-loaded key)       │
       │                                  │
       │                           3. Generate device_secret
       │                           4. Insert into devices table
       │                                  │
       │◄── Return device_secret ─────────│
       │                                  │
       │  5. Save to /opt/mdai/           │
       │  6. DELETE SSH key               │
       │                                  │
       ▼                                  ▼
   REGISTERED                        DEVICE IN DB
```

### Prerequisites for New Device

1. **SSH Key**: Place `ec2_register.pem` at `/home/mercleDev/.ssh/ec2_register.pem`
2. **Permissions**: `chmod 600 ~/.ssh/ec2_register.pem`
3. **Registration script**: Included in deployment folder

### Automatic Registration (Recommended)

The `mdai-register.service` runs on first boot and handles everything:

```bash
# Install the service
sudo cp deployment/mdai-register.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable mdai-register
```

On first boot:
- Service runs `register-device.sh`
- Device registers with EC2
- SSH key is deleted automatically
- Device config saved to `/opt/mdai/device_config.json`

### Manual Registration (If Needed)

```bash
# Run registration manually
./deployment/register-device.sh
```

### After Registration

Device will have TWO config files:

```
/opt/mdai/
├── device_config.json    # Full config (used by registration & QR decryption)
├── device_private.pem    # RSA private key (for future signature auth)
├── device_public.pem     # RSA public key
└── .registered           # Flag file (prevents re-registration)

/etc/mdai/
└── device_config.json    # Minimal config (used by C++ SystemController)
```

**`/opt/mdai/device_config.json`** (full):
```json
{
    "qr_shared_key": "ad5295ee97afce....",
    "device_id": "rdk_437e99da05f23221",
    "device_secret": "3f194119ceef26afe...",
    "hostname": "rdk-prod-001",
    "hardware_serial": "27b188d70ccc48d0...",
    "ec2_host": "mdai.mercle.ai",
    "registered_at": 1764644766
}
```

**`/etc/mdai/device_config.json`** (minimal):
```json
{
    "device_id": "rdk_437e99da05f23221",
    "hardware_id": "27b188d70ccc48d0..."
}
```

### Device Revocation

To revoke a device, update its status in EC2:

```bash
# SSH to EC2 and revoke device
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai
cd /opt/mdai && ./venv/bin/python3 -c "
import sqlite3
conn = sqlite3.connect('/var/lib/mdai/mdai_server.db')
cursor = conn.cursor()
cursor.execute('UPDATE devices SET status = \"revoked\" WHERE device_id = \"rdk_XXXXXXXX\"')
conn.commit()
print('Device revoked')
"
```

---

## Complete File List to Copy

### From Source RDK to New RDK

```
# SSH Key (CRITICAL - for registration only)
/home/mercleDev/.ssh/ec2_register.pem   → ~/.ssh/ec2_register.pem

# Application
/home/mercleDev/codebase/build/mdai_system
/home/mercleDev/codebase/build/libmdai_realsense.so

# MediaPipe
/home/mercleDev/mediapipe_arm64_final/  (entire folder)

# Models
/home/mercleDev/codebase/models/face_landmarker.task
/home/mercleDev/codebase/models/deploy_ssd.prototxt
/home/mercleDev/codebase/models/res10_300x300_ssd_iter_140000.caffemodel

# Deployment scripts
/home/mercleDev/codebase/deployment/mdai-system.service
/home/mercleDev/codebase/deployment/mdai-watchdog.service
/home/mercleDev/codebase/deployment/mdai-register.service    # First-boot registration
/home/mercleDev/codebase/deployment/watchdog.sh
/home/mercleDev/codebase/deployment/register-device.sh       # Device registration script
/home/mercleDev/codebase/deployment/install.sh
/home/mercleDev/codebase/deployment/optimize-system.sh
/home/mercleDev/codebase/deployment/status.sh

# WiFi Always-On
/etc/systemd/system/wifi-always-on.service
/usr/local/bin/wifi-always-on.sh

# Tailscale Watchdogs
/etc/systemd/system/tailscale-watchdog.service
/usr/local/bin/tailscale-watchdog.sh
/etc/systemd/system/tailscale-dns-monitor.service
/usr/local/bin/tailscale-dns-monitor.sh

# Libraries (to /usr/local/lib/)
/usr/local/lib/librealsense2.so*
/usr/local/lib/libquirc.a
```

---

## Installation Steps

### Step 1: Copy Files

From the source RDK, run:

```bash
./deployment/deploy_to_device.sh <new-rdk-ip> mercleDev
```

Or manually with rsync:

```bash
TARGET="mercleDev@<new-rdk-ip>"

# Copy application
rsync -avz ~/codebase/ $TARGET:~/codebase/
rsync -avz ~/mediapipe_arm64_final/ $TARGET:~/mediapipe_arm64_final/

# Copy SSH key for registration (CRITICAL!)
# Get this key from your secure storage - do NOT copy from existing RDK
rsync -avz /path/to/ec2_register.pem $TARGET:~/.ssh/ec2_register.pem

# Copy libraries
rsync -avz /usr/local/lib/librealsense2.so* $TARGET:/tmp/
rsync -avz /usr/local/lib/libquirc.a $TARGET:/tmp/

# Copy custom services
rsync -avz /etc/systemd/system/wifi-always-on.service $TARGET:/tmp/
rsync -avz /usr/local/bin/wifi-always-on.sh $TARGET:/tmp/
rsync -avz /etc/systemd/system/tailscale-watchdog.service $TARGET:/tmp/
rsync -avz /usr/local/bin/tailscale-watchdog.sh $TARGET:/tmp/
rsync -avz /etc/systemd/system/tailscale-dns-monitor.service $TARGET:/tmp/
rsync -avz /usr/local/bin/tailscale-dns-monitor.sh $TARGET:/tmp/
```

### Step 2: Install on New RDK

SSH to the new device and run:

```bash
# Move libraries
sudo mv /tmp/librealsense2.so* /usr/local/lib/
sudo mv /tmp/libquirc.a /usr/local/lib/
sudo ldconfig

# Move custom services
sudo mv /tmp/wifi-always-on.service /etc/systemd/system/
sudo mv /tmp/wifi-always-on.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/wifi-always-on.sh

sudo mv /tmp/tailscale-watchdog.service /etc/systemd/system/
sudo mv /tmp/tailscale-watchdog.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/tailscale-watchdog.sh

sudo mv /tmp/tailscale-dns-monitor.service /etc/systemd/system/
sudo mv /tmp/tailscale-dns-monitor.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/tailscale-dns-monitor.sh

# Install apt packages
sudo apt update
sudo apt install -y ntp watchdog

# Install Tailscale
curl -fsSL https://tailscale.com/install.sh | sh

# Install MDAI services
cd ~/codebase/deployment
sudo ./install.sh

# Install registration service (runs on first boot)
sudo cp mdai-register.service /etc/systemd/system/
chmod +x register-device.sh

# Enable all services
sudo systemctl daemon-reload
sudo systemctl enable mdai-register           # First-boot registration
sudo systemctl enable --now wifi-always-on
sudo systemctl enable --now tailscaled
sudo systemctl enable --now tailscale-watchdog
sudo systemctl enable --now tailscale-dns-monitor
sudo systemctl enable --now watchdog
sudo systemctl enable --now ntp
sudo systemctl enable --now mdai-system
sudo systemctl enable --now mdai-watchdog
```

### Step 3: Configure

```bash
# Set hostname
sudo hostnamectl set-hostname rdk-prod-XXX

# Join Tailscale network
sudo tailscale up --authkey=<your-key>

# Verify
./status.sh
```

### Step 4: Reboot Test

```bash
sudo reboot
# After reboot, verify all services started:
./status.sh
```

---

## Service Verification Script

Run this to verify all required services are running:

```bash
#!/bin/bash
echo "=== MDAI System Service Check ==="

SERVICES=(
    "mdai-system:MDAI App"
    "mdai-watchdog:MDAI Watchdog"
    "wifi-always-on:WiFi Always-On"
    "tailscaled:Tailscale VPN"
    "tailscale-watchdog:Tailscale Watchdog"
    "tailscale-dns-monitor:Tailscale DNS Monitor"
    "watchdog:Hardware Watchdog"
    "ntp:Time Sync"
    "NetworkManager:WiFi Manager"
    "wpa_supplicant:WiFi Auth"
    "ssh:SSH Access"
)

for item in "${SERVICES[@]}"; do
    svc="${item%%:*}"
    name="${item##*:}"
    if systemctl is-active --quiet "$svc"; then
        echo "✅ $name ($svc)"
    else
        echo "❌ $name ($svc) - NOT RUNNING"
    fi
done
```

---

## Summary: What to Install

| Category | Count | Services/Items |
|----------|-------|----------------|
| **MDAI App** | 2 | mdai-system, mdai-watchdog |
| **Self-Healing** | 3 | wifi-always-on, tailscale-watchdog, tailscale-dns-monitor |
| **Network** | 1 | tailscaled (install via script) |
| **System** | 2 | ntp, watchdog (install via apt) |
| **Libraries** | 3 | librealsense2, libface_landmarker, libquirc |

**Total custom services to install: 8**
**Total libraries to copy: 3**

---

## Troubleshooting

### Registration Failed
```bash
# Check if registration ran
journalctl -xeu mdai-register -n 50

# Check if SSH key exists (should NOT after successful registration)
ls -la ~/.ssh/ec2_register.pem

# Manual registration (if needed)
cd ~/codebase/deployment && ./register-device.sh
```

### "Device not registered" Error
```bash
# Check both config files exist
cat /opt/mdai/device_config.json
cat /etc/mdai/device_config.json

# If missing, run registration manually
./deployment/register-device.sh
```

### Check Service Status
```bash
systemctl status mdai-system
journalctl -xeu mdai-system -f
```

### Check Libraries
```bash
ldd ~/codebase/build/mdai_system | grep "not found"
```

### Check Camera
```bash
lsusb | grep Intel
rs-enumerate-devices
```

### Check Network
```bash
tailscale status
nmcli connection show
ping 8.8.8.8
```

---

## Version Info

| Component | Version |
|-----------|---------|
| Hardware | D-Robotics RDK X5 V1.0 |
| OS | Ubuntu 22.04.5 LTS |
| Kernel | 6.1.83 PREEMPT |
| MDAI System | v1.0.0 |
| RealSense SDK | 2.56 |
| OpenCV | 4.5.4 |
| MediaPipe | Native ARM64 |
