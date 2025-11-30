# Production Deployment Guide

## Quick Deploy to New RDK Device

### Prerequisites

- Ubuntu 22.04 LTS (ARM64)
- Network connectivity (WiFi configured)
- Root access

---

## Step 1: Transfer Files

From your development machine:

```bash
# Set target device IP
TARGET=user@<device-ip>

# Transfer codebase
rsync -avz --exclude='build/' --exclude='.git/' \
    /path/to/codebase/ $TARGET:/home/user/codebase/
```

---

## Step 2: Build on Device

SSH to the device:

```bash
ssh $TARGET

# Install dependencies (if not already installed)
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev

# Build
cd /home/user/codebase
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

---

## Step 3: Install Service

```bash
# Run installer
cd /home/user/codebase/deployment
sudo ./install.sh

# Optimize system
sudo ./optimize-system.sh
```

---

## Step 4: Configure

### Set Hostname

```bash
sudo hostnamectl set-hostname rdk-prod-001
```

### Configure Tailscale

```bash
sudo tailscale up --authkey=<your-auth-key>
```

### Configure WiFi (if needed)

```bash
# List networks
nmcli device wifi list

# Connect
nmcli device wifi connect "SSID" password "password"
```

---

## Step 5: Verify

```bash
# Check status
./status.sh

# Test reboot
sudo reboot

# After reboot
./status.sh
```

---

## Files to Copy for Each New Device

```
codebase/
├── build/mdai_system          # Pre-built binary (or build on device)
├── lib/mediapipe/             # MediaPipe libraries
├── models/                    # ML models
├── deployment/
│   ├── mdai-system.service
│   ├── mdai-watchdog.service
│   ├── watchdog.sh
│   ├── install.sh
│   ├── optimize-system.sh
│   └── status.sh
└── (source files if building on device)
```

---

## Production Checklist

### Per Device

- [ ] Unique hostname set (`rdk-prod-XXX`)
- [ ] WiFi configured and tested
- [ ] Tailscale joined to network
- [ ] mdai-system.service installed and enabled
- [ ] mdai-watchdog.service installed and enabled
- [ ] Unnecessary services disabled
- [ ] Reboot tested - services auto-start
- [ ] Logs verified - no errors

### Network Requirements

| Port | Direction | Purpose |
|------|-----------|---------|
| 443 | Outbound | WebSocket to EC2 server |
| 41641 | Outbound | Tailscale |
| 22 | Inbound (Tailscale) | SSH access |

---

## Monitoring

### Check Device Health

```bash
# Via Tailscale
ssh user@100.x.x.x '/home/user/codebase/deployment/status.sh'
```

### Bulk Check (Multiple Devices)

```bash
#!/bin/bash
DEVICES=(
    "100.127.197.95"  # rdk-prod-001
    # Add more IPs
)

for ip in "${DEVICES[@]}"; do
    echo "=== Checking $ip ==="
    ssh -o ConnectTimeout=5 user@$ip 'systemctl is-active mdai-system' 2>/dev/null || echo "OFFLINE"
done
```



