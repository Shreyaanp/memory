# MDAI RDK Service Architecture

## Overview

The RDK (Reference Development Kit) device is a **dedicated single-task system** designed to run face detection and liveness verification. The entire device is optimized for running one primary service: `mdai_system`.

```
┌─────────────────────────────────────────────────────────────────────┐
│                         RDK DEVICE                                   │
│                                                                      │
│   ┌──────────────────────────────────────────────────────────────┐  │
│   │                    mdai_system (C++)                          │  │
│   │                                                               │  │
│   │   • IR Camera capture (RealSense)                            │  │
│   │   • Face detection (MediaPipe)                               │  │
│   │   • Liveness verification                                     │  │
│   │   • WebSocket communication to EC2 server                     │  │
│   │   • Serial communication to ESP32 display                     │  │
│   │   • QR code decryption (AES-256)                             │  │
│   │                                                               │  │
│   └──────────────────────────────────────────────────────────────┘  │
│                              ▲                                       │
│                              │                                       │
│   ┌──────────────────────────┴───────────────────────────────────┐  │
│   │                   Supporting Services                         │  │
│   │   • NetworkManager (WiFi)                                     │  │
│   │   • Tailscale (VPN remote access)                            │  │
│   │   • SSH (maintenance)                                         │  │
│   │   • systemd (service management)                              │  │
│   └──────────────────────────────────────────────────────────────┘  │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Hardware Specifications (Test Device)

| Component | Specification |
|-----------|---------------|
| **Hostname** | rdk-dev-001 |
| **OS** | Ubuntu 22.04.5 LTS |
| **Architecture** | ARM64 (aarch64) |
| **CPU** | 8 cores |
| **RAM** | 7 GB |
| **Kernel** | 6.1.83 PREEMPT |

---

## Service Architecture

### Primary Service: mdai-system

The main application that must run at all times.

```
/etc/systemd/system/mdai-system.service
```

**Key Configuration:**

| Setting | Value | Purpose |
|---------|-------|---------|
| `User` | root | Required for real-time priority |
| `Nice` | -20 | Highest CPU priority |
| `OOMScoreAdjust` | -1000 | Never killed by OOM |
| `MemoryMin` | 512M | Reserved memory |
| `Restart` | always | Auto-restart on failure |
| `RestartSec` | 2 | Restart delay |

**Boot Order:**
```
network-online.target
       ↓
NetworkManager.service
       ↓
wpa_supplicant.service
       ↓
tailscaled.service
       ↓
mdai-system.service  ← Main service starts here
```

### Backup Service: mdai-watchdog

Secondary watchdog that monitors and restarts mdai-system if needed.

```
/etc/systemd/system/mdai-watchdog.service
```

---

## Directory Structure

```
/home/mercleDev/codebase/
├── build/
│   └── mdai_system          # Main executable
├── src/                     # C++ source code
├── include/                 # Header files
├── lib/
│   └── mediapipe/          # MediaPipe libraries
├── models/                  # ML models
├── deployment/              # Deployment scripts
│   ├── mdai-system.service
│   ├── mdai-watchdog.service
│   ├── watchdog.sh
│   ├── install.sh
│   ├── optimize-system.sh
│   └── status.sh
├── rdk-service/            # Architecture docs (this folder)
└── mdai.sh                 # Manual control script

/home/mercleDev/mdai_logs/
├── system.log              # Main application logs
└── watchdog.log            # Watchdog logs
```

---

## Network Configuration

### Interfaces

| Interface | Purpose | IP Address |
|-----------|---------|------------|
| `wlan0` | Primary WiFi | DHCP (e.g., 10.104.72.230) |
| `tailscale0` | VPN remote access | 100.x.x.x |
| `eth0` | Local debugging (Ichiro) | 192.168.127.10 (static) |

### Connectivity Flow

```
RDK Device ──WiFi──► Internet ──► EC2 Server (WebSocket)
     │
     └──Tailscale──► Remote SSH Access
```

---

## Service Dependencies

### Required Services (DO NOT DISABLE)

| Service | Purpose |
|---------|---------|
| `NetworkManager` | WiFi management |
| `wpa_supplicant` | WiFi authentication |
| `tailscaled` | Tailscale VPN |
| `ssh` | Remote access |
| `systemd-journald` | Logging |
| `systemd-udevd` | Device management |
| `dbus` | System bus |

### Disabled Services (Not Needed)

| Service | Reason |
|---------|--------|
| `docker` | Not used |
| `containerd` | Not used |
| `xrdp` | Use SSH instead |
| `avahi-daemon` | mDNS not needed |
| `ModemManager` | No modem |
| `atd` | Deferred execution not needed |

---

## Production Migration Checklist

### 1. Pre-Deployment

- [ ] Build mdai_system binary: `cd build && make -j7`
- [ ] Verify binary runs: `./mdai_system` (manual test)
- [ ] Configure WiFi credentials
- [ ] Set unique hostname: `sudo hostnamectl set-hostname rdk-prod-XXX`

### 2. Installation

```bash
# Copy deployment files to device
scp -r codebase/deployment/ user@device:/home/user/codebase/

# SSH to device
ssh user@device

# Run installation
cd /home/user/codebase/deployment
sudo ./install.sh

# Optimize system (disable unnecessary services)
sudo ./optimize-system.sh
```

### 3. Configuration

Create `/home/user/codebase/.env` (if needed):
```env
# EC2 Server URL
WS_SERVER_URL=wss://mdai.mercle.ai/ws/device

# QR Encryption Key (must match EC2 server)
QR_ENCRYPTION_KEY=<64-hex-chars>
```

### 4. Post-Deployment

```bash
# Verify services
./status.sh

# Test reboot
sudo reboot

# After reboot, verify auto-start
./status.sh
```

### 5. Production Hardening (Optional)

- [ ] Disable SSH password auth (use keys only)
- [ ] Configure firewall (ufw)
- [ ] Set up log rotation
- [ ] Configure automatic updates (or disable for stability)

---

## Quick Reference Commands

```bash
# Check status
/home/mercleDev/codebase/deployment/status.sh

# View real-time logs
tail -f /home/mercleDev/mdai_logs/system.log

# Restart service
sudo systemctl restart mdai-system

# Stop service
sudo systemctl stop mdai-system

# Rebuild and restart
cd /home/mercleDev/codebase/build
make -j7 && sudo systemctl restart mdai-system

# Check service details
sudo systemctl status mdai-system -l

# Check boot order
systemd-analyze critical-chain mdai-system.service
```

---

## Troubleshooting

### Service Won't Start

```bash
# Check logs
sudo journalctl -xeu mdai-system.service

# Common issues:
# - Missing libraries: check LD_LIBRARY_PATH
# - Permission denied: ensure running as root
# - Binary not found: rebuild with make
```

### Network Issues

```bash
# Check WiFi
nmcli device wifi list
nmcli connection show

# Check Tailscale
tailscale status
sudo systemctl status tailscaled
```

### High CPU / Memory

```bash
# Check process
ps aux | grep mdai_system
top -p $(pgrep mdai_system)

# Check ring buffer logs for warnings
grep "Ring buffer" /home/mercleDev/mdai_logs/system.log
```

---

## Version History

| Date | Version | Changes |
|------|---------|---------|
| 2025-11-29 | 1.0 | Initial deployment architecture |

---

## Contact

- **Repository**: /home/mercleDev/codebase
- **EC2 Server**: /home/mercleDev/ec2-server


