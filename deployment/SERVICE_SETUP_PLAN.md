# RDK Server Systemd Service Setup Plan

## Overview
Set up the RDK server (`mdai_system`) as a systemd service for proper process management.

## Current State
- ✅ Service file exists: `/home/mercleDev/codebase/deployment/mdai-system.service`
- ✅ Installation script exists: `/home/mercleDev/codebase/deployment/install_service.sh`
- ❌ Service is NOT currently installed
- ❌ Multiple manual processes running (needs cleanup)

## Service Configuration

### Service Name
`mdai-system.service`

### Service File Location
- Source: `/home/mercleDev/codebase/deployment/mdai-system.service`
- Target: `/etc/systemd/system/mdai-system.service`

### Key Features
1. **Auto-restart**: Restarts automatically if it crashes
2. **Logging**: Logs to both:
   - Systemd journal: `journalctl -u mdai-system -f`
   - Application log: `/tmp/mdai/rdk.log` (from main.cpp)
3. **USB Device Access**: Automatically sets permissions for ttyACM devices
4. **Network Dependency**: Waits for network to be online
5. **Resource Limits**: Proper file descriptor and process limits

## Installation Steps

### Step 1: Ensure Log Directory Exists
```bash
mkdir -p /tmp/mdai
chmod 755 /tmp/mdai
```

### Step 2: Kill All Existing Processes
```bash
sudo pkill -9 mdai_system
```

### Step 3: Install Service
```bash
cd /home/mercleDev/codebase/deployment
sudo ./install_service.sh
```

### Step 4: Verify Installation
```bash
sudo systemctl status mdai-system
```

## Service Management Commands

### Start Service
```bash
sudo systemctl start mdai-system
```

### Stop Service
```bash
sudo systemctl stop mdai-system
```

### Restart Service
```bash
sudo systemctl restart mdai-system
```

### Check Status
```bash
sudo systemctl status mdai-system
```

### View Logs (Systemd Journal)
```bash
# Follow logs in real-time
sudo journalctl -u mdai-system -f

# View last 100 lines
sudo journalctl -u mdai-system -n 100

# View logs since boot
sudo journalctl -u mdai-system -b
```

### View Application Logs
```bash
tail -f /tmp/mdai/rdk.log
```

### Enable Auto-Start on Boot
```bash
sudo systemctl enable mdai-system
```

### Disable Auto-Start
```bash
sudo systemctl disable mdai-system
```

## Service File Details

### Current Configuration
- **User**: root (needed for USB device access)
- **Working Directory**: `/home/mercleDev/codebase`
- **Executable**: `/home/mercleDev/codebase/build/mdai_system`
- **Restart Policy**: always (restarts on failure)
- **Restart Delay**: 10 seconds
- **Log Output**: Systemd journal

### Pre-Start Actions
1. Wait 5 seconds (for USB devices to initialize)
2. Set permissions on ttyACM devices (chmod 666)

## Benefits of Systemd Service

1. **Single Process Guarantee**: Systemd ensures only one instance runs
2. **Auto-Recovery**: Automatically restarts if process crashes
3. **Boot Integration**: Can start automatically on system boot
4. **Log Management**: Centralized logging via journalctl
5. **Resource Control**: Proper limits and resource management
6. **Dependency Management**: Waits for network before starting
7. **Clean Shutdown**: Proper signal handling on system shutdown

## Verification Checklist

After installation, verify:
- [ ] Service is running: `sudo systemctl status mdai-system`
- [ ] Only one process: `ps aux | grep mdai_system | grep -v grep | wc -l` (should be 1)
- [ ] Logs are being written: `tail -f /tmp/mdai/rdk.log`
- [ ] Service auto-restarts on failure (test by killing process)
- [ ] Service starts on boot (if enabled)

## Troubleshooting

### Service Won't Start
```bash
# Check service status
sudo systemctl status mdai-system

# Check logs
sudo journalctl -u mdai-system -n 50

# Check if binary exists
ls -la /home/mercleDev/codebase/build/mdai_system
```

### Service Keeps Restarting
```bash
# Check why it's failing
sudo journalctl -u mdai-system -n 100

# Check application logs
tail -100 /tmp/mdai/rdk.log
```

### Permission Issues
```bash
# Check USB device permissions
ls -la /dev/ttyACM*

# Check if user has access
groups | grep dialout
```

## Next Steps After Installation

1. Test service start/stop/restart
2. Verify logs are working
3. Test auto-restart (kill process, verify it restarts)
4. Enable auto-start on boot if desired
5. Update any scripts that manually start mdai_system to use systemctl instead

