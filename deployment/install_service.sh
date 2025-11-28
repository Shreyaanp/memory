#!/bin/bash
# Install MDai System Service - High Priority, Single Instance
#
# This service:
#   - Starts at boot with HIGHEST priority
#   - GUARANTEES only ONE instance ever runs
#   - Uses 100% of system resources (no limits)
#   - Auto-restarts on any error
#   - Reboots system if it keeps failing

set -e

echo "==========================================="
echo " MDai System Service Installer"
echo "==========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "ERROR: Please run as root: sudo $0"
    exit 1
fi

# Step 1: KILL ALL mdai processes - no survivors
echo "[1/7] KILLING all mdai_system processes..."
pkill -9 -x mdai_system 2>/dev/null || true
pkill -9 -f "mdai_system" 2>/dev/null || true
killall -9 mdai_system 2>/dev/null || true
sleep 2

# Verify nothing is running
RUNNING=$(pgrep -c mdai_system 2>/dev/null | head -1 || echo "0")
RUNNING=${RUNNING:-0}
if [ "$RUNNING" -gt 0 ] 2>/dev/null; then
    echo "WARNING: Still found $RUNNING mdai_system processes, force killing..."
    pkill -9 mdai_system 2>/dev/null || true
    sleep 1
fi

# Step 2: Stop and disable any existing service
echo "[2/7] Stopping existing service..."
systemctl stop mdai-system.service 2>/dev/null || true
systemctl disable mdai-system.service 2>/dev/null || true
systemctl reset-failed mdai-system.service 2>/dev/null || true

# Step 3: Copy service file
echo "[3/7] Installing service file..."
cp /home/mercleDev/codebase/deployment/mdai-system.service /etc/systemd/system/
chmod 644 /etc/systemd/system/mdai-system.service

# Step 4: Create required directories
echo "[4/7] Creating directories..."
mkdir -p /var/log/mdai
mkdir -p /tmp/mdai
mkdir -p /opt/mdai

# Ensure device config exists
if [ ! -f "/opt/mdai/device_config.json" ]; then
    cat > /opt/mdai/device_config.json <<EOF
{
  "device_id": "dev_rdk_x5_001",
  "hardware_id": "$(cat /etc/machine-id)",
  "device_key": "placeholder",
  "created_at": "$(date -Iseconds)",
  "device_type": "RDK_X5",
  "firmware_version": "1.0.0"
}
EOF
fi

# Step 5: Set up udev rules for ESP32 serial
echo "[5/7] Setting up udev rules for ESP32 serial..."
cat > /etc/udev/rules.d/99-mdai-serial.rules <<EOF
# LilyGo ESP32 Display - auto permissions
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", MODE="0666", GROUP="dialout"
# Generic USB serial
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", MODE="0666", GROUP="dialout"
# Intel RealSense cameras
SUBSYSTEM=="usb", ATTR{idVendor}=="8086", MODE="0666"
EOF
udevadm control --reload-rules
udevadm trigger

# Step 6: Reload systemd
echo "[6/7] Reloading systemd daemon..."
systemctl daemon-reload

# Step 7: Enable and start
echo "[7/7] Enabling and starting service..."
systemctl enable mdai-system.service
systemctl start mdai-system.service

sleep 3

echo ""
echo "==========================================="
echo " Installation Complete!"
echo "==========================================="

# Final status
echo ""
systemctl status mdai-system.service --no-pager -l || true

echo ""
echo "==========================================="
echo " Verification:"
echo "==========================================="
PROC_COUNT=$(pgrep -c mdai_system 2>/dev/null || echo "0")
echo "  Running instances: $PROC_COUNT (MUST be 1)"

if [ "$PROC_COUNT" -eq 1 ]; then
    echo "  ✓ GOOD: Exactly one instance running"
elif [ "$PROC_COUNT" -eq 0 ]; then
    echo "  ✗ ERROR: No instance running!"
else
    echo "  ✗ CRITICAL: Multiple instances detected!"
fi

echo ""
echo "==========================================="
echo " Commands:"
echo "==========================================="
echo "  Status:    sudo systemctl status mdai-system"
echo "  Logs:      sudo journalctl -u mdai-system -f"
echo "  Restart:   sudo systemctl restart mdai-system"
echo "  Stop:      sudo systemctl stop mdai-system"
echo ""
