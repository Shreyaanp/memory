#!/bin/bash
# Install MDai System Service

set -e

echo "Installing MDai System Service..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root: sudo $0"
    exit 1
fi

# Stop any existing service
systemctl stop mdai-system.service 2>/dev/null || true
systemctl disable mdai-system.service 2>/dev/null || true

# Copy service file
cp /home/mercleDev/codebase/deployment/mdai-system.service /etc/systemd/system/
chmod 644 /etc/systemd/system/mdai-system.service

# Create log directory
mkdir -p /var/log/mdai
chown root:root /var/log/mdai

# Ensure device config exists
if [ ! -f "/opt/mdai/device_config.json" ]; then
    mkdir -p /opt/mdai
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

# Set permissions for serial device (permanent)
cat > /etc/udev/rules.d/99-mdai-serial.rules <<EOF
# LilyGo ESP32 Display
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", MODE="0666", GROUP="dialout"
EOF
udevadm control --reload-rules
udevadm trigger

# Reload systemd
systemctl daemon-reload

# Enable and start service
systemctl enable mdai-system.service
systemctl start mdai-system.service

echo ""
echo "Service installed and started!"
echo "Check status: sudo systemctl status mdai-system"
echo "View logs: sudo journalctl -u mdai-system -f"
