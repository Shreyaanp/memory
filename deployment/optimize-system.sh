#!/bin/bash
# MDAI System Optimizer
# Disables unnecessary services for dedicated single-task device
# Run as root: sudo ./optimize-system.sh

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=========================================="
echo "   MDAI System Optimizer"
echo "   Disabling unnecessary services"
echo -e "==========================================${NC}"

if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Please run as root: sudo ./optimize-system.sh${NC}"
    exit 1
fi

# ============================================
# SERVICES TO DISABLE
# ============================================
# These are not needed for dedicated RDK device

DISABLE_SERVICES=(
    # Docker (not needed)
    "docker.service"
    "docker.socket"
    "containerd.service"
    
    # Remote desktop (use SSH instead)
    "xrdp.service"
    "xrdp-sesman.service"
    
    # Deferred execution (not needed)
    "atd.service"
    
    # Sensors (not needed for our use case)
    "iio-sensor-proxy.service"
    
    # Bluetooth (if not used)
    # "bluetooth.service"
    
    # Printing (not needed)
    "cups.service"
    "cups-browsed.service"
    
    # Avahi/mDNS (not needed)
    "avahi-daemon.service"
    
    # ModemManager (not needed)
    "ModemManager.service"
    
    # Snapd (not needed)
    "snapd.service"
    "snapd.socket"
    "snapd.seeded.service"
)

# ============================================
# SERVICES TO KEEP (DO NOT DISABLE)
# ============================================
# NetworkManager.service    - WiFi
# wpa_supplicant.service    - WiFi auth
# tailscaled.service        - Tailscale VPN
# ssh.service               - SSH access
# systemd-*                 - System essentials
# dbus.service              - System bus
# rtkit-daemon.service      - Real-time scheduling
# watchdog.service          - Hardware watchdog

echo -e "${YELLOW}Disabling unnecessary services...${NC}"
echo ""

for service in "${DISABLE_SERVICES[@]}"; do
    if systemctl list-unit-files | grep -q "^$service"; then
        echo -n "  Disabling $service... "
        systemctl stop "$service" 2>/dev/null || true
        systemctl disable "$service" 2>/dev/null || true
        systemctl mask "$service" 2>/dev/null || true
        echo -e "${GREEN}done${NC}"
    else
        echo -e "  $service - ${BLUE}not installed${NC}"
    fi
done

echo ""
echo -e "${YELLOW}Cleaning up...${NC}"

# Disable apt auto-updates (can cause CPU spikes)
systemctl disable apt-daily.timer 2>/dev/null || true
systemctl disable apt-daily-upgrade.timer 2>/dev/null || true

# Disable man-db auto-update
systemctl disable man-db.timer 2>/dev/null || true

echo ""
echo -e "${GREEN}=========================================="
echo "   Optimization Complete!"
echo -e "==========================================${NC}"
echo ""
echo "Services still running:"
systemctl list-units --type=service --state=running --no-pager | grep -E "loaded active" | wc -l
echo ""
echo -e "${YELLOW}Reboot recommended to apply all changes.${NC}"



