#!/bin/bash
# MDAI System Status
# Quick status check for all MDAI services

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=========================================="
echo "   MDAI System Status"
echo -e "==========================================${NC}"
echo ""

# System info
echo -e "${YELLOW}System:${NC}"
echo "  Hostname: $(hostname)"
echo "  Uptime:   $(uptime -p)"
echo "  Load:     $(cat /proc/loadavg | cut -d' ' -f1-3)"
echo ""

# Memory
echo -e "${YELLOW}Memory:${NC}"
free -h | grep -E "Mem:|Swap:"
echo ""

# CPU
echo -e "${YELLOW}CPU:${NC}"
echo "  Cores: $(nproc)"
echo "  Temp:  $(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null | awk '{print $1/1000"°C"}' || echo 'N/A')"
echo ""

# Services
echo -e "${YELLOW}Critical Services:${NC}"

check_service() {
    local name=$1
    local display=$2
    if systemctl is-active --quiet "$name"; then
        echo -e "  ${GREEN}✓${NC} $display"
    else
        echo -e "  ${RED}✗${NC} $display ($(systemctl is-active $name))"
    fi
}

check_service "mdai-system" "mdai-system (main)"
check_service "mdai-watchdog" "mdai-watchdog"
check_service "NetworkManager" "NetworkManager (WiFi)"
check_service "wpa_supplicant" "wpa_supplicant (WiFi auth)"
check_service "tailscaled" "tailscaled (VPN)"
check_service "ssh" "ssh (remote access)"

echo ""

# Network
echo -e "${YELLOW}Network:${NC}"
echo "  WiFi (wlan0):   $(ip -4 addr show wlan0 2>/dev/null | grep -oP 'inet \K[\d.]+' || echo 'not connected')"
echo "  Tailscale:      $(ip -4 addr show tailscale0 2>/dev/null | grep -oP 'inet \K[\d.]+' || echo 'not connected')"
echo "  Ethernet:       $(ip -4 addr show eth0 2>/dev/null | grep -oP 'inet \K[\d.]+' || echo 'not connected')"
echo ""

# MDAI Process
echo -e "${YELLOW}MDAI Process:${NC}"
if pgrep -f mdai_system > /dev/null; then
    PID=$(pgrep -f mdai_system | head -1)
    echo -e "  Status: ${GREEN}RUNNING${NC} (PID: $PID)"
    echo "  Memory: $(ps -p $PID -o rss= 2>/dev/null | awk '{print int($1/1024)"MB"}' || echo 'N/A')"
    echo "  CPU:    $(ps -p $PID -o %cpu= 2>/dev/null || echo 'N/A')%"
    echo "  Nice:   $(ps -p $PID -o ni= 2>/dev/null || echo 'N/A')"
else
    echo -e "  Status: ${RED}NOT RUNNING${NC}"
fi
echo ""

# Logs (last 5 lines)
echo -e "${YELLOW}Recent Logs:${NC}"
if [ -f /home/mercleDev/mdai_logs/system.log ]; then
    tail -5 /home/mercleDev/mdai_logs/system.log 2>/dev/null | sed 's/^/  /'
else
    echo "  No logs found"
fi
echo ""
echo -e "${BLUE}==========================================${NC}"




