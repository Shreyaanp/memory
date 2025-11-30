#!/bin/bash
# MDAI System Installation Script
# Run as root: sudo ./install.sh

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=========================================="
echo "   MDAI System Installation"
echo -e "==========================================${NC}"

# Check root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Please run as root: sudo ./install.sh${NC}"
    exit 1
fi

DEPLOY_DIR="/home/mercleDev/codebase/deployment"
BUILD_DIR="/home/mercleDev/codebase/build"
LOG_DIR="/home/mercleDev/mdai_logs"

# Verify binary exists
if [ ! -f "$BUILD_DIR/mdai_system" ]; then
    echo -e "${RED}ERROR: mdai_system binary not found at $BUILD_DIR/mdai_system${NC}"
    echo "Please build the project first: cd $BUILD_DIR && make -j7"
    exit 1
fi

echo -e "${GREEN}✓ Binary found: $BUILD_DIR/mdai_system${NC}"

# Create directories
echo -e "${YELLOW}Creating directories...${NC}"
mkdir -p "$LOG_DIR"
chown mercleDev:mercleDev "$LOG_DIR"

# Make watchdog executable
chmod +x "$DEPLOY_DIR/watchdog.sh"

# Stop existing services if running
echo -e "${YELLOW}Stopping any existing services...${NC}"
systemctl stop mdai-system 2>/dev/null || true
systemctl stop mdai-watchdog 2>/dev/null || true

# Remove old service files
rm -f /etc/systemd/system/mdai-system.service
rm -f /etc/systemd/system/mdai-watchdog.service

# Install new service files
echo -e "${YELLOW}Installing systemd services...${NC}"
cp "$DEPLOY_DIR/mdai-system.service" /etc/systemd/system/
cp "$DEPLOY_DIR/mdai-watchdog.service" /etc/systemd/system/

# Set permissions
chmod 644 /etc/systemd/system/mdai-system.service
chmod 644 /etc/systemd/system/mdai-watchdog.service

# Reload systemd
echo -e "${YELLOW}Reloading systemd...${NC}"
systemctl daemon-reload

# Enable services (start on boot)
echo -e "${YELLOW}Enabling services...${NC}"
systemctl enable mdai-system
systemctl enable mdai-watchdog

echo -e "${GREEN}=========================================="
echo "   Installation Complete!"
echo -e "==========================================${NC}"
echo ""
echo "Services installed but NOT started."
echo ""
echo "Commands:"
echo -e "  ${BLUE}Start now:${NC}     sudo systemctl start mdai-system"
echo -e "  ${BLUE}Check status:${NC}  sudo systemctl status mdai-system"
echo -e "  ${BLUE}View logs:${NC}     tail -f $LOG_DIR/system.log"
echo -e "  ${BLUE}Stop:${NC}          sudo systemctl stop mdai-system"
echo ""
echo -e "${YELLOW}On next reboot, mdai-system will start automatically.${NC}"




