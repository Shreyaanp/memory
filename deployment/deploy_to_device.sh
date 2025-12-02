#!/bin/bash
# ==============================================================================
# MDAI System - Deploy to New Device Script
# ==============================================================================
# This script copies all necessary files to a new device via SSH/rsync.
#
# Usage: ./deploy_to_device.sh <target_ip_or_hostname> [username]
# Example: ./deploy_to_device.sh 192.168.1.100 mercleDev
# ==============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Source directory
SRC_DIR="/home/mercleDev"

# Usage check
if [ -z "$1" ]; then
    echo -e "${RED}Usage: $0 <target_ip_or_hostname> [username]${NC}"
    echo "Example: $0 192.168.1.100 mercleDev"
    exit 1
fi

TARGET_HOST=$1
TARGET_USER=${2:-"mercleDev"}
TARGET="$TARGET_USER@$TARGET_HOST"
TARGET_DIR="/home/$TARGET_USER"

echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║     MDAI System - Deploy to New Device                       ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "Target: ${GREEN}$TARGET${NC}"
echo -e "Destination: ${GREEN}$TARGET_DIR${NC}"
echo ""

# Test connection
echo -e "${YELLOW}[1/6] Testing SSH connection...${NC}"
if ! ssh -o ConnectTimeout=10 "$TARGET" "echo 'Connection successful'" 2>/dev/null; then
    echo -e "${RED}ERROR: Cannot connect to $TARGET${NC}"
    echo "Please ensure:"
    echo "  - The device is powered on and connected to network"
    echo "  - SSH is enabled on the device"
    echo "  - SSH key or password is configured"
    exit 1
fi
echo -e "${GREEN}✓ SSH connection successful${NC}"
echo ""

# Create target directories
echo -e "${YELLOW}[2/6] Creating target directories...${NC}"
ssh "$TARGET" "mkdir -p $TARGET_DIR/codebase $TARGET_DIR/mediapipe_arm64_final $TARGET_DIR/mdai_logs"
echo -e "${GREEN}✓ Directories created${NC}"
echo ""

# Copy codebase
echo -e "${YELLOW}[3/6] Copying codebase...${NC}"
rsync -avz --progress \
    --exclude='CMakeFiles/' \
    --exclude='*.o' \
    --exclude='.git/' \
    --exclude='highres/' \
    --exclude='*.raw' \
    "$SRC_DIR/codebase/" \
    "$TARGET:$TARGET_DIR/codebase/"
echo -e "${GREEN}✓ Codebase copied${NC}"
echo ""

# Copy MediaPipe native library
echo -e "${YELLOW}[4/6] Copying MediaPipe native library...${NC}"
rsync -avz --progress \
    "$SRC_DIR/mediapipe_arm64_final/" \
    "$TARGET:$TARGET_DIR/mediapipe_arm64_final/"
echo -e "${GREEN}✓ MediaPipe library copied${NC}"
echo ""

# Copy librealsense (if built locally)
echo -e "${YELLOW}[5/6] Copying librealsense...${NC}"
if [ -f "/usr/local/lib/librealsense2.so" ]; then
    # Create temp directory for library
    ssh "$TARGET" "mkdir -p /tmp/librealsense_libs"
    
    rsync -avz --progress \
        /usr/local/lib/librealsense2.so* \
        "$TARGET:/tmp/librealsense_libs/"
    
    # Move to system location (requires sudo on target)
    echo "Moving libraries to /usr/local/lib (requires sudo on target)..."
    ssh "$TARGET" "sudo mv /tmp/librealsense_libs/* /usr/local/lib/ && sudo ldconfig"
    echo -e "${GREEN}✓ librealsense copied${NC}"
else
    echo -e "${YELLOW}⚠ librealsense not found locally - skip${NC}"
fi
echo ""

# Copy quirc library
echo -e "${YELLOW}[6/6] Copying quirc library...${NC}"
if [ -f "/usr/local/lib/libquirc.a" ]; then
    rsync -avz --progress \
        /usr/local/lib/libquirc.a \
        "$TARGET:/tmp/"
    ssh "$TARGET" "sudo mv /tmp/libquirc.a /usr/local/lib/"
    echo -e "${GREEN}✓ quirc library copied${NC}"
else
    echo -e "${YELLOW}⚠ quirc library not found locally - skip${NC}"
fi
echo ""

# Make scripts executable
echo -e "${YELLOW}Setting permissions...${NC}"
ssh "$TARGET" "chmod +x $TARGET_DIR/codebase/deployment/*.sh"

# ==============================================================================
# SUMMARY
# ==============================================================================
echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║     Deployment Complete!                                     ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Files copied to $TARGET:${NC}"
echo "  ✓ $TARGET_DIR/codebase/"
echo "  ✓ $TARGET_DIR/mediapipe_arm64_final/"
echo "  ✓ /usr/local/lib/librealsense2.so*"
echo "  ✓ /usr/local/lib/libquirc.a"
echo ""
echo -e "${YELLOW}Next steps on the target device ($TARGET):${NC}"
echo ""
echo "  1. SSH to the device:"
echo -e "     ${BLUE}ssh $TARGET${NC}"
echo ""
echo "  2. Install dependencies (if not done):"
echo -e "     ${BLUE}sudo $TARGET_DIR/codebase/deployment/install_dependencies.sh${NC}"
echo ""
echo "  3. Install services:"
echo -e "     ${BLUE}cd $TARGET_DIR/codebase/deployment${NC}"
echo -e "     ${BLUE}sudo ./install.sh${NC}"
echo ""
echo "  4. Optimize system (disable unnecessary services):"
echo -e "     ${BLUE}sudo ./optimize-system.sh${NC}"
echo ""
echo "  5. Configure device:"
echo -e "     ${BLUE}sudo hostnamectl set-hostname rdk-prod-XXX${NC}"
echo -e "     ${BLUE}sudo tailscale up --authkey=<your-key>${NC}"
echo ""
echo "  6. Start the service:"
echo -e "     ${BLUE}sudo systemctl start mdai-system${NC}"
echo ""
echo "  7. Verify:"
echo -e "     ${BLUE}./status.sh${NC}"
echo ""

