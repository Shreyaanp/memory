#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════════
# Rebuild and Restart MDAI System
# ═══════════════════════════════════════════════════════════════════════════════
# This script:
# 1. Builds the C++ application
# 2. Kills the running mdai_system process
# 3. Starts the new build
# ═══════════════════════════════════════════════════════════════════════════════

set -e

PROJECT_ROOT="/home/mercleDev/codebase"
BUILD_DIR="$PROJECT_ROOT/build"
BINARY_NAME="mdai_system"
LOG_FILE="/home/mercleDev/mdai_logs/rdk.log"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}"
echo "╔══════════════════════════════════════════════════════════╗"
echo "║       MDAI System - Rebuild and Restart                  ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# ==============================================================================
# Step 1: Build
# ==============================================================================
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}Step 1: Building mdai_system${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

# Create build directory
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Run CMake
echo -e "${BLUE}🔨 Running CMake...${NC}"
cmake .. -DCMAKE_BUILD_TYPE=Release || {
    echo -e "${RED}❌ CMake failed${NC}"
    exit 1
}

# Build
echo -e "${BLUE}🔨 Compiling...${NC}"
make -j$(nproc) || {
    echo -e "${RED}❌ Build failed${NC}"
    exit 1
}

echo -e "${GREEN}✅ Build complete${NC}"
echo ""

# ==============================================================================
# Step 2: Kill ALL existing mdai_system instances
# ==============================================================================
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}Step 2: Stopping ALL mdai_system instances${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

# First, stop any systemd service that might auto-restart it
echo -e "${BLUE}🛑 Stopping systemd service (if any)...${NC}"
sudo systemctl stop mdai-system 2>/dev/null || true
sudo systemctl disable mdai-system 2>/dev/null || true

# Kill ALL processes matching mdai_system (handles multiple instances)
if pgrep -f mdai_system > /dev/null; then
    echo -e "${BLUE}🛑 Killing all mdai_system processes...${NC}"
    
    # Show what we're killing
    echo -e "${YELLOW}Found processes:${NC}"
    ps aux | grep mdai_system | grep -v grep || true
    
    # Kill all instances forcefully
    sudo pkill -9 -f mdai_system 2>/dev/null || true
    sleep 2
    
    # Double-check and force kill any remaining
    if pgrep -f mdai_system > /dev/null; then
        echo -e "${YELLOW}⚠️  Some processes still running, force killing...${NC}"
        sudo killall -9 mdai_system 2>/dev/null || true
        sleep 1
    fi
    
    # Final verification
    if pgrep -f mdai_system > /dev/null; then
        echo -e "${RED}❌ WARNING: Could not stop all mdai_system processes${NC}"
        ps aux | grep mdai_system | grep -v grep
    else
        echo -e "${GREEN}✅ All mdai_system processes stopped${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  No mdai_system processes running${NC}"
fi
echo ""

# ==============================================================================
# Step 3: Start new build
# ==============================================================================
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}Step 3: Starting new mdai_system${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

cd "$PROJECT_ROOT"

# Check if binary exists
if [ ! -f "./build/$BINARY_NAME" ]; then
    echo -e "${RED}❌ Binary not found: ./build/$BINARY_NAME${NC}"
    exit 1
fi

# Ensure log directory exists
mkdir -p "$(dirname $LOG_FILE)"

# Start in background
echo -e "${BLUE}🚀 Starting mdai_system in background...${NC}"
nohup ./build/mdai_system > "$LOG_FILE" 2>&1 &
sleep 3

# Verify it's running
if pgrep -f mdai_system > /dev/null; then
    echo -e "${GREEN}✅ mdai_system started successfully${NC}"
    echo ""
    echo -e "${BLUE}Process info:${NC}"
    ps aux | grep mdai_system | grep -v grep
    echo ""
    echo -e "${BLUE}Log file: ${LOG_FILE}${NC}"
    echo -e "${BLUE}View logs: tail -f ${LOG_FILE}${NC}"
else
    echo -e "${RED}❌ Failed to start mdai_system${NC}"
    echo -e "${YELLOW}Check logs: ${LOG_FILE}${NC}"
    tail -20 "$LOG_FILE"
    exit 1
fi

echo ""
echo -e "${GREEN}"
echo "╔══════════════════════════════════════════════════════════╗"
echo "║              🎉 RESTART COMPLETE! 🎉                    ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo -e "${NC}"

