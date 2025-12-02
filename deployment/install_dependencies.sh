#!/bin/bash
# ==============================================================================
# MDAI System - Complete Dependency Installation Script
# ==============================================================================
# This script installs ALL required dependencies for running mdai_system
# on a new Ubuntu 22.04 ARM64 device.
#
# Usage: sudo ./install_dependencies.sh
# ==============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║     MDAI System - Dependency Installation Script             ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}ERROR: Please run as root: sudo ./install_dependencies.sh${NC}"
    exit 1
fi

# Check architecture
ARCH=$(uname -m)
if [ "$ARCH" != "aarch64" ]; then
    echo -e "${YELLOW}WARNING: This script is optimized for ARM64 (aarch64).${NC}"
    echo -e "${YELLOW}Current architecture: $ARCH${NC}"
    read -p "Continue anyway? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# ==============================================================================
# SECTION 1: System Update
# ==============================================================================
echo -e "${YELLOW}[1/8] Updating system packages...${NC}"
apt update
apt upgrade -y

# ==============================================================================
# SECTION 2: Build Tools
# ==============================================================================
echo -e "${YELLOW}[2/8] Installing build tools...${NC}"
apt install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    wget \
    curl \
    unzip

# ==============================================================================
# SECTION 3: Core Libraries
# ==============================================================================
echo -e "${YELLOW}[3/8] Installing core libraries...${NC}"

# OpenCV (full suite)
apt install -y \
    libopencv-dev \
    libopencv-contrib-dev

# OpenSSL
apt install -y \
    libssl-dev \
    openssl

# Threading (TBB)
apt install -y \
    libtbb-dev

# Abseil (for MediaPipe compatibility)
apt install -y \
    libabsl-dev

# USB support (for RealSense)
apt install -y \
    libusb-1.0-0-dev

# Protocol Buffers
apt install -y \
    libprotobuf-dev \
    protobuf-compiler

# CURL
apt install -y \
    libcurl4-openssl-dev

# JSON parsing
apt install -y \
    nlohmann-json3-dev

# ==============================================================================
# SECTION 4: Camera & Graphics Libraries
# ==============================================================================
echo -e "${YELLOW}[4/8] Installing camera and graphics libraries...${NC}"

# RealSense dependencies
apt install -y \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libudev-dev

# GTK (for OpenCV GUI)
apt install -y \
    libgtk-3-dev

# GStreamer
apt install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad

# FFmpeg
apt install -y \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev

# Image libraries
apt install -y \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev

# ==============================================================================
# SECTION 5: Network Services
# ==============================================================================
echo -e "${YELLOW}[5/8] Installing network services...${NC}"

apt install -y \
    network-manager \
    wpasupplicant \
    openssh-server \
    ntp

# Enable network services
systemctl enable NetworkManager
systemctl enable wpa_supplicant
systemctl enable ssh
systemctl enable ntp

# ==============================================================================
# SECTION 6: System Services
# ==============================================================================
echo -e "${YELLOW}[6/8] Installing system services...${NC}"

apt install -y \
    rtkit \
    policykit-1 \
    watchdog \
    usbutils

# ==============================================================================
# SECTION 7: Tailscale Installation
# ==============================================================================
echo -e "${YELLOW}[7/8] Installing Tailscale...${NC}"

if ! command -v tailscale &> /dev/null; then
    curl -fsSL https://tailscale.com/install.sh | sh
    systemctl enable tailscaled
    echo -e "${GREEN}✓ Tailscale installed${NC}"
    echo -e "${YELLOW}  Run 'sudo tailscale up --authkey=<key>' to join network${NC}"
else
    echo -e "${GREEN}✓ Tailscale already installed${NC}"
fi

# ==============================================================================
# SECTION 8: Library Cache Update
# ==============================================================================
echo -e "${YELLOW}[8/8] Updating library cache...${NC}"

# Add common library paths
echo "/usr/local/lib" | tee /etc/ld.so.conf.d/local.conf > /dev/null
ldconfig

# ==============================================================================
# SUMMARY
# ==============================================================================
echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║     Dependency Installation Complete!                        ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Installed Components:${NC}"
echo "  ✓ Build tools (cmake, gcc, g++)"
echo "  ✓ OpenCV 4.5+"
echo "  ✓ OpenSSL 3"
echo "  ✓ Threading (TBB)"
echo "  ✓ Abseil"
echo "  ✓ USB libraries"
echo "  ✓ Protocol Buffers"
echo "  ✓ CURL"
echo "  ✓ JSON (nlohmann)"
echo "  ✓ GStreamer"
echo "  ✓ FFmpeg"
echo "  ✓ Network services"
echo "  ✓ Tailscale"
echo ""
echo -e "${YELLOW}Manual Steps Required:${NC}"
echo "  1. Install Intel RealSense SDK from source"
echo "  2. Copy MediaPipe native library (mediapipe_arm64_final/)"
echo "  3. Build quirc library from source"
echo "  4. Copy mdai_system binary and libraries"
echo "  5. Run install.sh to set up services"
echo ""
echo -e "${BLUE}See NEW_DEVICE_SETUP.md for detailed instructions.${NC}"
echo ""

