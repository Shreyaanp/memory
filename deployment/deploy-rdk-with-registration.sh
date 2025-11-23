#!/bin/bash

# ═══════════════════════════════════════════════════════════════════════════════
# Deploy Complete RDK System with Device Registration
# ═══════════════════════════════════════════════════════════════════════════════
# This script deploys the complete system including device registration for
# secure challenge-response authentication with EC2 server
# ═══════════════════════════════════════════════════════════════════════════════

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

error() {
    echo -e "${RED}❌ $1${NC}"
}

success() {
    echo -e "${GREEN}✅ $1${NC}"
}

warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

echo ""
echo "══════════════════════════════════════════════════════════════════════════════"
echo "  Deploy RDK System with Secure Device Registration"
echo "══════════════════════════════════════════════════════════════════════════════"
echo ""

# Server URL (EC2 instance)
SERVER_URL="https://mdai.mercle.ai"

# Check if we have the required files
REQUIRED_FILES=(
    "register_device.py"
)

for file in "${REQUIRED_FILES[@]}"; do
    if [ ! -f "$file" ]; then
        error "Required file missing: $file"
        exit 1
    fi
done

# Get RDK connection details
echo "RDK Connection Details:"
echo "─────────────────────────────────────────────────────────────────────────────"
read -p "RDK IP address (local): " RDK_IP
read -p "SSH Username [mercleDev]: " RDK_USER
RDK_USER=${RDK_USER:-mercleDev}
read -sp "SSH Password: " RDK_PASS
echo ""
echo ""

# Test connection
info "Testing connection to RDK..."
if ! sshpass -p "$RDK_PASS" ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no -o ConnectTimeout=10 -o StrictHostKeyChecking=no $RDK_USER@$RDK_IP "echo 'connected'" &>/dev/null; then
    error "Cannot connect to RDK at $RDK_IP"
    exit 1
fi
success "Connected to RDK"

# Check if RDK has internet
info "Checking internet connectivity on RDK..."
if ! sshpass -p "$RDK_PASS" ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no $RDK_USER@$RDK_IP "curl -s --connect-timeout 5 https://mdai.mercle.ai/api/health &>/dev/null"; then
    warning "RDK does not have internet access"
    warning "Device registration will be skipped"
    warning "You must register the device manually later"
    SKIP_REGISTRATION=true
else
    success "RDK has internet access"
    SKIP_REGISTRATION=false
fi

echo ""
echo "══════════════════════════════════════════════════════════════════════════════"
echo "  Deployment Steps"
echo "══════════════════════════════════════════════════════════════════════════════"
echo ""

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 1: Install dependencies
# ═══════════════════════════════════════════════════════════════════════════════
info "[1/5] Installing dependencies on RDK..."
sshpass -p "$RDK_PASS" ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no $RDK_USER@$RDK_IP << 'EOF'
echo "$1" | sudo -S bash << 'SUDO_EOF'
# Install Python and required packages
apt-get update -qq
apt-get install -y -qq python3 python3-pip openssl curl libcurl4-openssl-dev

# Install Python dependencies
pip3 install -q requests

echo "Dependencies installed"
SUDO_EOF
EOF
success "Dependencies installed"

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 2: Copy registration script to RDK
# ═══════════════════════════════════════════════════════════════════════════════
info "[2/5] Copying registration script to RDK..."
sshpass -p "$RDK_PASS" scp -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no \
    register_device.py \
    $RDK_USER@$RDK_IP:/tmp/
success "Script copied"

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 3: Register device with server
# ═══════════════════════════════════════════════════════════════════════════════
if [ "$SKIP_REGISTRATION" = false ]; then
    info "[3/5] Registering device with server..."
    sshpass -p "$RDK_PASS" ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no $RDK_USER@$RDK_IP << EOF
echo "$RDK_PASS" | sudo -S python3 /tmp/register_device.py $SERVER_URL
EOF
    success "Device registered"
else
    warning "[3/5] Skipping device registration (no internet)"
fi

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 4: Verify installation
# ═══════════════════════════════════════════════════════════════════════════════
info "[4/5] Verifying installation..."

sshpass -p "$RDK_PASS" ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no $RDK_USER@$RDK_IP << 'EOF'
echo ""
echo "Device Security Status:"
echo "─────────────────────────────────────────────────────────────"

if [ -f "/etc/mdai/device_private.pem" ]; then
    echo "  ✅ Private key: $(ls -lh /etc/mdai/device_private.pem | awk '{print $9 " (" $5 ")"}')"
else
    echo "  ❌ Private key: Not found"
fi

if [ -f "/etc/mdai/device_public.pem" ]; then
    echo "  ✅ Public key:  $(ls -lh /etc/mdai/device_public.pem | awk '{print $9 " (" $5 ")"}')"
else
    echo "  ❌ Public key: Not found"
fi

if [ -f "/etc/mdai/device_config.json" ]; then
    echo "  ✅ Configuration: /etc/mdai/device_config.json"
    echo ""
    echo "Device Information:"
    echo "─────────────────────────────────────────────────────────────"
    cat /etc/mdai/device_config.json | python3 -m json.tool
else
    echo "  ❌ Configuration: Not found"
fi

echo ""
EOF

success "Installation verified"

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 5: Display device information
# ═══════════════════════════════════════════════════════════════════════════════
info "[5/5] Retrieving device information..."

DEVICE_INFO=$(sshpass -p "$RDK_PASS" ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no $RDK_USER@$RDK_IP 'cat /etc/mdai/device_config.json 2>/dev/null || echo "{}"')

echo ""
echo "══════════════════════════════════════════════════════════════════════════════"
echo "  ✅ Deployment Complete!"
echo "══════════════════════════════════════════════════════════════════════════════"
echo ""
echo "$DEVICE_INFO" | python3 -c "
import sys, json
try:
    data = json.load(sys.stdin)
    if data.get('device_id'):
        print('Device ID:       ' + data['device_id'])
        print('Hardware ID:     ' + data['hardware_id'])
        print('Hostname:        ' + data.get('hostname', 'unknown'))
        print('Registered:      ' + data.get('registered_at', 'unknown'))
    else:
        print('⚠️  Device not registered')
except:
    print('⚠️  Could not read device info')
"
echo ""

if [ "$SKIP_REGISTRATION" = true ]; then
    warning "Device was not registered due to network issues"
    echo ""
    info "To register manually later:"
    echo "  1. Ensure RDK has internet access"
    echo "  2. SSH to RDK: ssh $RDK_USER@$RDK_IP"
    echo "  3. Run: sudo python3 /tmp/register_device.py $SERVER_URL"
    echo ""
else
    info "Next Steps:"
    echo "  1. Open admin portal: $SERVER_URL"
    echo "  2. Select this device from the list"
    echo "  3. Generate WiFi provisioning QR code"
    echo "  4. Scan QR with device to provision WiFi"
    echo ""
fi

echo "══════════════════════════════════════════════════════════════════════════════"
echo ""

exit 0


