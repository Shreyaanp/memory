#!/bin/bash

# ═══════════════════════════════════════════════════════════════════════════════
# Deploy Complete RDK System with Device Registration
# ═══════════════════════════════════════════════════════════════════════════════
# This script deploys the complete, production-ready system to an RDK
# Includes: WiFi management, boot sequence, SSH messages, Tailscale, DNS monitoring
#           + Device Registration for secure challenge-response authentication
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
echo "  Deploy Production-Ready RDK System + Device Registration"
echo "══════════════════════════════════════════════════════════════════════════════"
echo ""

# Server URL (EC2 instance)
SERVER_URL="https://mdai.mercle.ai"

# Check if we have the required files
REQUIRED_FILES=(
    "wifi-manager.sh"
    "rdk-boot-sequence.sh"
    "rdk-ssh-login-message.sh"
    "permanent-dns-fix.sh"
    "register_device.py"
)

for file in "${REQUIRED_FILES[@]}"; do
    if [ ! -f "$file" ]; then
        error "Required file missing: $file"
        echo "Please ensure all deployment files are in the current directory."
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
info "[1/11] Installing dependencies on RDK..."
sshpass -p "$RDK_PASS" ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no $RDK_USER@$RDK_IP << EOF
echo "$RDK_PASS" | sudo -S bash << 'SUDO_EOF'
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
# STEP 2: Copy all scripts to RDK
# ═══════════════════════════════════════════════════════════════════════════════
info "[2/11] Copying scripts to RDK..."
sshpass -p "$RDK_PASS" scp -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no \
    wifi-manager.sh \
    rdk-boot-sequence.sh \
    rdk-ssh-login-message.sh \
    permanent-dns-fix.sh \
    register_device.py \
    $RDK_USER@$RDK_IP:/tmp/
success "Scripts copied"

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 3: Install WiFi Manager
# ═══════════════════════════════════════════════════════════════════════════════
info "[3/11] Installing WiFi manager..."
sshpass -p "$RDK_PASS" ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no $RDK_USER@$RDK_IP << EOF
echo "$RDK_PASS" | sudo -S bash << 'SUDO_EOF'
chmod +x /tmp/wifi-manager.sh
mv /tmp/wifi-manager.sh /usr/local/bin/rdk-wifi-manager.sh
echo "WiFi manager installed"
SUDO_EOF
EOF
success "WiFi manager installed"

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 4: Install Boot Sequence
# ═══════════════════════════════════════════════════════════════════════════════
info "[4/11] Installing boot sequence..."
sshpass -p "$RDK_PASS" ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no $RDK_USER@$RDK_IP << EOF
echo "$RDK_PASS" | sudo -S bash << 'SUDO_EOF'
# Install boot script
chmod +x /tmp/rdk-boot-sequence.sh
mv /tmp/rdk-boot-sequence.sh /usr/local/bin/rdk-boot-sequence.sh

# Create systemd service
cat > /etc/systemd/system/rdk-boot-sequence.service << 'SERVICE_EOF'
[Unit]
Description=RDK Boot Sequence Manager
After=network-online.target
Wants=network-online.target
Before=tailscaled.service

[Service]
Type=oneshot
ExecStart=/usr/local/bin/rdk-boot-sequence.sh
RemainAfterExit=yes
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
SERVICE_EOF

# Enable service
systemctl daemon-reload
systemctl enable rdk-boot-sequence.service
echo "Boot sequence installed"
SUDO_EOF
EOF
success "Boot sequence installed"

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 5: Install SSH Login Message
# ═══════════════════════════════════════════════════════════════════════════════
info "[5/11] Installing SSH login message..."
sshpass -p "$RDK_PASS" ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no $RDK_USER@$RDK_IP << EOF
echo "$RDK_PASS" | sudo -S bash << 'SUDO_EOF'
# Install login message script
chmod +x /tmp/rdk-ssh-login-message.sh
mv /tmp/rdk-ssh-login-message.sh /usr/local/bin/rdk-ssh-login-message.sh

# Add to profile
if ! grep -q "rdk-ssh-login-message.sh" /etc/profile; then
    echo "" >> /etc/profile
    echo "# RDK Boot Status Message" >> /etc/profile
    echo "/usr/local/bin/rdk-ssh-login-message.sh 2>/dev/null || true" >> /etc/profile
fi

echo "SSH login message installed"
SUDO_EOF
EOF
success "SSH login message installed"

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 6: Apply DNS Fix
# ═══════════════════════════════════════════════════════════════════════════════
info "[6/11] Applying permanent DNS fix..."
sshpass -p "$RDK_PASS" ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no $RDK_USER@$RDK_IP << EOF
echo "$RDK_PASS" | sudo -S bash /tmp/permanent-dns-fix.sh > /tmp/dns-fix-output.log 2>&1
cat /tmp/dns-fix-output.log | tail -20
EOF
success "DNS fix applied"

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 7: Register device with server (SECURITY CRITICAL)
# ═══════════════════════════════════════════════════════════════════════════════
if [ "$SKIP_REGISTRATION" = false ]; then
    info "[7/11] Registering device with server (generating RSA keys)..."
    sshpass -p "$RDK_PASS" ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no $RDK_USER@$RDK_IP << EOF
echo "$RDK_PASS" | sudo -S python3 /tmp/register_device.py $SERVER_URL
EOF
    success "Device registered with server"
else
    warning "[7/11] Skipping device registration (no internet)"
fi

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 8: Verify Security Installation
# ═══════════════════════════════════════════════════════════════════════════════
info "[8/11] Verifying security installation..."

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
else
    echo "  ❌ Configuration: Not found"
fi

echo ""
EOF

success "Security verification complete"

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 9: Configure WiFi Networks
# ═══════════════════════════════════════════════════════════════════════════════
echo ""
info "[9/11] Configure WiFi networks..."
echo ""
read -p "Do you want to add WiFi networks now? (y/n): " ADD_WIFI

if [ "$ADD_WIFI" == "y" ]; then
    while true; do
        echo ""
        read -p "Enter WiFi SSID (or 'done' to finish): " WIFI_SSID
        if [ "$WIFI_SSID" == "done" ]; then
            break
        fi
        
        read -sp "Enter WiFi Password: " WIFI_PASSWORD
        echo ""
        read -p "Enter priority (1-100, higher = preferred): " WIFI_PRIORITY
        
        # Add network to RDK
        sshpass -p "$RDK_PASS" ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no $RDK_USER@$RDK_IP << NETWORK_EOF
echo "$RDK_PASS" | sudo -S bash << 'SUDO_EOF'
# Create config directory
mkdir -p /etc/rdk-wifi-networks
chmod 700 /etc/rdk-wifi-networks

# Store network configuration
cat > "/etc/rdk-wifi-networks/${WIFI_SSID}.conf" << 'CONF_EOF'
SSID="$WIFI_SSID"
PASSWORD="$WIFI_PASSWORD"
PRIORITY=$WIFI_PRIORITY
ADDED_DATE="\$(date '+%Y-%m-%d %H:%M:%S')"
CONF_EOF

chmod 600 "/etc/rdk-wifi-networks/${WIFI_SSID}.conf"

# Add to NetworkManager
nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASSWORD" || true
nmcli connection modify "$WIFI_SSID" connection.autoconnect-priority "$WIFI_PRIORITY" || true

echo "Network $WIFI_SSID added"
SUDO_EOF
NETWORK_EOF
        success "WiFi network '$WIFI_SSID' added (Priority: $WIFI_PRIORITY)"
    done
fi

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 10: Verify Complete Installation
# ═══════════════════════════════════════════════════════════════════════════════
info "[10/11] Verifying complete installation..."

sshpass -p "$RDK_PASS" ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no $RDK_USER@$RDK_IP << 'EOF'
echo ""
echo "Installed Components:"
echo "─────────────────────────────────────────────────────────────"
ls -lh /usr/local/bin/rdk-* 2>/dev/null | awk '{print "  ✅ " $9}'

echo ""
echo "Systemd Services:"
echo "─────────────────────────────────────────────────────────────"
systemctl list-unit-files | grep -E "rdk-boot|tailscale-dns-monitor" | awk '{print "  ✅ " $1 " - " $2}'

echo ""
echo "WiFi Networks Configured:"
echo "─────────────────────────────────────────────────────────────"
NETWORK_COUNT=$(ls /etc/rdk-wifi-networks/*.conf 2>/dev/null | wc -l)
echo "  $NETWORK_COUNT network(s) configured"

echo ""
echo "Device Information:"
echo "─────────────────────────────────────────────────────────────"
if [ -f "/etc/mdai/device_config.json" ]; then
    cat /etc/mdai/device_config.json | python3 -m json.tool
else
    echo "  ⚠️  Device not registered"
fi
echo ""
EOF

success "Installation verified"

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 11: Display device information
# ═══════════════════════════════════════════════════════════════════════════════
info "[11/11] Retrieving device information..."

DEVICE_INFO=$(sshpass -p "$RDK_PASS" ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no $RDK_USER@$RDK_IP 'cat /etc/mdai/device_config.json 2>/dev/null || echo "{}"')

echo ""
echo "══════════════════════════════════════════════════════════════════════════════"
echo "  ✅ Deployment Complete!"
echo "══════════════════════════════════════════════════════════════════════════════"
echo ""

# Display device info nicely
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
fi

info "Next Steps:"
echo "  1. Reboot the RDK to test boot sequence:"
echo "     ssh $RDK_USER@$RDK_IP 'sudo reboot'"
echo ""
echo "  2. After reboot, test MagicDNS SSH:"
echo "     ssh $RDK_USER@rdk-dev-001"
echo ""
echo "  3. Open admin portal to provision WiFi securely:"
echo "     $SERVER_URL"
echo ""
echo "  4. To manage WiFi networks manually on RDK:"
echo "     ssh $RDK_USER@$RDK_IP 'sudo rdk-wifi-manager.sh'"
echo ""
echo "══════════════════════════════════════════════════════════════════════════════"
echo ""

exit 0


