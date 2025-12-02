#!/bin/bash
# ==============================================================================
# MDAI RDK Device Registration Script
# ==============================================================================
# This script runs on FIRST BOOT of a new RDK device to:
#   1. Generate a unique device identity
#   2. SSH into EC2 server and register the device
#   3. Save device credentials locally
#   4. DELETE the SSH key (one-time use only)
#
# Prerequisites:
#   - SSH key at ~/.ssh/ec2_register.pem (will be deleted after registration)
#   - Network connectivity to EC2 server
#
# Usage: sudo ./register-device.sh
# ==============================================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
EC2_HOST="${EC2_HOST:-mdai.mercle.ai}"
EC2_USER="${EC2_USER:-ubuntu}"
EC2_REGISTER_SCRIPT="${EC2_REGISTER_SCRIPT:-/home/ubuntu/ec2-server/register_device.sh}"
SSH_KEY="/home/mercleDev/.ssh/ec2_register.pem"
CONFIG_DIR="/opt/mdai"
REGISTRATION_FLAG="$CONFIG_DIR/.registered"
LOG_FILE="/home/mercleDev/mdai_logs/registration.log"

# Create log directory
mkdir -p /home/mercleDev/mdai_logs

log() {
    echo -e "$1" | tee -a "$LOG_FILE"
}

log "${BLUE}========================================${NC}"
log "${BLUE}   MDAI Device Registration${NC}"
log "${BLUE}========================================${NC}"
log "Timestamp: $(date)"

# Check if already registered
if [ -f "$REGISTRATION_FLAG" ]; then
    log "${GREEN}✓ Device already registered. Skipping.${NC}"
    log "  Config: $CONFIG_DIR/device_config.json"
    exit 0
fi

# Check SSH key exists
if [ ! -f "$SSH_KEY" ]; then
    log "${RED}ERROR: SSH key not found at $SSH_KEY${NC}"
    log "This device cannot be registered without the SSH key."
    log "Please ensure the SSH key is present in the OS image."
    exit 1
fi

# Check SSH key permissions
chmod 600 "$SSH_KEY" 2>/dev/null || true

# Create config directory
log "${YELLOW}Creating config directory...${NC}"
sudo mkdir -p "$CONFIG_DIR"
sudo chown mercleDev:mercleDev "$CONFIG_DIR"

# Generate device ID from hardware
log "${YELLOW}Generating device identity...${NC}"
HARDWARE_SERIAL=$(cat /etc/machine-id)
DEVICE_ID="rdk_$(echo -n "$HARDWARE_SERIAL" | sha256sum | cut -c1-16)"
HOSTNAME=$(hostname)

log "  Device ID: $DEVICE_ID"
log "  Hostname: $HOSTNAME"
log "  Hardware Serial: $HARDWARE_SERIAL"

# Generate RSA key pair for this device (optional, for future signature-based auth)
log "${YELLOW}Generating device key pair...${NC}"
openssl genrsa -out "$CONFIG_DIR/device_private.pem" 2048 2>/dev/null
openssl rsa -in "$CONFIG_DIR/device_private.pem" -pubout -out "$CONFIG_DIR/device_public.pem" 2>/dev/null
chmod 600 "$CONFIG_DIR/device_private.pem"
chmod 644 "$CONFIG_DIR/device_public.pem"
log "  Private key: $CONFIG_DIR/device_private.pem"
log "  Public key: $CONFIG_DIR/device_public.pem"

# Encode public key for transfer
PUBLIC_KEY_B64=$(cat "$CONFIG_DIR/device_public.pem" | base64 -w0)

# Test network connectivity
log "${YELLOW}Testing network connectivity...${NC}"
if ! ping -c 1 -W 5 8.8.8.8 &>/dev/null; then
    log "${RED}ERROR: No network connectivity${NC}"
    log "Please ensure the device is connected to the internet."
    exit 1
fi
log "  ✓ Network OK"

# Test SSH connectivity
log "${YELLOW}Testing SSH connectivity to $EC2_HOST...${NC}"
if ! ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$EC2_USER@$EC2_HOST" "echo 'SSH OK'" &>/dev/null; then
    log "${RED}ERROR: Cannot SSH to $EC2_HOST${NC}"
    log "Please check:"
    log "  - EC2 server is running"
    log "  - SSH key is authorized on EC2"
    log "  - Network allows SSH (port 22)"
    exit 1
fi
log "  ✓ SSH OK"

# Register with EC2 server
log "${YELLOW}Registering with EC2 server...${NC}"

RESULT=$(ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no "$EC2_USER@$EC2_HOST" \
    "$EC2_REGISTER_SCRIPT '$DEVICE_ID' '$HOSTNAME' '$HARDWARE_SERIAL' '$PUBLIC_KEY_B64'" 2>&1)

log "EC2 Response:"
echo "$RESULT" | tee -a "$LOG_FILE"

# Check if successful
if echo "$RESULT" | grep -q "SUCCESS"; then
    # Extract device secret
    DEVICE_SECRET=$(echo "$RESULT" | grep "DEVICE_SECRET=" | cut -d'=' -f2)
    
    if [ -z "$DEVICE_SECRET" ]; then
        log "${RED}ERROR: Failed to get device secret from EC2${NC}"
        exit 1
    fi
    
    # Get QR encryption key from EC2
    log "${YELLOW}Getting QR encryption key from EC2...${NC}"
    QR_KEY=$(ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no "$EC2_USER@$EC2_HOST" \
        "grep QR_ENCRYPTION_KEY /opt/mdai/.env | cut -d'=' -f2" 2>/dev/null)
    
    if [ -z "$QR_KEY" ]; then
        log "${RED}ERROR: Could not get QR encryption key from EC2${NC}"
        exit 1
    fi
    log "  ✓ QR key retrieved"
    
    # Save device config
    log "${YELLOW}Saving device configuration...${NC}"
    cat > "$CONFIG_DIR/device_config.json" <<EOF
{
    "qr_shared_key": "$QR_KEY",
    "device_id": "$DEVICE_ID",
    "device_secret": "$DEVICE_SECRET",
    "hostname": "$HOSTNAME",
    "hardware_serial": "$HARDWARE_SERIAL",
    "ec2_host": "$EC2_HOST",
    "registered_at": $(date +%s)
}
EOF
    chmod 600 "$CONFIG_DIR/device_config.json"
    log "  ✓ Config saved: $CONFIG_DIR/device_config.json"
    
    # Also create /etc/mdai/device_config.json (required by SystemController)
    log "${YELLOW}Creating /etc/mdai/device_config.json...${NC}"
    sudo mkdir -p /etc/mdai
    sudo bash -c "cat > /etc/mdai/device_config.json << INNEREOF
{
  \"device_id\": \"$DEVICE_ID\",
  \"hardware_id\": \"$HARDWARE_SERIAL\"
}
INNEREOF"
    log "  ✓ Config saved: /etc/mdai/device_config.json"
    
    # Create registration flag
    touch "$REGISTRATION_FLAG"
    chmod 644 "$REGISTRATION_FLAG"
    
    # DELETE SSH KEY - Critical security step!
    log "${YELLOW}Deleting SSH registration key (security)...${NC}"
    rm -f "$SSH_KEY"
    rm -f "$SSH_KEY.pub" 2>/dev/null || true
    rm -f "/home/mercleDev/.ssh/known_hosts" 2>/dev/null || true
    log "  ✓ SSH key deleted: $SSH_KEY"
    
    log ""
    log "${GREEN}========================================${NC}"
    log "${GREEN}   Registration Complete!${NC}"
    log "${GREEN}========================================${NC}"
    log ""
    log "Device ID: $DEVICE_ID"
    log "Config: $CONFIG_DIR/device_config.json"
    log "Status: REGISTERED"
    log ""
    log "The device can now authenticate with the EC2 server."
    log "SSH key has been deleted - this device cannot re-register."
    log ""
    
    exit 0
else
    log "${RED}========================================${NC}"
    log "${RED}   Registration FAILED${NC}"
    log "${RED}========================================${NC}"
    log "EC2 server did not return SUCCESS."
    log "Please check the EC2 server logs."
    exit 1
fi

