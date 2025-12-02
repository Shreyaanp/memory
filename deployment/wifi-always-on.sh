#!/bin/bash

# ═══════════════════════════════════════════════════════════════════════════════
# WiFi Always-On Boot Service
# ═══════════════════════════════════════════════════════════════════════════════
# Ensures WiFi radio is ALWAYS enabled and connects to known networks on boot
# This prevents the RDK from being unreachable due to WiFi being disabled
# ═══════════════════════════════════════════════════════════════════════════════

LOG_FILE="/var/log/wifi-always-on.log"

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

log "═══════════════════════════════════════════════════════════"
log "WiFi Always-On Service Starting"
log "═══════════════════════════════════════════════════════════"

# Step 1: Ensure WiFi radio is ON
log "Step 1: Ensuring WiFi radio is enabled..."
nmcli radio wifi on 2>&1 | tee -a "$LOG_FILE"
if nmcli radio wifi | grep -q "enabled"; then
    log "✅ WiFi radio is ON"
else
    log "⚠️  WiFi radio status unclear, trying again..."
    sleep 2
    nmcli radio wifi on
fi

# Step 2: Wait for WiFi interface to be ready
log "Step 2: Waiting for WiFi interface..."
for i in {1..10}; do
    if nmcli device status | grep -q "wifi.*connected\|wifi.*disconnected"; then
        log "✅ WiFi interface ready"
        break
    fi
    log "Waiting for WiFi interface... ($i/10)"
    sleep 1
done

# Step 3: Try to connect to known networks (from WiFi manager config)
log "Step 3: Connecting to known WiFi networks..."

WIFI_CONFIG_DIR="/etc/rdk-wifi-networks"

if [ -d "$WIFI_CONFIG_DIR" ] && [ "$(ls -A $WIFI_CONFIG_DIR/*.conf 2>/dev/null)" ]; then
    log "Found WiFi configurations, attempting to connect..."
    
    # Try each configured network by priority
    for conf in $(ls -v $WIFI_CONFIG_DIR/*.conf 2>/dev/null | sort -r); do
        if [ -f "$conf" ]; then
            source "$conf"
            log "Trying to connect to: $SSID (Priority: $PRIORITY)"
            
            # Check if network is available
            if nmcli device wifi list | grep -q "^.*$SSID"; then
                log "Network '$SSID' is available, connecting..."
                if nmcli connection up "$SSID" 2>&1 | tee -a "$LOG_FILE"; then
                    log "✅ Connected to $SSID"
                    break
                else
                    log "Failed to connect to $SSID, trying next..."
                fi
            else
                log "Network '$SSID' not in range"
            fi
        fi
    done
else
    log "⚠️  No WiFi configurations found in $WIFI_CONFIG_DIR"
    log "⚠️  Please configure WiFi networks using rdk-wifi-manager.sh"
fi

# Step 4: Verify connection
log "Step 4: Verifying WiFi connection..."
sleep 3

CURRENT_SSID=$(nmcli -t -f active,ssid dev wifi | grep '^yes' | cut -d: -f2)
if [ -n "$CURRENT_SSID" ]; then
    log "✅ Connected to WiFi: $CURRENT_SSID"
    
    # Test internet
    if ping -c 1 8.8.8.8 &>/dev/null; then
        log "✅ Internet connectivity verified"
    else
        log "⚠️  Connected to WiFi but no internet"
    fi
else
    log "⚠️  Not connected to any WiFi network"
    log "⚠️  Device may not be reachable remotely"
fi

log "═══════════════════════════════════════════════════════════"
log "WiFi Always-On Service Complete"
log "═══════════════════════════════════════════════════════════"

exit 0

