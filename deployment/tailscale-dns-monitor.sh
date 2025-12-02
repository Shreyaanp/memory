#!/bin/bash

# Tailscale DNS Health Monitor
# Checks every 60 seconds if MagicDNS is working and auto-recovers if not

HOSTNAME=$(tailscale status --json 2>/dev/null | grep -o '"HostName":"[^"]*"' | head -1 | cut -d'"' -f4)
MAGIC_DNS_SUFFIX="tail812503.ts.net"

# Function to check if DNS is working
check_dns() {
    if [ -z "$HOSTNAME" ]; then
        return 1
    fi
    
    # Try to resolve a known Tailscale hostname
    nslookup "${HOSTNAME}.${MAGIC_DNS_SUFFIX}" 100.100.100.100 &>/dev/null
    return $?
}

# Function to recover DNS
recover_dns() {
    logger -t tailscale-dns-monitor "MagicDNS failure detected, recovering..."
    
    # Restart systemd-resolved
    systemctl restart systemd-resolved
    sleep 2
    
    # Reconfigure Tailscale with DNS
    tailscale down 2>/dev/null || true
    sleep 2
    tailscale up --accept-dns --accept-routes
    sleep 3
    
    if check_dns; then
        logger -t tailscale-dns-monitor "DNS recovery successful"
        return 0
    else
        logger -t tailscale-dns-monitor "DNS recovery failed"
        return 1
    fi
}

# Main monitoring loop
while true; do
    if ! check_dns; then
        logger -t tailscale-dns-monitor "DNS check failed, attempting recovery"
        recover_dns
    fi
    sleep 60
done
