#!/bin/bash

# Tailscale Watchdog - Ensures Tailscale is ALWAYS running

while true; do
    # Wait 30 seconds between checks
    sleep 30
    
    # Check if tailscaled service is running
    if ! systemctl is-active --quiet tailscaled; then
        logger -t tailscale-watchdog "ERROR: tailscaled is DOWN! Restarting..."
        systemctl restart tailscaled
        sleep 10
    fi
    
    # Check if Tailscale has an IP
    TS_IP=
    if [ -z "" ]; then
        logger -t tailscale-watchdog "WARNING: Tailscale has no IP! Reconnecting..."
        tailscale up --accept-dns --accept-routes --reset 2>&1 | logger -t tailscale-watchdog
    fi
    
    # Check if Tailscale can reach the coordinator
    if ! tailscale status &>/dev/null; then
        logger -t tailscale-watchdog "ERROR: Tailscale status check failed! Restarting service..."
        systemctl restart tailscaled
        sleep 10
        tailscale up --accept-dns --accept-routes --reset 2>&1 | logger -t tailscale-watchdog
    fi
done
