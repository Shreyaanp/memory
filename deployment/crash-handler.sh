#!/bin/bash
# MDAI Crash Handler
# Called when mdai_system crashes - logs info and ensures restart

CRASH_LOG="/home/mercleDev/mdai_logs/crashes.log"
CORE_DIR="/home/mercleDev/mdai_logs/cores"

# Create directories
mkdir -p "$CORE_DIR"

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" >> "$CRASH_LOG"
}

log "=========================================="
log "CRASH DETECTED"
log "Signal: $1"
log "PID: $2"
log "=========================================="

# Save system state at crash time
log "Memory at crash:"
free -h >> "$CRASH_LOG" 2>&1

log "Load at crash:"
cat /proc/loadavg >> "$CRASH_LOG" 2>&1

# Check for core dump
LATEST_CORE=$(ls -t /var/crash/*mdai* 2>/dev/null | head -1)
if [ -n "$LATEST_CORE" ]; then
    log "Core dump found: $LATEST_CORE"
    # Move to our directory
    mv "$LATEST_CORE" "$CORE_DIR/" 2>/dev/null
fi

log "Crash handler complete"



