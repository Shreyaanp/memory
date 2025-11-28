#!/bin/bash
# MDai System Control Script
# Usage: ./mdai.sh [restart|build|stop]

cd /home/mercleDev/codebase

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

kill_all() {
    echo -e "${YELLOW}Killing all mdai processes...${NC}"
    sudo pkill -9 mdai_system 2>/dev/null
    pkill -9 mdai_system 2>/dev/null
    sleep 1
    
    # Verify killed
    if pgrep -f mdai_system > /dev/null; then
        echo -e "${RED}Warning: Some processes still running${NC}"
        pgrep -f mdai_system
    else
        echo -e "${GREEN}All processes stopped${NC}"
    fi
}

start_server() {
    echo -e "${GREEN}Starting mdai_system...${NC}"
    cd /home/mercleDev/codebase/build
    export LD_LIBRARY_PATH=$PWD:$LD_LIBRARY_PATH
    
    # Clear log
    echo "" > /home/mercleDev/mdai_logs/rdk.log
    
    # Start in background
    ./mdai_system &
    
    sleep 3
    
    if pgrep -f mdai_system > /dev/null; then
        echo -e "${GREEN}✓ Server started (PID: $(pgrep -f mdai_system | head -1))${NC}"
        echo -e "${GREEN}✓ Logs: tail -f /home/mercleDev/mdai_logs/rdk.log${NC}"
    else
        echo -e "${RED}✗ Server failed to start${NC}"
        tail -20 /home/mercleDev/mdai_logs/rdk.log
    fi
}

build_server() {
    echo -e "${YELLOW}Building with 7 parallel jobs...${NC}"
    cd /home/mercleDev/codebase/build
    
    if make -j7 2>&1; then
        echo -e "${GREEN}✓ Build successful${NC}"
        return 0
    else
        echo -e "${RED}✗ Build failed${NC}"
        return 1
    fi
}

case "$1" in
    restart)
        kill_all
        start_server
        ;;
    build)
        kill_all
        if build_server; then
            start_server
        fi
        ;;
    stop)
        kill_all
        ;;
    logs)
        tail -f /home/mercleDev/mdai_logs/rdk.log
        ;;
    *)
        echo "MDai System Control"
        echo ""
        echo "Usage: ./mdai.sh [command]"
        echo ""
        echo "Commands:"
        echo "  restart  - Kill all & restart existing build"
        echo "  build    - Kill all & build (7 jobs) & restart"
        echo "  stop     - Kill all processes"
        echo "  logs     - Follow log output"
        echo ""
        ;;
esac



