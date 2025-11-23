#!/bin/bash
# Quick Production Readiness Test

echo "==================================="
echo "MDAI SYSTEM PRODUCTION READY CHECK"
echo "==================================="
echo ""

PASS=0
FAIL=0

# 1. Check Serial Device
echo "1. Checking ESP32 Serial Device..."
if [ -e "/dev/ttyACM0" ]; then
    echo "   ✅ /dev/ttyACM0 exists"
    PASS=$((PASS+1))
else
    echo "   ❌ /dev/ttyACM0 NOT FOUND"
    FAIL=$((FAIL+1))
fi

# 2. Check Camera
echo "2. Checking RealSense Camera..."
if lsusb | grep -q "RealSense"; then
    echo "   ✅ RealSense camera detected"
    PASS=$((PASS+1))
else
    echo "   ❌ RealSense camera NOT FOUND"
    FAIL=$((FAIL+1))
fi

# 3. Check Device Config
echo "3. Checking Device Config..."
if [ -f "/opt/mdai/device_config.json" ]; then
    echo "   ✅ Device config exists"
    PASS=$((PASS+1))
else
    echo "   ❌ Device config MISSING"
    FAIL=$((FAIL+1))
fi

# 4. Check Binary
echo "4. Checking Binary..."
if [ -f "/home/mercleDev/codebase/build/mdai_system" ]; then
    echo "   ✅ mdai_system binary exists"
    PASS=$((PASS+1))
else
    echo "   ❌ Binary NOT FOUND"
    FAIL=$((FAIL+1))
fi

# 5. Check Models
echo "5. Checking Face Detection Models..."
if [ -f "/home/mercleDev/codebase/models/res10_300x300_ssd_iter_140000.caffemodel" ]; then
    echo "   ✅ OpenCV DNN model exists"
    PASS=$((PASS+1))
else
    echo "   ❌ Model files MISSING"
    FAIL=$((FAIL+1))
fi

# 6. Check User Permissions
echo "6. Checking Permissions..."
if groups $USER | grep -q dialout; then
    echo "   ✅ User in dialout group"
    PASS=$((PASS+1))
else
    echo "   ❌ User needs dialout group"
    FAIL=$((FAIL+1))
fi

# 7. Quick Binary Test
echo "7. Testing Binary Startup..."
cd /home/mercleDev/codebase
timeout 3 sudo ./build/mdai_system >/dev/null 2>&1
if [ $? -eq 124 ]; then
    echo "   ✅ Binary starts (timeout = running)"
    PASS=$((PASS+1))
else
    echo "   ❌ Binary failed to start"
    FAIL=$((FAIL+1))
fi

echo ""
echo "==================================="
echo "RESULTS: $PASS passed, $FAIL failed"
echo "==================================="
echo ""

if [ $FAIL -eq 0 ]; then
    echo "✅ SYSTEM IS PRODUCTION READY"
    echo ""
    echo "Next Steps:"
    echo "  1. Create systemd services"
    echo "  2. Enable auto-start on boot"
    echo "  3. Test end-to-end flow"
    exit 0
else
    echo "❌ SYSTEM NOT READY - Fix failures above"
    exit 1
fi


# Quick Production Readiness Test

echo "==================================="
echo "MDAI SYSTEM PRODUCTION READY CHECK"
echo "==================================="
echo ""

PASS=0
FAIL=0

# 1. Check Serial Device
echo "1. Checking ESP32 Serial Device..."
if [ -e "/dev/ttyACM0" ]; then
    echo "   ✅ /dev/ttyACM0 exists"
    PASS=$((PASS+1))
else
    echo "   ❌ /dev/ttyACM0 NOT FOUND"
    FAIL=$((FAIL+1))
fi

# 2. Check Camera
echo "2. Checking RealSense Camera..."
if lsusb | grep -q "RealSense"; then
    echo "   ✅ RealSense camera detected"
    PASS=$((PASS+1))
else
    echo "   ❌ RealSense camera NOT FOUND"
    FAIL=$((FAIL+1))
fi

# 3. Check Device Config
echo "3. Checking Device Config..."
if [ -f "/opt/mdai/device_config.json" ]; then
    echo "   ✅ Device config exists"
    PASS=$((PASS+1))
else
    echo "   ❌ Device config MISSING"
    FAIL=$((FAIL+1))
fi

# 4. Check Binary
echo "4. Checking Binary..."
if [ -f "/home/mercleDev/codebase/build/mdai_system" ]; then
    echo "   ✅ mdai_system binary exists"
    PASS=$((PASS+1))
else
    echo "   ❌ Binary NOT FOUND"
    FAIL=$((FAIL+1))
fi

# 5. Check Models
echo "5. Checking Face Detection Models..."
if [ -f "/home/mercleDev/codebase/models/res10_300x300_ssd_iter_140000.caffemodel" ]; then
    echo "   ✅ OpenCV DNN model exists"
    PASS=$((PASS+1))
else
    echo "   ❌ Model files MISSING"
    FAIL=$((FAIL+1))
fi

# 6. Check User Permissions
echo "6. Checking Permissions..."
if groups $USER | grep -q dialout; then
    echo "   ✅ User in dialout group"
    PASS=$((PASS+1))
else
    echo "   ❌ User needs dialout group"
    FAIL=$((FAIL+1))
fi

# 7. Quick Binary Test
echo "7. Testing Binary Startup..."
cd /home/mercleDev/codebase
timeout 3 sudo ./build/mdai_system >/dev/null 2>&1
if [ $? -eq 124 ]; then
    echo "   ✅ Binary starts (timeout = running)"
    PASS=$((PASS+1))
else
    echo "   ❌ Binary failed to start"
    FAIL=$((FAIL+1))
fi

echo ""
echo "==================================="
echo "RESULTS: $PASS passed, $FAIL failed"
echo "==================================="
echo ""

if [ $FAIL -eq 0 ]; then
    echo "✅ SYSTEM IS PRODUCTION READY"
    echo ""
    echo "Next Steps:"
    echo "  1. Create systemd services"
    echo "  2. Enable auto-start on boot"
    echo "  3. Test end-to-end flow"
    exit 0
else
    echo "❌ SYSTEM NOT READY - Fix failures above"
    exit 1
fi

