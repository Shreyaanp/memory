#!/bin/bash
# ESP32 Detection Debug Script
# Checks for ESP32 serial devices and provides diagnostic information

echo "=========================================="
echo "ESP32 Serial Detection Debug Tool"
echo "=========================================="
echo ""

# Check for ttyACM devices
echo "1. Checking for /dev/ttyACM* devices..."
if ls /dev/ttyACM* 1> /dev/null 2>&1; then
    echo "   ✅ Found devices:"
    ls -la /dev/ttyACM* | while read line; do
        echo "      $line"
    done
else
    echo "   ❌ No /dev/ttyACM* devices found"
fi
echo ""

# Check for ttyUSB devices (alternative ESP32 connection)
echo "2. Checking for /dev/ttyUSB* devices..."
if ls /dev/ttyUSB* 1> /dev/null 2>&1; then
    echo "   ✅ Found devices:"
    ls -la /dev/ttyUSB* | while read line; do
        echo "      $line"
    done
else
    echo "   ❌ No /dev/ttyUSB* devices found"
fi
echo ""

# Check USB devices
echo "3. Checking USB devices..."
echo "   USB devices connected:"
lsusb | grep -i "esp\|serial\|cp210\|ch340\|ft232\|arduino\|lilygo" || echo "   ⚠️  No ESP32-related USB devices found"
echo ""

# Check dmesg for recent USB/serial events
echo "4. Recent USB/Serial events (last 50 lines)..."
dmesg | tail -50 | grep -i "tty\|usb\|serial\|acm\|cp210\|ch340\|ft232" | tail -20 || echo "   No recent USB/serial events"
echo ""

# Check if user is in dialout group (required for serial access)
echo "5. Checking user permissions..."
if groups | grep -q dialout; then
    echo "   ✅ User is in 'dialout' group (can access serial ports)"
else
    echo "   ❌ User is NOT in 'dialout' group"
    echo "   💡 Fix: sudo usermod -a -G dialout $USER"
    echo "   💡 Then logout and login again"
fi
echo ""

# Check udev rules
echo "6. Checking udev rules..."
if [ -d /etc/udev/rules.d ]; then
    echo "   Udev rules directory exists"
    if ls /etc/udev/rules.d/*serial* 1> /dev/null 2>&1; then
        echo "   ✅ Found serial-related udev rules:"
        ls -la /etc/udev/rules.d/*serial*
    else
        echo "   ⚠️  No serial-specific udev rules found"
    fi
else
    echo "   ❌ /etc/udev/rules.d does not exist"
fi
echo ""

# Check if ESP32 might be on a different port
echo "7. All available serial ports..."
if ls /dev/tty* 1> /dev/null 2>&1; then
    echo "   Available serial ports:"
    ls -la /dev/tty* 2>/dev/null | grep -E "ACM|USB|S[0-9]" | head -10
else
    echo "   ❌ No serial ports found"
fi
echo ""

# Test if we can open a port (if found)
echo "8. Testing port access..."
if ls /dev/ttyACM* 1> /dev/null 2>&1; then
    for port in /dev/ttyACM*; do
        echo "   Testing $port..."
        if [ -r "$port" ] && [ -w "$port" ]; then
            echo "      ✅ Read/Write access: OK"
        else
            echo "      ❌ Read/Write access: FAILED"
        fi
    done
else
    echo "   ⚠️  No ports to test"
fi
echo ""

echo "=========================================="
echo "Diagnostic Summary"
echo "=========================================="
echo ""
echo "Common ESP32 detection issues:"
echo "  1. ESP32 not plugged in or powered off"
echo "  2. Wrong USB cable (data vs power-only)"
echo "  3. User not in 'dialout' group"
echo "  4. ESP32 driver not installed (CP210x, CH340, etc.)"
echo "  5. ESP32 in wrong mode (needs to be in programming/upload mode)"
echo ""
echo "Next steps:"
echo "  1. Check USB cable connection"
echo "  2. Try unplugging and replugging ESP32"
echo "  3. Check if ESP32 LED is blinking (indicates power)"
echo "  4. Run: sudo usermod -a -G dialout $USER (if not in dialout group)"
echo "  5. Check ESP32 board documentation for driver requirements"
echo ""

