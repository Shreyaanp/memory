#!/usr/bin/env python3
"""
ESP Serial Communication Tool

Usage:
    python esp_serial.py                    # Interactive mode (default port)
    python esp_serial.py -p /dev/ttyUSB0    # Specify port
    python esp_serial.py -l                 # List available ports
    python esp_serial.py -s "hello"         # Send single message and exit
"""

import serial
import serial.tools.list_ports
import argparse
import sys
import threading
import time

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200


def list_ports():
    """List all available serial ports."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    
    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device} - {port.description}")
        if port.manufacturer:
            print(f"      Manufacturer: {port.manufacturer}")
        if port.serial_number:
            print(f"      Serial: {port.serial_number}")


def read_serial(ser, stop_event):
    """Background thread to read and print serial data."""
    while not stop_event.is_set():
        try:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8', errors='replace').strip()
                if data:
                    print(f"\n← ESP: {data}")
                    print("> ", end="", flush=True)
        except Exception as e:
            if not stop_event.is_set():
                print(f"\nRead error: {e}")
            break
        time.sleep(0.01)


def send_message(ser, message):
    """Send a message to the ESP."""
    try:
        # Add newline if not present (ESP usually expects it)
        if not message.endswith('\n'):
            message += '\n'
        ser.write(message.encode('utf-8'))
        ser.flush()
        return True
    except Exception as e:
        print(f"Send error: {e}")
        return False


def interactive_mode(port, baud):
    """Run interactive serial communication."""
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        print(f"Connected to {port} at {baud} baud")
        print("Type messages to send. Commands: 'quit' to exit, 'clear' to clear screen")
        print("-" * 50)
    except serial.SerialException as e:
        print(f"Failed to open {port}: {e}")
        print("Use -l to list available ports")
        sys.exit(1)
    
    # Start reader thread
    stop_event = threading.Event()
    reader = threading.Thread(target=read_serial, args=(ser, stop_event), daemon=True)
    reader.start()
    
    try:
        while True:
            try:
                msg = input("> ")
            except EOFError:
                break
            
            if msg.lower() == 'quit':
                break
            elif msg.lower() == 'clear':
                print("\033[2J\033[H", end="")  # ANSI clear screen
                continue
            elif msg.strip() == '':
                continue
            
            send_message(ser, msg)
            print(f"→ Sent: {msg.strip()}")
            
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        stop_event.set()
        ser.close()
        print("Connection closed.")


def single_send(port, baud, message, wait_response=True):
    """Send a single message and optionally wait for response."""
    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"Failed to open {port}: {e}")
        sys.exit(1)
    
    try:
        send_message(ser, message)
        print(f"→ Sent: {message}")
        
        if wait_response:
            time.sleep(0.1)
            response = ""
            while ser.in_waiting > 0:
                response += ser.read(ser.in_waiting).decode('utf-8', errors='replace')
                time.sleep(0.05)
            if response:
                print(f"← Response: {response.strip()}")
    finally:
        ser.close()


def main():
    parser = argparse.ArgumentParser(description="ESP Serial Communication Tool")
    parser.add_argument("-p", "--port", default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument("-l", "--list", action="store_true", help="List available ports")
    parser.add_argument("-s", "--send", type=str, help="Send single message and exit")
    parser.add_argument("--no-wait", action="store_true", help="Don't wait for response (with -s)")
    
    args = parser.parse_args()
    
    if args.list:
        list_ports()
        return
    
    if args.send:
        single_send(args.port, args.baud, args.send, wait_response=not args.no_wait)
        return
    
    interactive_mode(args.port, args.baud)


if __name__ == "__main__":
    main()

