#!/usr/bin/env python3
"""
Arduino uploader reset helper for PlatformIO, specifically for the Arduino Uno (uses DTR for reset)
Performs a DTR reset on the specified serial port, then calls PlatformIO's upload.
Usage: python rts_reset_upload_wrapper.py <port>
"""

import sys
import time
import serial
import subprocess

def perform_reset(port):
    try:
        print(f"Performing DTR reset on {port}...")
        with serial.Serial(port) as ser:
            ser.dtr = False  # Assert reset (active low)
            time.sleep(0.3)
            ser.dtr = True   # Release reset
        time.sleep(0.5)  # Wait for bootloader
        print("Reset completed, bootloader ready")
        return True
    except Exception as e:
        print(f"ERROR: Reset failed - {e}")
        return False

def main():
    if len(sys.argv) != 2:
        print("Usage: python rts_reset_upload_wrapper.py <port>")
        sys.exit(1)
    port = sys.argv[1]

    if not perform_reset(port):
        sys.exit(2)

    # Call PlatformIO's upload (uses platformio.ini for all params)
    result = subprocess.run(["pio", "run", "-t", "upload"])
    sys.exit(result.returncode)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nUpload cancelled by user")
        sys.exit(130)