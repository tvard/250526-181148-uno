#!/usr/bin/env python3
"""
Arduino firmware uploader with automatic reset functionality.
Optimized for PlatformIO integration with proper exit codes and progress output.
"""

import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional

import serial 

class ArduinoUploader:
    """Handles Arduino firmware uploads with automatic reset."""
    
    # Common avrdude installation paths
    AVRDUDE_PATHS = [
        "~/.platformio/packages/tool-avrdude/avrdude",
        "~/.platformio/packages/tool-avrdude/bin/avrdude", 
        "~/.platformio/packages/tool-avrdude/avrdude.exe",
        "~/.platformio/packages/tool-avrdude/bin/avrdude.exe",
        "/usr/bin/avrdude",
        "/usr/local/bin/avrdude",
        "C:/avrdude/bin/avrdude.exe",
    ]
    
    # Bootloader programmers in order of preference
    BOOTLOADER_PROGRAMMERS = [
        "arduino",      # Most common Arduino bootloader
        "stk500v1",     # STK500 v1 protocol
        "stk500v2",     # STK500 v2 protocol  
        "avr109",       # AVR109 bootloader protocol
    ]
    
    # Exit codes for different failure types
    EXIT_SUCCESS = 0
    EXIT_UPLOAD_FAILED = 1
    EXIT_AVRDUDE_NOT_FOUND = 2
    EXIT_INVALID_ARGS = 3
    EXIT_FILE_NOT_FOUND = 4
    EXIT_RESET_FAILED = 5
    
    def __init__(self, mcu: str = "atmega328p"):
        """Initialize uploader with target MCU."""
        self.mcu = mcu
        self.avrdude_path = self._find_avrdude()
    
    def _find_avrdude(self) -> Optional[str]:
        """Find avrdude executable, return None if not found."""

        # Check if avrdude is in PATH first
        if shutil.which("avrdude"):
            return "avrdude"
        
        # Check common installation paths
        for path_str in self.AVRDUDE_PATHS:
            path = Path(path_str).expanduser()
            if path.exists() and path.is_file():
                return str(path)
        
        return None
    
    def _perform_reset(self, port: str) -> bool:
        """Perform hardware reset via RTS signal."""
        try:
            print(f"Performing reset on {port}...")
            with serial.Serial(port) as ser:
                ser.rts = True   # Assert reset
                time.sleep(0.3)  # Hold reset
                ser.rts = False  # Release reset
            
            time.sleep(0.5)  # Wait for bootloader
            print("Reset completed, bootloader ready")
            return True
            
        except Exception as e:
            print(f"ERROR: Reset failed - {e}")
            return False
    
    def _try_upload(self, port: str, baud_rate: str, firmware_path: str, 
                   programmer: str) -> bool:
        """Attempt upload with specific programmer."""
        print(f"Trying programmer: {programmer}")
        
        # Build avrdude command - let avrdude handle its own output
        cmd = [
            self.avrdude_path, "-v",
            "-p", self.mcu,
            "-c", programmer,
            "-P", port,
            "-b", baud_rate,
            "-D",  # Don't erase chip
            "-U", f"flash:w:{firmware_path}:i"
        ]
        
        try:
            # Don't capture output - let avrdude print directly to console
            # This preserves progress bars and real-time feedback
            result = subprocess.run(cmd, timeout=30)
            
            if result.returncode == 0:
                print(f"SUCCESS: Upload completed with {programmer}")
                return True
            else:
                print(f"FAILED: Upload failed with {programmer}")
                return False
                
        except subprocess.TimeoutExpired:
            print(f"TIMEOUT: Upload timed out with {programmer}")
            return False
        except Exception as e:
            print(f"ERROR: {programmer} failed - {e}")
            return False
    
    def upload(self, port: str, baud_rate: str, firmware_path: str) -> int:
        """
        Upload firmware with automatic reset.
        
        Returns:
            Exit code (0 for success, non-zero for various failures)
        """
        # Validate avrdude
        if not self.avrdude_path:
            print("ERROR: avrdude not found!")
            print("Please install Arduino IDE, PlatformIO, or avrdude separately.")
            return self.EXIT_AVRDUDE_NOT_FOUND
        
        # Validate firmware file
        if not Path(firmware_path).exists():
            print(f"ERROR: Firmware file not found: {firmware_path}")
            return self.EXIT_FILE_NOT_FOUND
        
        print("=" * 50)
        print("Arduino Upload with Auto-Reset")
        print("=" * 50)
        print(f"Port: {port}")
        print(f"Speed: {baud_rate}")
        print(f"Firmware: {firmware_path}")
        print(f"MCU: {self.mcu}")
        print(f"avrdude: {self.avrdude_path}")
        print("=" * 50)
        
        # Perform reset
        if not self._perform_reset(port):
            return self.EXIT_RESET_FAILED
        
        # Try each programmer until one succeeds
        for i, programmer in enumerate(self.BOOTLOADER_PROGRAMMERS, 1):
            print(f"\n--- Attempt {i}/{len(self.BOOTLOADER_PROGRAMMERS)} ---")
            
            if self._try_upload(port, baud_rate, firmware_path, programmer):
                print("\n" + "=" * 50)
                print("UPLOAD SUCCESSFUL!")
                print("=" * 50)
                return self.EXIT_SUCCESS
            
            # Small delay between attempts
            if i < len(self.BOOTLOADER_PROGRAMMERS):
                time.sleep(0.5)
        
        print("\n" + "=" * 50)
        print("UPLOAD FAILED - All programmers failed!")
        print("=" * 50)
        return self.EXIT_UPLOAD_FAILED


def main():
    """Main entry point for PlatformIO integration."""
    # Simple argument parsing for PlatformIO compatibility
    if len(sys.argv) != 4:
        print("ERROR: Invalid arguments")
        print("Usage: python reset_rts.py <port> <speed> <firmware_file>")
        print("Example: python reset_rts.py COM3 115200 firmware.hex")
        return 3  # EXIT_INVALID_ARGS
    
    port = sys.argv[1]
    baud_rate = sys.argv[2] 
    firmware_path = sys.argv[3]
    
    # Create uploader and run
    uploader = ArduinoUploader()
    exit_code = uploader.upload(port, baud_rate, firmware_path)
    return exit_code


if __name__ == "__main__":
    try:
        exit_code = main()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\nUpload cancelled by user")
        sys.exit(130)  # Standard exit code for SIGINT
    except Exception as e:
        print(f"UNEXPECTED ERROR: {e}")
        sys.exit(1)  # Generic error