#!/usr/bin/env python3

import subprocess

def scan_bus_1():
    try:
        output = subprocess.check_output(['i2cdetect', '-y', '1'], universal_newlines=True)
        print("=== Bus 1 (/dev/i2c-1) ===")
        print(output.strip())
    except subprocess.CalledProcessError as e:
        print(f"Error scanning i2c-1: {e}")

if __name__ == "__main__":
    scan_bus_1()
