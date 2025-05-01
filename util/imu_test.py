#!/usr/bin/env python3
"""
imu_test.py  –  robust smbus driver for SparkFun ICM‑20948 on Jetson

• Streams accel / gyro at ~20 Hz
• Recovers from any I²C lock‑up (<1 s pause) with watchdog‑style bus reset
• Handles DEVICE_RESET, re‑init, and exponential back‑off
"""

import os, fcntl, time, sys, smbus

# ────────────────────────────── constants ────────────────────────────────────
BUS_NUM        = 1
ADDR           = 0x69

REG_BANK_SEL   = 0x7F
WHO_AM_I       = 0x00
PWR_MGMT_1     = 0x06
ACCEL_XOUT_H   = 0x2D
GYRO_XOUT_H    = 0x33

ACCEL_SCALE    = 16384.0   # ±2 g
GYRO_SCALE     = 131.0     # ±250 dps
I2C_SLAVE_FORCE = 0x0706   # ioctl for ping helper

# ────────────────────────── low‑level helpers ────────────────────────────────
bus = None  # global smbus handle
fail_count = 0

def open_bus():
    global bus
    if bus:
        try: bus.close()
        except: pass
    bus = smbus.SMBus(BUS_NUM)

def select_bank(bank):
    bus.write_byte_data(ADDR, REG_BANK_SEL, bank << 4)

def read_word(reg):
    hi = bus.read_byte_data(ADDR, reg)
    lo = bus.read_byte_data(ADDR, reg + 1)
    v  = (hi << 8) | lo
    return v if v < 32768 else v - 65536

def soft_reset():
    select_bank(0)
    bus.write_byte_data(ADDR, PWR_MGMT_1, 0x80)  # DEVICE_RESET
    time.sleep(0.06)                             # datasheet: 50 ms min

def init_imu():
    select_bank(0)
    if bus.read_byte_data(ADDR, WHO_AM_I) != 0xEA:
        raise RuntimeError("WHO_AM_I mismatch")
    bus.write_byte_data(ADDR, PWR_MGMT_1, 0x01)  # wake
    time.sleep(0.01)
    select_bank(2)                               # enable sensors
    bus.write_byte_data(ADDR, 0x10, 0x01)        # gyro
    bus.write_byte_data(ADDR, 0x14, 0x01)        # accel
    select_bank(0)

def read_sensors():
    ax = read_word(ACCEL_XOUT_H)      / ACCEL_SCALE
    ay = read_word(ACCEL_XOUT_H + 2)  / ACCEL_SCALE
    az = read_word(ACCEL_XOUT_H + 4)  / ACCEL_SCALE
    gx = read_word(GYRO_XOUT_H)       / GYRO_SCALE
    gy = read_word(GYRO_XOUT_H + 2)   / GYRO_SCALE
    gz = read_word(GYRO_XOUT_H + 4)   / GYRO_SCALE
    return ax, ay, az, gx, gy, gz

# ───────────────────────── watchdog (non‑blocking) ───────────────────────────
def ping(timeout=1.0, reg=WHO_AM_I):
    """ Return True when a single‑byte read succeeds within timeout. """
    start = time.time()
    while time.time() - start < timeout:
        try:
            fd = os.open(f"/dev/i2c-{BUS_NUM}", os.O_RDWR | os.O_NONBLOCK)
            try:
                fcntl.ioctl(fd, I2C_SLAVE_FORCE, ADDR)
                os.write(fd, bytes([reg]))
                os.read(fd, 1)
                return True
            finally:
                os.close(fd)
        except OSError:
            time.sleep(0.02)
    return False

def recover():
    """Re‑open bus & re‑init IMU with exponential back‑off."""
    global fail_count
    backoff = min(1.0, 0.1 * (2 ** fail_count))
    if fail_count == 0:
        print("⚠️  I²C error — attempting recovery …", flush=True)
    fail_count += 1

    # wait until lines release (non‑blocking ping)
    while not ping(timeout=1.0):
        time.sleep(0.05)

    try:
        open_bus()
        soft_reset()
        init_imu()
        fail_count = 0
        print("✅  Link restored", flush=True)
    except Exception:
        time.sleep(backoff)   # next outer‑loop iteration will retry

# ───────────────────────────────── main ──────────────────────────────────────
if __name__ == "__main__":
    open_bus()
    try:
        init_imu()
    except Exception as e:
        sys.exit(f"Initial IMU init failed: {e}")

    try:
        while True:
            try:
                ax, ay, az, gx, gy, gz = read_sensors()
                if fail_count == 0:   # print only when healthy
                    print(f"Accel[g] X:{ax:+.2f} Y:{ay:+.2f} Z:{az:+.2f} | "
                          f"Gyro[°/s] X:{gx:+.1f} Y:{gy:+.1f} Z:{gz:+.1f}")
                time.sleep(0.05)
            except (OSError, IOError):
                recover()
    except KeyboardInterrupt:
        print("\nCtrl‑C — exiting")
