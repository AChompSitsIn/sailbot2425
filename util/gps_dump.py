import serial

ser = serial.Serial("/dev/ttyAMA0", 38400, timeout=1)

while True:
    line = ser.readline().decode(errors='ignore').strip()
    print(f"RAW: {line}")
