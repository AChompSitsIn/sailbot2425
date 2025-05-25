import serial

def read_ubx_nav_pvt(port="/dev/ttyACM0", baudrate=38400):
    def find_ubx_pvt(buffer):
        # UBX header: 0xB5 0x62
        for i in range(len(buffer) - 1):
            if buffer[i] == 0xB5 and buffer[i+1] == 0x62:
                # Check if it's NAV-PVT: Class = 0x01, ID = 0x07
                if buffer[i+2] == 0x01 and buffer[i+3] == 0x07:
                    length = int.from_bytes(buffer[i+4:i+6], 'little')
                    if i + 6 + length <= len(buffer):
                        return buffer[i:i+6+length]
        return None

    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1)
        print(f"Reading UBX-NAV-PVT on {port} at {baudrate} baud")
        while True:
            data = ser.read(100)
            if not data:
                continue
            msg = find_ubx_pvt(data)
            if msg:
                fix_type = msg[26]
                fix_map = {
                    0: "No Fix",
                    1: "Dead Reckoning only",
                    2: "2D Fix",
                    3: "3D Fix",
                    4: "RTK Float",
                    5: "RTK Fixed"
                }
                print(f"UBX Fix Type: {fix_map.get(fix_type, 'Unknown')} ({fix_type})")
    except KeyboardInterrupt:
        print("Exiting...")
    except Exception as e:
        print(f"Error: {e}")

read_ubx_nav_pvt()
