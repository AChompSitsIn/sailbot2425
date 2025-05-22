#!/usr/bin/env python3
import serial, time, re, argparse, sys

# ANSI color codes
class Colors:
    RESET  = "\033[0m"
    GREEN  = "\033[92m"
    YELLOW = "\033[93m"
    CYAN   = "\033[96m"
    RED    = "\033[91m"

def open_port(port, baud):
    try:
        s = serial.Serial(port, baudrate=baud, timeout=1.0)
        print(f"{Colors.GREEN}→ Opened {port} @ {baud} baud{Colors.RESET}")
        return s
    except Exception as e:
        print(f"{Colors.RED}✖ Failed to open {port}: {e}{Colors.RESET}")
        sys.exit(1)

def is_nmea(line):
    return re.match(r"^\$[A-Z]{5},.*\*[0-9A-F]{2}$", line)

def parse_nmea(line):
    parts = line.split(',')

    if line.startswith('$GNGGA'):
        fix_map = {
            '0': f"{Colors.RED}Invalid{Colors.RESET}",
            '1': f"{Colors.YELLOW}3D GPS Fix{Colors.RESET}",
            '2': f"{Colors.CYAN}DGPS Fix{Colors.RESET}",
            '4': f"{Colors.GREEN}RTK Fixed{Colors.RESET}",
            '5': f"{Colors.GREEN}RTK Float{Colors.RESET}"
        }
        fix_q = parts[6] if len(parts) > 6 else '0'
        fix_status = fix_map.get(fix_q, f"{Colors.RED}Unknown{Colors.RESET}")

        lat_raw  = parts[2] if len(parts) > 2 else ''
        lat_dir  = parts[3] if len(parts) > 3 else ''
        lon_raw  = parts[4] if len(parts) > 4 else ''
        lon_dir  = parts[5] if len(parts) > 5 else ''

        if lat_raw and lon_raw:
            try:
                lat = float(lat_raw[:2]) + float(lat_raw[2:]) / 60.0
                if lat_dir == 'S': lat = -lat
                lon = float(lon_raw[:3]) + float(lon_raw[3:]) / 60.0
                if lon_dir == 'W': lon = -lon
                print(f"{Colors.CYAN}GGA → Lat={lat:.6f}°, Lon={lon:.6f}°  Fix={fix_status}{Colors.RESET}")
            except:
                print(f"{Colors.RED}GGA parse error (bad lat/lon formatting){Colors.RESET}")
        else:
            print(f"{Colors.RED}GGA has no position yet — waiting for fix{Colors.RESET}")

    elif line.startswith('$GNVTG'):
        try:
            speed_kn = float(parts[5]) if len(parts) > 5 and parts[5] else 0.0
            speed_km = float(parts[7]) if len(parts) > 7 and parts[7] else 0.0
            print(f"{Colors.YELLOW}VTG → {speed_km:.2f} km/h | {speed_kn:.2f} knots{Colors.RESET}")
        except:
            print(f"{Colors.RED}VTG parse error{Colors.RESET}")

def main():
    parser = argparse.ArgumentParser(description="Read GPS NMEA from a serial port")
    parser.add_argument('-p','--port', default='/dev/ttyACM0', help='Serial port (e.g. /dev/ttyACM0)')
    parser.add_argument('-b','--baud', type=int, default=38400, help='Baud rate (default: 38400)')
    args = parser.parse_args()

    ser = open_port(args.port, args.baud)

    try:
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if line and is_nmea(line):
                parse_nmea(line)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}Interrupted, exiting.{Colors.RESET}")
    finally:
        ser.close()
        print(f"{Colors.RED}Closed {args.port}{Colors.RESET}")

if __name__ == '__main__':
    main()
