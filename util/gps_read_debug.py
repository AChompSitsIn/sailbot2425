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
        print(f"{Colors.GREEN}Opened {port} @ {baud}{Colors.RESET}")
        return s
    except Exception as e:
        print(f"{Colors.RED}Error opening {port}: {e}{Colors.RESET}")
        sys.exit(1)

def is_nmea(line):
    return re.match(r"^\$[A-Z]{5},.*\*[0-9A-F]{2}$", line)

def parse_and_print(line):
    parts = line.split(',')
    if line.startswith('$GNGGA'):
        # fix map
        fm = {'0':'Invalid','1':'3D GPS','2':'DGPS','4':'RTK Fixed','5':'RTK Float'}
        fix = parts[6] if len(parts)>6 else '0'
        status = fm.get(fix,'Unknown')
        try:
            lat = float(parts[2][:2]) + float(parts[2][2:])/60
            if parts[3]=='S': lat = -lat
            lon = float(parts[4][:3]) + float(parts[4][3:])/60
            if parts[5]=='W': lon = -lon
            print(f"{Colors.CYAN}GGA → Lat:{lat:.6f} Lon:{lon:.6f}  Fix:{status}{Colors.RESET}")
        except:
            print(f"{Colors.RED}GGA parse error{Colors.RESET}")
    elif line.startswith('$GNVTG'):
        try:
            kn = float(parts[5]) if parts[5] else 0.0
            km = float(parts[7]) if parts[7] else 0.0
            print(f"{Colors.YELLOW}VTG → {km:.2f} km/h | {kn:.2f} knots{Colors.RESET}")
        except:
            print(f"{Colors.RED}VTG parse error{Colors.RESET}")
    else:
        # uncomment next line if you want to see *all* valid NMEA
        # print(f"{Colors.GREEN}RAW: {line}{Colors.RESET}")
        pass

def main():
    p = argparse.ArgumentParser()
    p.add_argument('-p','--port', default='/dev/ttyACM0',
                   help='serial port (e.g. /dev/ttyACM0, /dev/ttyUSB0, /dev/ttyAMA0)')
    p.add_argument('-b','--baud', type=int, default=38400, help='baud rate')
    args = p.parse_args()

    ser = open_port(args.port, args.baud)
    try:
        while True:
            raw = ser.readline().decode(errors='ignore').strip()
            if not raw: 
                continue
            if is_nmea(raw):
                parse_and_print(raw)
            else:
                # you can uncomment this to see non-NMEA lines (RTCM, etc)
                # print(f"{Colors.RED}NON-NMEA: {raw}{Colors.RESET}")
                pass
            time.sleep(0.02)
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}Exit{Colors.RESET}")
    finally:
        ser.close()
        print(f"{Colors.RED}Closed {args.port}{Colors.RESET}")

if __name__=='__main__':
    main()
