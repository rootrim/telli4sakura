#! /usr/bin/env nix-shell
#! nix-shell -i python3 --packages python3
"""
u-blox M8 (Radiolink SE100) UBX-CFG-RATE setting script.

Change GPS's data rate using UBX-CFG-RATE protocol thingy

Usage:
  ./set_gps_rate.py --port /dev/ttyUSB0 --baud 38400 --hz 10
"""
import argparse
import struct
import time


def ubx_checksum(data: bytes) -> bytes:
    """UBX Fletcher-8 checksum. data = class+id+length+payload (header not included)."""
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])


def build_ubx_cfg_rate(hz: float) -> bytes:
    meas_rate_ms = round(1000.0 / hz)
    nav_rate = 1       
    time_ref = 1      

    payload = struct.pack("<HHH", meas_rate_ms, nav_rate, time_ref)

    cls = 0x06   # CFG
    msg_id = 0x08  # RATE
    length = len(payload)

    body = bytes([cls, msg_id]) + struct.pack("<H", length) + payload
    checksum = ubx_checksum(body)

    packet = bytes([0xB5, 0x62]) + body + checksum
    return packet


def build_ubx_cfg_rate_poll() -> bytes:
    """Mevcut rate ayarini sormak icin (payload=0, poll request)."""
    cls = 0x06
    msg_id = 0x08
    length = 0
    body = bytes([cls, msg_id]) + struct.pack("<H", length)
    checksum = ubx_checksum(body)
    return bytes([0xB5, 0x62]) + body + checksum


def parse_args():
    parser = argparse.ArgumentParser(description="u-blox M8 UBX-CFG-RATE ayarlayici")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Seri port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=38400, help="Baud rate (default: 38400)")
    parser.add_argument("--hz", type=float, default=10.0, help="Hedef update rate, Hz (default: 10.0)")
    parser.add_argument("--poll", action="store_true", help="Ayarlamak yerine mevcut rate'i sor")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.poll:
        packet = build_ubx_cfg_rate_poll()
        print(f"Poll komutu gonderiliyor: {packet.hex()}")
    else:
        packet = build_ubx_cfg_rate(args.hz)
        print(f"{args.hz}Hz icin UBX-CFG-RATE gonderiliyor: {packet.hex()}")

    import os
    fd = os.open(args.port, os.O_RDWR)

    import termios
    attrs = termios.tcgetattr(fd)
    # baud rate setting 
    baud_map = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }
    if args.baud not in baud_map:
        raise ValueError(f"Unsupported baud: {args.baud}")
    b = baud_map[args.baud]
    attrs[4] = b  # ispeed
    attrs[5] = b  # ospeed
    # raw mod
    attrs[0] = 0  # iflag
    attrs[1] = 0  # oflag
    attrs[3] = 0  # lflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    termios.tcflush(fd, termios.TCIOFLUSH)

    os.write(fd, packet)
    print("Send, waiting answers (2 seconds)...")

    start = time.time()
    buf = b""
    while time.time() - start < 2.0:
        try:
            chunk = os.read(fd, 256)
            buf += chunk
        except BlockingIOError:
            pass
        time.sleep(0.05)

    if buf:
        print(f"Alinan veri ({len(buf)} byte): {buf.hex()}")
        # ACK-ACK (0x05 0x01) or ACK-NAK (0x05 0x00) searching
        if b"\xb5\x62\x05\x01" in buf:
            print("ACK alindi -- command got accepted.")
        elif b"\xb5\x62\x05\x00" in buf:
            print("NAK alindi -- command got denied.")
        else:
            print("Couldn't see UBX ACK/NAK , but we have some data (Probably got confused by NMEA datas).")
    else:
        print("No answer.")

    os.close(fd)


if __name__ == "__main__":
    main()
