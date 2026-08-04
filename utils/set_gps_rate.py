#! /usr/bin/env nix-shell
#! nix-shell -i python3 --packages python3
"""
u-blox M8 (Radiolink SE100) UBX-CFG-RATE / UBX-CFG-CFG ayar scripti.

UBX-CFG-RATE ile GPS'in olcum/nav hizini degistirir (varsayilan genelde 1Hz -> 10Hz).
--save ile UBX-CFG-CFG gonderip ayari BBR+Flash+EEPROM'a kalici kaydeder.

Kullanim:
  ./set_gps_rate.py --port /dev/ttyUSB0 --baud 38400 --hz 10 --save
  ./set_gps_rate.py --port /dev/ttyUSB0 --poll
"""
import argparse
import os
import struct
import termios
import time


def ubx_checksum(data: bytes) -> bytes:
    """UBX Fletcher-8 checksum. data = class+id+length+payload (header haric)."""
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])


def build_ubx_cfg_rate(hz: float) -> bytes:
    meas_rate_ms = round(1000.0 / hz)
    nav_rate = 1       # her olcumde bir nav cozumu
    time_ref = 1        # 1 = GPS time referansi

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


def build_ubx_cfg_cfg_save() -> bytes:
    """UBX-CFG-CFG: mevcut RAM ayarlarini BBR+Flash+EEPROM'a kalici kaydeder."""
    clear_mask = 0x00000000
    save_mask = 0xFFFFFFFF   # her seyi kaydet
    load_mask = 0x00000000
    device_mask = 0x01       # BBR(1) + Flash(2) + EEPROM(4)

    payload = struct.pack("<III", clear_mask, save_mask, load_mask) + bytes([device_mask])

    cls = 0x06    # CFG
    msg_id = 0x09  # CFG-CFG
    length = len(payload)

    body = bytes([cls, msg_id]) + struct.pack("<H", length) + payload
    checksum = ubx_checksum(body)

    return bytes([0xB5, 0x62]) + body + checksum


def parse_args():
    parser = argparse.ArgumentParser(description="u-blox M8 UBX-CFG-RATE ayarlayici")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Seri port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=38400, help="Baud rate (default: 38400)")
    parser.add_argument("--hz", type=float, default=10.0, help="Hedef update rate, Hz (default: 10.0)")
    parser.add_argument("--poll", action="store_true", help="Ayarlamak yerine mevcut rate'i sor")
    parser.add_argument("--save", action="store_true",
                         help="Rate ayarindan sonra UBX-CFG-CFG ile kalici kaydet (BBR+Flash+EEPROM)")
    return parser.parse_args()


def send_and_wait_ack(fd, packet: bytes, label: str) -> bool:
    print(f"{label} gonderiliyor: {packet.hex()}")
    os.write(fd, packet)

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
        print(f"  Alinan veri ({len(buf)} byte): {buf.hex()}")
        if b"\xb5\x62\x05\x01" in buf:
            print(f"  {label}: ACK alindi -- kabul edildi.")
            return True
        elif b"\xb5\x62\x05\x00" in buf:
            print(f"  {label}: NAK alindi -- reddedildi.")
            return False
        else:
            print(f"  {label}: ACK/NAK gorulmedi (NMEA cikisiyla karismis olabilir).")
            return False
    else:
        print(f"  {label}: hic cevap gelmedi.")
        return False


def main():
    args = parse_args()

    fd = os.open(args.port, os.O_RDWR)

    attrs = termios.tcgetattr(fd)
    # baud rate ayari (termios sabitleri baud'a gore degisir)
    baud_map = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }
    if args.baud not in baud_map:
        raise ValueError(f"Desteklenmeyen baud: {args.baud}")
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

    if args.poll:
        send_and_wait_ack(fd, build_ubx_cfg_rate_poll(), "POLL")
    else:
        rate_ok = send_and_wait_ack(fd, build_ubx_cfg_rate(args.hz), f"CFG-RATE ({args.hz}Hz)")
        if args.save:
            if rate_ok:
                send_and_wait_ack(fd, build_ubx_cfg_cfg_save(), "CFG-CFG (kalici kaydet)")
            else:
                print("Rate ayari basarisiz oldugu icin kaydetme atlaniyor.")

    os.close(fd)


if __name__ == "__main__":
    main()
