#! /usr/bin/env nix-shell
#! nix-shell -i python3 --packages python3
"""
Testing GRU
Usage:
  1) socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1
  2) run gru in a way that allows it to listen /tmp/ttyV0
  3) ./data_sample_send.py --port /tmp/ttyV1
Testing LoRa
Usage:
  1) Connect lora via a serial port (e.g. /dev/ttyUSB0)
  2) Set the port up `stty -F /dev/ttyUSB0 9600 cs8 -cstopb -parenb raw -echo`
  2) ./data_sample_send.py --port /dev/ttyUSB0
"""

import argparse
import struct
import time

LORA_HEADER = 0xAB
LORA_FOOTER_1 = 0x0D
LORA_FOOTER_2 = 0x0A
PACKET_SIZE = 44


def calc_checksum(buf: bytes) -> int:
    return sum(buf) & 0xFF


def build_packet(altitude, pressure, accel_x, accel_y, accel_z,
                  angle_x, angle_y, angle_z, gps_lat, gps_lon) -> bytes:
    packet = bytearray(PACKET_SIZE)
    packet[0] = LORA_HEADER

    # little-endian
    floats = struct.pack("<10f", altitude, pressure, accel_x, accel_y, accel_z,
                          angle_x, angle_y, angle_z, gps_lat, gps_lon)
    packet[1:41] = floats
    packet[41] = calc_checksum(bytes(packet[0:41]))
    packet[42] = LORA_FOOTER_1
    packet[43] = LORA_FOOTER_2
    return bytes(packet)


def altitude_wave(t: float, low: float = 0.0, high: float = 120.0, period: float = 10.0) -> float:
    phase = (t % period) / period
    if phase < 0.5:
        frac = phase / 0.5
    else:
        frac = 1.0 - (phase - 0.5) / 0.5
    return low + (high - low) * frac


def corrupt_packet(pkt: bytes) -> bytes:
    corrupted = bytearray(pkt)
    corrupted[41] = (corrupted[41] + 1) & 0xFF
    return bytes(corrupted)


def parse_args():
    parser = argparse.ArgumentParser(description="LoRa GRU test paket göndericisi")
    parser.add_argument("--port", default="/dev/ttyUSB1",
                         help="Serial port device (default: /dev/ttyUSB1)")
    parser.add_argument("--hz", type=float, default=5.0,
                         help="Tx frequency, Hz (default: 5.0)")
    parser.add_argument("--corrupt-every", type=float, default=1.0,
                         help="How many seconds can a broken package be sent (default: 1.0, 0 = never)")
    parser.add_argument("--alt-low", type=float, default=0.0,
                         help="Pyramid wave minimum, meter (default: 0.0)")
    parser.add_argument("--alt-high", type=float, default=120.0,
                         help="Pyramid wave max, meter (default: 120.0)")
    parser.add_argument("--wave-period", type=float, default=10.0,
                         help="Pyramid wave period, seconds (default: 10.0)")
    parser.add_argument("--pressure", type=float, default=101100.0,
                         help="Anchor pressure value, Pa (default: 101100.0)")
    parser.add_argument("--quiet", action="store_true",
                         help="Should log?")
    return parser.parse_args()


def main():
    args = parse_args()
    period = 1.0 / args.hz

    start = time.monotonic()
    last_corrupt = start

    with open(args.port, "wb", buffering=0) as f:
        print(f"Sending with {args.hz}Hz by {args.port} (Ctrl+C to exit)")
        try:
            while True:
                now = time.monotonic()
                t = now - start
                altitude = altitude_wave(t, low=args.alt_low, high=args.alt_high,
                                          period=args.wave_period)
                pkt = build_packet(
                    altitude=altitude,
                    pressure=args.pressure,
                    accel_x=0.1,
                    accel_y=0.2,
                    accel_z=9.8,
                    angle_x=0.0,
                    angle_y=0.0,
                    angle_z=0.0,
                    gps_lat=0.0,
                    gps_lon=0.0,
                )

                is_corrupt = args.corrupt_every > 0 and (now - last_corrupt >= args.corrupt_every)
                if is_corrupt:
                    f.write(corrupt_packet(pkt))
                    last_corrupt = now
                    if not args.quiet:
                        print(f"Sent CORRUPT packet (alt={altitude:.1f})")
                else:
                    f.write(pkt)
                    if not args.quiet:
                        print(f"Sent packet (alt={altitude:.1f})")

                time.sleep(period)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
