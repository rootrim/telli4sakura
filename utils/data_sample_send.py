#! /usr/bin/env nix-shell
#! nix-shell -i python3 --packages python3
"""
GRU test scripti — lora_send()'in ürettiği 44 byte'lık paketi
socat ile açılan sanal seri porta (örn. /tmp/ttyV1) yazar.

Kullanım:
  1) socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1
  2) GRU'yu /tmp/ttyV0'ı dinleyecek şekilde çalıştır
  3) python3 lora_test_send.py
"""

import struct
import time

SERIAL_PORT = "/tmp/ttyV1"

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

    # ESP32 little-endian, C float (4 byte) -> Python 'f' little-endian '<f'
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


def main():
    period = 0.2  # 5Hz
    corrupt_every = 1.0
    wave_period = 10.0  

    start = time.monotonic()
    last_corrupt = start

    with open(SERIAL_PORT, "wb", buffering=0) as f:
        while True:
            now = time.monotonic()
            t = now - start

            altitude = altitude_wave(t, low=0.0, high=120.0, period=wave_period)
            pkt = build_packet(
                altitude=altitude,
                pressure=101100.0,
                accel_x=0.1,
                accel_y=0.2,
                accel_z=9.8,
                angle_x=0.0,
                angle_y=0.0,
                angle_z=0.0,
                gps_lat=0.0,
                gps_lon=0.0,
            )

            if now - last_corrupt >= corrupt_every:
                f.write(corrupt_packet(pkt))
                print(f"Sent CORRUPT packet (alt={altitude:.1f})")
                last_corrupt = now
            else:
                f.write(pkt)
                print(f"Sent packet (alt={altitude:.1f})")

            time.sleep(period)


if __name__ == "__main__":
    main()
