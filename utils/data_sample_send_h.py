#! /usr/bin/env nix-shell
#! nix-shell -i python3 --packages python3

import argparse
import random
import struct
import time

LORA_HEADER = 0xAB
LORA_FOOTER_1 = 0x0D
LORA_FOOTER_2 = 0x0A
PACKET_SIZE = 44


def calc_checksum(buf: bytes) -> int:
    return sum(buf) & 0xFF


def build_packet(
    altitude,
    pressure,
    accel_x,
    accel_y,
    accel_z,
    angle_x,
    angle_y,
    angle_z,
    gps_lat,
    gps_lon,
) -> bytes:
    packet = bytearray(PACKET_SIZE)

    packet[0] = LORA_HEADER

    floats = struct.pack(
        "<10f",
        altitude,
        pressure,
        accel_x,
        accel_y,
        accel_z,
        angle_x,
        angle_y,
        angle_z,
        gps_lat,
        gps_lon,
    )

    packet[1:41] = floats

    packet[41] = calc_checksum(packet[0:41])
    packet[42] = LORA_FOOTER_1
    packet[43] = LORA_FOOTER_2

    return bytes(packet)


def corrupt_packet(pkt: bytes) -> bytes:
    corrupted = bytearray(pkt)
    corrupted[41] = (corrupted[41] + 1) & 0xFF
    return bytes(corrupted)


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--port", default="/dev/ttyUSB1")
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--corrupt-every", type=float, default=0.0)
    parser.add_argument("--quiet", action="store_true")

    return parser.parse_args()


def main():
    args = parse_args()

    period = 1.0 / args.hz

    # -----------------------------
    # Gerçek (ideal) değerler
    # -----------------------------
    TRUE_ALTITUDE = 0
    TRUE_PRESSURE = 0

    TRUE_ACCEL_X = 0
    TRUE_ACCEL_Y = 0
    TRUE_ACCEL_Z = 0

    TRUE_ANGLE_X = 0
    TRUE_ANGLE_Y = 0
    TRUE_ANGLE_Z = 0

    TRUE_GPS_LAT = 41.01642856878581
    TRUE_GPS_LON = 29.220506690302834

    # -----------------------------
    # Noise (Standart Sapma)
    # -----------------------------
    ALTITUDE_NOISE = 0.0      # metre
    PRESSURE_NOISE = 0.0       # Pa

    ACCEL_NOISE = 0.0         # m/s²
    ANGLE_NOISE = 0.0         # derece

    GPS_NOISE = 0.000001        # yaklaşık 1 metre

    last_corrupt = time.monotonic()

    with open(args.port, "wb", buffering=0) as f:
        print(f"Sending to {args.port} ({args.hz} Hz)")

        try:
            while True:
                now = time.monotonic()

                altitude = TRUE_ALTITUDE + random.gauss(0, ALTITUDE_NOISE)
                pressure = TRUE_PRESSURE + random.gauss(0, PRESSURE_NOISE)

                accel_x = TRUE_ACCEL_X + random.gauss(0, ACCEL_NOISE)
                accel_y = TRUE_ACCEL_Y + random.gauss(0, ACCEL_NOISE)
                accel_z = TRUE_ACCEL_Z + random.gauss(0, ACCEL_NOISE)

                angle_x = TRUE_ANGLE_X + random.gauss(0, ANGLE_NOISE)
                angle_y = TRUE_ANGLE_Y + random.gauss(0, ANGLE_NOISE)
                angle_z = TRUE_ANGLE_Z + random.gauss(0, ANGLE_NOISE)

                gps_lat = TRUE_GPS_LAT + random.gauss(0, GPS_NOISE)
                gps_lon = TRUE_GPS_LON + random.gauss(0, GPS_NOISE)

                pkt = build_packet(
                    altitude,
                    pressure,
                    accel_x,
                    accel_y,
                    accel_z,
                    angle_x,
                    angle_y,
                    angle_z,
                    gps_lat,
                    gps_lon,
                )

                is_corrupt = (
                    args.corrupt_every > 0
                    and now - last_corrupt >= args.corrupt_every
                )

                if is_corrupt:
                    f.write(corrupt_packet(pkt))
                    last_corrupt = now

                    if not args.quiet:
                        print("Sent CORRUPT packet")
                else:
                    f.write(pkt)

                    if not args.quiet:
                        print(
                            f"ALT={altitude:6.2f} "
                            f"P={pressure:8.1f} "
                            f"AX={accel_x:+.2f} "
                            f"AY={accel_y:+.2f} "
                            f"AZ={accel_z:+.2f}"
                        )

                time.sleep(period)

        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
