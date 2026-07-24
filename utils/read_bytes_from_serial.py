#! /usr/bin/env nix-shell
#! nix-shell -i python3 --packages python3

import os
import sys
import termios

def main():
    device = sys.argv[1]
    num_bytes = int(sys.argv[2]) if len(sys.argv) > 2 else 64

    fd = os.open(device, os.O_RDONLY)
    termios.tcflush(fd, termios.TCIFLUSH)

    try: 
        while True:
            data = os.read(fd, num_bytes)
            if data:
                print(data.hex())
    except KeyboardInterrupt:
        print("\nUser exit")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()

