#!/usr/bin/env python3

from serial import Serial
import sys
import time
import struct


if __name__ == '__main__':
    port = sys.argv[1]
    baud = 115200
    s = Serial(port, baudrate=baud, timeout=1)
    s.flush()
    i = 0

    while True:
        print(f'Waiting for initial data to be received')

        while True:
            if s.in_waiting > 0:
                file = open(f'./current{i}.bin', 'wb')
                t0 = time.time()
                length = 0
                print(f'First data received, saving to {file.name}')
                while True:
                    data = s.read(1024)
                    if data:
                        file.write(data)
                        length += len(data)
                        sys.stdout.write(f'\r{length} bytes ')
                        sys.stdout.flush()
                    else:
                        break

                print(f'\nDone, total size: {length} bytes')
            else:
                time.sleep(0.5)