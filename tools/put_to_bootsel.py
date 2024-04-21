#!/usr/bin/env python3

from serial import Serial
import time
import sys

def put_into_bootsel(port: str) -> None:
    try:
        com = Serial(port, baudrate=1200, dsrdtr=True)
        com.dtr = False
        com.rts = False
        time.sleep(0.2)
        com.close()
    except OSError:
        # Typically we get "OSError: [Errno 71] Protocol error", but no worries
        pass


if __name__ == '__main__':
    put_into_bootsel(sys.argv[1])
