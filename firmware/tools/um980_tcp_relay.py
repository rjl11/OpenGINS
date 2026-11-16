#!/usr/bin/env python3
"""Raw byte relay: a serial port <-> one TCP client. Runs on miniPi.

The other half of um980_bridge.py (see there): exposes the UM980's USB-C
(COM3, /dev/ttyUSB0) on TCP for the bridge on dietpi. No parsing, no
rewriting. Uses termios only (no pyserial on miniPi). Stays at 115200: the
bridge keeps the receiver's COM3 there, because miniPi's CH340 loses bytes
at 460800.

    nohup nice -n 10 python3 um980_tcp_relay.py > relay.log 2>&1 &
"""
import argparse
import os
import select
import socket
import termios
import time

BAUDS = {9600: termios.B9600, 19200: termios.B19200, 38400: termios.B38400,
         57600: termios.B57600, 115200: termios.B115200, 230400: termios.B230400,
         460800: termios.B460800}


def open_serial(path, baud):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = 0                                    # iflag
    a[1] = 0                                    # oflag
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[3] = 0                                    # lflag
    a[4] = a[5] = BAUDS[baud]
    a[6][termios.VMIN] = 0
    a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def write_all(fd, d):
    """The tty is non-blocking: an RTCM burst can fill its buffer, so wait
    for room instead of losing the tail of a frame."""
    while d:
        try:
            d = d[os.write(fd, d):]
        except BlockingIOError:
            select.select([], [fd], [], 1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--port", type=int, default=5017)
    args = ap.parse_args()

    fd = open_serial(args.dev, args.baud)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(1)
    print(f"relay {args.dev}@{args.baud} on tcp :{args.port}", flush=True)

    while True:
        cli, addr = srv.accept()
        cli.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"{time.strftime('%H:%M:%S')} client {addr}", flush=True)
        termios.tcflush(fd, termios.TCIFLUSH)
        up = down = 0
        try:
            while True:
                r, _, _ = select.select([fd, cli], [], [], 1.0)
                if fd in r:
                    d = os.read(fd, 4096)
                    if d:
                        cli.sendall(d)
                        down += len(d)
                if cli in r:
                    d = cli.recv(4096)
                    if not d:
                        break
                    write_all(fd, d)  # command lines and RTCM frames from the bridge
                    up += len(d)
        except OSError as e:
            print(f"client error: {e}", flush=True)
        cli.close()
        print(f"{time.strftime('%H:%M:%S')} client gone, {down} B down, {up} B up", flush=True)


if __name__ == "__main__":
    main()
