#!/usr/bin/env python3
"""
Capture the PX4 NuttX console from power-on, polling a command as it goes.

Written to answer one question: WHERE do the ~50 seconds between powering the
Pixhawk and the ARK Flow's distance_sensor appearing actually go?

The MAVLink console in QGC cannot answer it. It only attaches after PX4 has
booted and MAVLink is up, so the first several seconds -- the interesting
ones -- are already gone, and it has no way to repeat a command on a timer.
The NuttX USB console has neither problem: it is alive from the first
millisecond of boot and it is just a serial port.

USAGE
-----
    1. Plug the Pixhawk's USB port into the Jetson (this is the NuttX console,
       /dev/ttyACM0 -- not the telemetry UART the DDS agent uses).
    2. CLOSE QGROUNDCONTROL and stop the uXRCE-DDS agent. Both will fight for
       the port and you will get a partial log.
    3. Start this script with the Pixhawk POWERED OFF:

           python3 tools/px4_boot_capture.py -o boot.log

    4. Power the Pixhawk on. Let it run for 90 s. Ctrl-C.

Every line is stamped with seconds since the script started, so "when did the
node appear" is answered by reading the timestamp column. `uavcan status` is
re-sent every 2 s automatically, which is the part the MAVLink console cannot
do.

The line to look for is under "Online nodes (Node ID, Health, Mode)": until
the ARK Flow finishes dynamic node-ID allocation that list is empty, and the
timestamp where 125 first shows up is the number we are after.
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed: pip3 install pyserial")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-p', '--port', default='/dev/ttyACM0',
                    help='NuttX console device (default: /dev/ttyACM0)')
    ap.add_argument('-b', '--baud', type=int, default=57600,
                    help='ignored by USB CDC-ACM, kept for real UART consoles')
    ap.add_argument('-o', '--output', default='boot.log',
                    help='file to write the timestamped capture to')
    ap.add_argument('-c', '--command', default='uavcan status',
                    help='command to re-send on a timer (default: uavcan status)')
    ap.add_argument('-i', '--interval', type=float, default=2.0,
                    help='seconds between repeats of that command')
    ap.add_argument('--start-after', type=float, default=3.0,
                    help='seconds to wait after the port opens before the first '
                         'command, so the boot messages are not interleaved with it')
    args = ap.parse_args()

    print(f"Waiting for {args.port} -- power the Pixhawk on now. Ctrl-C to stop.")

    # The port does not exist until the board enumerates, so poll for it. This
    # is what lets you start the script BEFORE powering up, which is the whole
    # point: a capture that begins after boot has already missed the answer.
    ser = None
    while ser is None:
        try:
            ser = serial.Serial(args.port, args.baud, timeout=0.1)
        except (OSError, serial.SerialException):
            time.sleep(0.1)

    t0 = time.monotonic()
    print(f"Port opened at t=0.0 s. Logging to {args.output}.")

    next_cmd = t0 + args.start_after
    buf = b''
    with open(args.output, 'w') as out:
        def emit(text):
            line = f"[{time.monotonic() - t0:8.3f}] {text}"
            print(line)
            out.write(line + "\n")
            out.flush()

        emit("=== capture started ===")
        try:
            while True:
                data = ser.read(4096)
                if data:
                    buf += data
                    # Split on either newline convention; NuttX uses \r\n.
                    while b'\n' in buf:
                        raw, buf = buf.split(b'\n', 1)
                        emit(raw.decode('utf-8', errors='replace').rstrip('\r'))

                now = time.monotonic()
                if now >= next_cmd:
                    next_cmd = now + args.interval
                    emit(f"--> sending: {args.command}")
                    ser.write((args.command + '\r\n').encode())
        except KeyboardInterrupt:
            emit("=== capture stopped ===")
        finally:
            ser.close()

    print(f"\nWrote {args.output}")


if __name__ == '__main__':
    main()
