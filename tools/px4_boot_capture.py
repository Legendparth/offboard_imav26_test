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
    3. Set SYS_USB_AUTO = 1 (Auto-detect) in QGC and reboot the board. The
       default is 2 (MAVLink), which makes the USB port speak MAVLink binary
       and there is no console to capture.
    4. Start this script with the Pixhawk POWERED OFF:

           python3 tools/px4_boot_capture.py -o boot.log

    5. Power the Pixhawk on. Let it run for 90 s. Ctrl-C.

WAKING THE CONSOLE
------------------
With SYS_USB_AUTO=1 the port starts out undecided and only becomes an nsh
console once PX4 sees THREE CONSECUTIVE CARRIAGE RETURNS:

    cdcacm_autostart.cpp:465
        if (_buffer[i - 1] == 0xD && _buffer[i] == 0xD && _buffer[i + 1] == 0xD) {
            PX4_INFO("%s: launching nshterm", USB_DEVICE_PATH);

Note 0xD, not 0xA -- sending "cmd\r\n" has a single CR in it and will never
match, which looks exactly like a dead port. So this script sends a bare
"\r\r\r" every --wake-interval seconds until the board answers with
something, and only then starts polling the command.

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
                    help='seconds to wait after the console wakes before the first '
                         'command, so the boot messages are not interleaved with it')
    ap.add_argument('--wake-interval', type=float, default=1.0,
                    help='seconds between the \\r\\r\\r bursts that make '
                         'SYS_USB_AUTO=1 hand over an nsh console')
    ap.add_argument('--no-wake', action='store_true',
                    help='skip the wake bursts (use on a real debug UART, where '
                         'the console is already there)')
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

    # Undecided until the board says something back. Until then we send CR
    # bursts rather than commands: a command sent into a port that is still
    # in MAVLink mode is silently discarded.
    woken = args.no_wake
    next_wake = t0
    next_cmd = t0 + args.start_after if woken else None
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
                    if not woken:
                        woken = True
                        next_cmd = time.monotonic() + args.start_after
                        emit("=== console responded; nsh is up ===")
                    buf += data
                    # Split on either newline convention; NuttX uses \r\n.
                    while b'\n' in buf:
                        raw, buf = buf.split(b'\n', 1)
                        emit(raw.decode('utf-8', errors='replace').rstrip('\r'))

                now = time.monotonic()

                if not woken:
                    if now >= next_wake:
                        next_wake = now + args.wake_interval
                        emit("--> waking console (3x CR)")
                        ser.write(b'\r\r\r')
                    continue

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
