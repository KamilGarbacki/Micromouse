#!/usr/bin/env python3
"""Captures the Micromouse run log from Serial and saves it to a file.

Opens the given COM port, waits for the "=== LAST RUN LOG ===" marker the
firmware prints at boot (see README "Run log"), and writes everything up to
and including "=== END LOG ===" to the output file. Lines outside that block
(e.g. "cal: ...") are ignored.

Dependency: pyserial (`pip install pyserial`).

Usage:
    python tools/serial_runlog.py <port> [output_file] [--baud 115200]

Example:
    python tools/serial_runlog.py COM5 run.txt
    python tools/serial_runlog.py /dev/ttyUSB0 run.txt
"""

import argparse
import sys

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")

START_MARKER = "=== LAST RUN LOG ==="
END_MARKER = "=== END LOG ==="


def capture(port: str, baud: int, timeout_s: float) -> list[str]:
    """Returns the captured lines (including both markers), or an empty list
    if the timeout is reached before a START/END marker pair is seen (e.g.
    the port only prints the live START RUN / END RUN markers, not a boot)."""
    lines: list[str] = []
    capturing = False
    with serial.Serial(port, baud, timeout=timeout_s) as ser:
        while True:
            raw = ser.readline()
            if not raw:
                break  # timeout: no more data
            line = raw.decode("ascii", errors="replace").rstrip("\r\n")
            if not capturing:
                if START_MARKER in line:
                    capturing = True
                    lines.append(line)
                continue
            lines.append(line)
            if END_MARKER in line:
                break
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("output", nargs="?", default="run_log.txt",
                        help="output file (default: run_log.txt)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="seconds of silence before giving up (default 10)")
    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud, waiting for "
          f"'{START_MARKER}' (reconnect / reset the robot now if needed)...")
    lines = capture(args.port, args.baud, args.timeout)
    if not lines or END_MARKER not in lines[-1]:
        print("Did not see a complete run log before the timeout.",
              file=sys.stderr)
        return 1

    with open(args.output, "w", encoding="ascii") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Wrote {len(lines)} lines to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
