#!/usr/bin/env python3
"""Read-only UART handshake for a stopped R550 and a SERVO_CAR_2 Client."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'demo'))

from collection import open_ws63
from servo_car import CarLink


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='auto', help='auto or the verified WS63 /dev/serial/by-id/... port')
    args = parser.parse_args()
    try:
        stream, device = open_ws63(args.device)
        with stream:
            link = CarLink(stream)
            # Keep the exact wire replies available when aligning firmware.
            link.on_line = lambda line: print(line, file=sys.stderr) if line.startswith('+CAR') else None
            hello = link.hello()
            status = link.status()
        print(json.dumps({'device': device, 'hello': hello, **asdict(status),
                          'can_start': status.can_start}, indent=2))
        return 0 if status.can_start else 1
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'串口探测失败：{exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
