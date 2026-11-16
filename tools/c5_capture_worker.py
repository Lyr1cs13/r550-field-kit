"""Run the supplied C5 collector with cancellation through its existing STOP path."""
from pathlib import Path
import os
import runpy
import sys

import serial


def main():
    script, stop_file = (Path(value).resolve() for value in sys.argv[1:3])

    class StoppableSerial(serial.Serial):
        def __init__(self, *args, **kwargs):
            if os.name == 'posix':
                kwargs['exclusive'] = True
            super().__init__(*args, **kwargs)
            self.stop_seen = False

        def read(self, size=1):
            if not self.stop_seen and stop_file.exists():
                self.stop_seen = True
                # The original capture loop sends STOP and drains its queue here.
                raise KeyboardInterrupt
            return super().read(size)

    serial.Serial = StoppableSerial
    sys.argv = [str(script), *sys.argv[3:]]
    sys.path.insert(0, str(script.parent))
    runpy.run_path(str(script), run_name='__main__')


if __name__ == '__main__':
    main()
