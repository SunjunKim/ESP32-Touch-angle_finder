"""Read-only serial capture using POSIX APIs; no firmware upload or commands."""
import argparse
import datetime
import json
import os
from pathlib import Path
import select
import termios
import time
import tty

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--duration', type=int, default=60)
parser.add_argument('--port', required=True, help='Serial device path (115200 baud)')
args = parser.parse_args()
if not 1 <= args.duration <= 60:
    parser.error('--duration must be between 1 and 60 seconds')
duration = args.duration
port = args.port
fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
old = termios.tcgetattr(fd)
try:
    tty.setraw(fd)
    settings = termios.tcgetattr(fd)
    settings[4] = settings[5] = termios.B115200
    settings[2] |= termios.CLOCAL | termios.CREAD
    termios.tcsetattr(fd, termios.TCSANOW, settings)
    output_dir = Path(__file__).resolve().parent / 'hardware_logs'
    output_dir.mkdir(exist_ok=True)
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    path = output_dir / f'stationary-{duration}s-{stamp}.jsonl'
    start = time.monotonic()
    next_progress = 15
    buffer = b''
    count = 0
    print(f'CAPTURE_STARTED port={port} duration={duration}s path={path}', flush=True)
    with path.open('x') as out:
        while time.monotonic() - start < duration:
            if select.select([fd], [], [], .25)[0]:
                chunk = os.read(fd, 65536)
                if not chunk:
                    raise RuntimeError('Serial disconnected')
                buffer += chunk
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    text = line.decode('utf-8', errors='replace').rstrip('\r')
                    elapsed = time.monotonic() - start
                    out.write(json.dumps({'elapsed_s': round(elapsed, 4), 'line': text}, ensure_ascii=False) + '\n')
                    out.flush()
                    count += 1
                    if count <= 4:
                        print(text, flush=True)
            elapsed = time.monotonic() - start
            if elapsed >= next_progress:
                print(f'PROGRESS {elapsed:.1f}s lines={count}', flush=True)
                next_progress += 15
        if buffer:
            out.write(json.dumps({'elapsed_s': round(time.monotonic()-start, 4), 'partial': buffer.decode('utf-8', errors='replace')})+'\n')
    print(f'CAPTURE_COMPLETE seconds={time.monotonic()-start:.2f} lines={count} path={path}', flush=True)
finally:
    termios.tcsetattr(fd, termios.TCSANOW, old)
    os.close(fd)
