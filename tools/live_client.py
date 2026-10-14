"""Tick-stamped transport to C++; no flight physics or device keycodes in the core."""
import json
import subprocess
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]

class CoreClient:
    def __init__(self,airframe='validation-bem.airframe'):
        self.process = subprocess.Popen([str(ROOT/'build'/'flight-live'), str(ROOT/'data'/airframe)],
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.state = self._read()

    def _read(self):
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError('C++ core closed its telemetry stream')
        state = json.loads(line)
        if 'error' in state:
            raise ValueError(state['error'])
        self.state = state
        return state

    def command(self, text):
        self.process.stdin.write(text+'\n')
        self.process.stdin.flush()
        return self._read()

    def step(self, ticks, aim=(1,0,0), direct=(0,0,0), active=(False,False,False), throttle=None, trim=None, mouse=True,prop_rpm_ratio=1):
        if throttle is None:
            throttle = self.state['throttle']
        if trim is None:
            trim = self.state['trim']
        values = [ticks, *aim, int(mouse), *direct, *(int(a) for a in active), throttle, *trim,prop_rpm_ratio]
        return self.command('step '+' '.join(str(v) for v in values))

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.write('quit\n')
            self.process.stdin.flush()
            self.process.wait(timeout=5)
        self.process.stdin.close()
        self.process.stdout.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
