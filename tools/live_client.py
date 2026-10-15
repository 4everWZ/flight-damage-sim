"""Tick-stamped transport to C++; no flight physics or device keycodes in the core."""
import json
import os
import subprocess
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]

class CoreClient:
    def __init__(self,airframe='validation-bem.airframe'):
        self.process = subprocess.Popen([str(ROOT/'build'/'flight-live'), str(ROOT/'data'/airframe)],
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1,
                                        creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
        try:
            self.state = self._read()
        except BaseException:
            self.close()
            raise

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
        try:
            if self.process.poll() is None:
                try:
                    self.process.stdin.write('quit\n')
                    self.process.stdin.flush()
                except (OSError,ValueError):
                    pass
                try:
                    self.process.wait(timeout=.5)
                except subprocess.TimeoutExpired:
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=.5)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait()
        finally:
            for stream in (self.process.stdin,self.process.stdout):
                try:
                    stream.close()
                except OSError:
                    pass

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
