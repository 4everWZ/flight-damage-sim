#!/usr/bin/env python3
"""Optional 2D effective-component polar capability, not aircraft calibration."""
import bisect
import csv
import json
import math
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results'
BASE = 'f0fa41e3190074b09a15b3cfae67ebb3d7b67eb5'


def main():
    OUT.mkdir(exist_ok=True)
    subprocess.run(['make', '-j2', 'build/control-polar-tests', 'build/flight-run', 'build/flight-live'], cwd=ROOT, check=True)
    subprocess.run(['build/control-polar-tests', 'data/validation-material.airframe',
                    'results/control-polar-summary.json'], cwd=ROOT, check=True)
    # Compile the accepted pre-change core against the current value-type
    # headers. No embedded old implementation or network install is needed.
    for file, target in [('src/core.cpp', 'core-before.cpp'), ('src/live.cpp', 'live-before.cpp')]:
        source = subprocess.check_output(['git', 'show', f'{BASE}:{file}'], cwd=ROOT)
        (ROOT/'build'/target).write_bytes(source)
    flags = shlex.split(os.environ.get('CXX', 'g++')) + ['-Iinclude'] + shlex.split(
        os.environ.get('CXXFLAGS', '-std=c++20 -O2 -g -Wall -Wextra -Wpedantic'))
    subprocess.run(flags + ['-c', 'build/core-before.cpp', '-o', 'build/core-before.o'], cwd=ROOT, check=True)
    for source, binary in [('src/runner.cpp', 'flight-run-before'), ('build/live-before.cpp', 'flight-live-before')]:
        subprocess.run(flags + ['build/core-before.o', 'build/propeller.o', source, '-o', 'build/'+binary], cwd=ROOT, check=True)

    text = (ROOT/'data/validation-material.airframe').read_text()
    tables = {}
    for line in text.splitlines():
        f = line.split()
        if f and f[0] == 'curve':
            tables.setdefault(f[1], []).append([float(v) for v in f[2:]])
    records = []
    for name, points in tables.items():
        xs = [p[0] for p in points]
        for delta in (-30, -15, 0, 15, 30):
            for alpha in xs:
                x = min(xs[-1], max(xs[0], alpha+.8*delta))
                i = max(1, min(len(xs)-1, bisect.bisect_right(xs, x)))
                weight = (x-xs[i-1])/(xs[i]-xs[i-1])
                cl, cd, cm = [(1-weight)*a+weight*b for a, b in zip(points[i-1][1:], points[i][1:])]
                # Declared synthetic family: tabulated old shifted coefficients
                # plus independent control CM/drag. No production table is adopted.
                cd += .01*math.radians(delta)**2
                cm += .01*math.radians(delta)
                records.append(f'control_polar {name}_demo {alpha} {delta} {cl:.17g} {cd:.17g} {cm:.17g}')
    unbound = ROOT/'build/control-polar-unbound.airframe'
    unbound.write_text(text+'\n'+'\n'.join(records)+'\n')
    bindings = ['cell_control_polar '+f[1]+' '+f[-1]+'_demo' for line in text.splitlines()
                if (f := line.split()) and f[0] == 'cell' and int(f[3]) >= 0]
    enabled = ROOT/'build/control-polar-enabled.airframe'
    enabled.write_text(unbound.read_text()+'\n'.join(bindings)+'\n')

    equality, flights = {}, {}
    for scenario, seconds in [('clean', 4), ('mixed', 4), ('stall', 10), ('highspeed_weakened', 6)]:
        prefixes = {}
        for variant, path, binary in [('before', ROOT/'data/validation-material.airframe', 'flight-run-before'),
                                      ('default', ROOT/'data/validation-material.airframe', 'flight-run'),
                                      ('unbound', unbound, 'flight-run'), ('enabled', enabled, 'flight-run')]:
            prefix = OUT/f'control-polar-{variant}-{scenario}'
            entry = subprocess.check_output(['build/'+binary, str(path), scenario, str(prefix), '120', str(seconds)], cwd=ROOT, text=True)
            prefixes[variant] = prefix
            if variant == 'enabled':
                with prefix.with_suffix('.csv').open() as stream:
                    rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(stream)]
                assert all(math.isfinite(v) for r in rows for v in r.values())
                if scenario in ('clean', 'mixed'):
                    assert 'converged=1' in entry
                flights[scenario] = {'entry': entry.strip(), 'seconds': seconds,
                                     'peak_legacy_stall_proxy_cells': max(r['stalled'] for r in rows),
                                     'first_failure_s': next((r['time'] for r in rows if r['failed']), None),
                                     'first_separation_s': next((r['time'] for r in rows if r['debris']), None)}
                if scenario == 'clean':
                    assert all(r['failed'] == 0 for r in rows)
                if scenario == 'highspeed_weakened':
                    event = flights[scenario]
                    assert 0 < event['first_failure_s'] < event['first_separation_s']
                if scenario == 'mixed':
                    assert all(r['roll_cmd'] == -1 and r['roll_owner'] == 1 and r['pitch_owner'] == 0 and
                               r['yaw_owner'] == 0 for r in rows if 1.02 < r['time'] < 1.99)
                    before = min(rows, key=lambda r: abs(r['time']-2))
                    released = next(r for r in rows if r['time'] > 2)
                    assert released['roll_cmd'] == before['roll_cmd']
        for variant in ('default', 'unbound'):
            same = all(prefixes[variant].with_suffix(s).read_bytes() == prefixes['before'].with_suffix(s).read_bytes()
                       for s in ('.csv', '.cells.csv', '.edges.csv'))
            assert same, (scenario, variant)
            equality[f'{variant}-{scenario}'] = same
    def transport(binary, path, commands='reset\nquit\n'):
        return subprocess.check_output(['build/'+binary, str(path)], cwd=ROOT, input=commands, text=True)
    native = ROOT/'data/validation-material.airframe'
    live_equal = transport('flight-live-before', native) == transport('flight-live', native)
    assert live_equal
    failure = ''.join(f'damage spar left_root_{part} .01\n' for part in ('front', 'rear', 'skin'))
    failure += 'step 1 1 0 0 0 0 0 0 0 0 0 .7 0 0 0\nstep 2 1 0 0 0 0 0 0 0 0 0 .7 0 0 0\nquit\n'
    detached_equal = transport('flight-live-before', native, failure) == transport('flight-live', native, failure)
    assert detached_equal
    snapshots = [json.loads(line) for line in transport('flight-live', enabled).splitlines()]
    for s in snapshots:
        optional = [c for c in s['cells'] if 'control_polar' in c]
        assert len(optional) == len(bindings)
        assert all(c['stall_diagnostic'] == 'legacy_effective_alpha_15deg' and
                   math.isfinite(c['flow_alpha']) and math.isfinite(c['delta_eff']) for c in optional)
    separated = [json.loads(line) for line in transport('flight-live', enabled, failure).splitlines()]
    assert separated[-1]['detached']
    piece = separated[-1]['detached'][0]
    assert all(not c['connected'] for c in piece['chains'])
    optional_piece_cells = [c for c in piece['cells'] if 'control_polar' in c]
    assert optional_piece_cells and all('flow_alpha' in c and 'control_polar_clamped' in c for c in optional_piece_cells)
    result = {'baseline': BASE, 'complete_csv_identity': equality, 'default_live_json_equal': live_equal,
              'default_detached_live_json_equal': detached_equal,
              'detached_optional_telemetry_cells': len(optional_piece_cells),
              'optional_telemetry_cells': len(bindings), 'dynamic_simulated_flights': flights,
              'meaning': 'Synthetic capability; no historical performance comparison or physical reversal validation.',
              'fixture': 'Original fixture unchanged. Ignored demo tables tabulate its shifted effective coefficients with independent CM=.01*delta and CD increment=.01*delta^2 at specified grid nodes. This is not measured sectional data.',
              'limits': 'Only explicit cell bindings activate 2D lookup. Legacy alpha/stall proxy is retained and labeled; no separation-state, downwash or dynamic-stall model is added. Bounds clamp and report, without extrapolation.'}
    (OUT/'control-polar-flight-summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
