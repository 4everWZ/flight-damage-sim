#!/usr/bin/env python3
"""Identity compatibility and real flights with opt-in synthetic canted cells."""
import csv
import json
import math
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT/'results'


def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make', '-j2', 'build/flight-run', 'build/cell-frame-tests'], cwd=ROOT, check=True)
    subprocess.run([str(ROOT/'build/cell-frame-tests'), str(ROOT/'data/validation-material.airframe'),
                    str(RESULTS/'cell-frame-summary.json')], cwd=ROOT, check=True)
    source = (ROOT/'data/validation-material.airframe').read_text()
    cells = [line.split() for line in source.splitlines() if line.startswith('cell ')]
    files = {'omitted': ROOT/'data/validation-material.airframe'}
    for variant in ('identity', 'negative_identity', 'canted'):
        records = []
        for cell in cells:
            if variant == 'canted':
                if cell[-1] != 'wing':
                    continue
                side = 1 if float(cell[6]) > 0 else -1  # cell datum y
                values = [math.cos(math.radians(3)), -side*math.sin(math.radians(3)), 0, 0]
            else:
                values = [1 if variant == 'identity' else -1, 0, 0, 0]
            records.append('cell_basis '+cell[1]+' '+' '.join(format(x, '.17g') for x in values))
        path = ROOT/'build'/f'cell-frame-{variant}.airframe'
        path.write_text(source+'\n'+'\n'.join(records)+'\n')
        files[variant] = path

    records, exact = {}, {}
    for variant, path in files.items():
        records[variant] = {}
        for scenario, seconds in (('clean',4),('mixed',4),('stall',10),('highspeed_weakened',6)):
            prefix = RESULTS/f'cell-frame-{variant}-{scenario}'
            entry = subprocess.check_output([str(ROOT/'build/flight-run'), str(path), scenario,
                                             str(prefix), '120', str(seconds)], text=True)
            with prefix.with_suffix('.csv').open() as stream:
                rows = [{k:float(v) for k,v in row.items()} for row in csv.DictReader(stream)]
            assert all(math.isfinite(v) for row in rows for v in row.values())
            if scenario != 'highspeed_weakened':
                assert 'converged=1' in entry
            if scenario == 'mixed':
                override = all(r['roll_cmd']==-1 and r['roll_owner']==1 and
                               r['pitch_owner']==0 and r['yaw_owner']==0
                               for r in rows if 1.02<r['time']<1.99)
                before = min(rows,key=lambda r:abs(r['time']-2))
                released = next(r for r in rows if r['time']>2)
                release_jump = abs(released['roll_cmd']-before['roll_cmd'])
                assert override and release_jump==0
            if scenario == 'stall':
                assert max(r['stalled'] for r in rows)>0 and rows[-1]['stalled']==0
            records[variant][scenario] = {
                'entry': entry.strip(), 'hz':120, 'seconds':seconds,
                'peak_aero_body_z_g':max(r['g'] for r in rows),
                'peak_stalled_cells':max(r['stalled'] for r in rows),
                'final_stalled_cells':rows[-1]['stalled'],
                'final_p_rad_s':rows[-1]['p'],
                'final_mass_kg':rows[-1]['mass'],
                'first_failure_s':next((r['time'] for r in rows if r['failed']),None),
                'separation_s':next((r['time'] for r in rows if r['debris']),None)}
            if scenario == 'mixed':
                records[variant][scenario].update(full_axis_override=override,release_request_jump=release_jump)
            if variant in ('identity','negative_identity'):
                same = all(prefix.with_suffix(suffix).read_bytes() ==
                           (RESULTS/f'cell-frame-omitted-{scenario}').with_suffix(suffix).read_bytes()
                           for suffix in ('.csv','.cells.csv','.edges.csv'))
                exact[variant+'_'+scenario] = same
                assert same, 'zero-angle definitions must preserve complete telemetry'
    result = {'meaning':'Synthetic geometry capability; no NACA flight-performance calibration.',
              'canted_definition':'Only wing cell bases rotate by left +6/right -6 degrees about body x; all original positions, incidences, coefficients, masses, controls and material values retained.',
              'historical_performance_comparisons_run':0,
              'identity_complete_telemetry_equal':exact, 'actual_flights':records,
              'entry_note':'140 m/s entry is a non-equilibrium transient, never labeled sustained cruise. The stall scenario is a powered low-speed transient, not the report throttle-closed glide. G is aerodynamic body-Z force/weight, not flight-path normal load.'}
    (RESULTS/'cell-frame-flight-summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
