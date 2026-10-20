#!/usr/bin/env python3
"""Fixed-throttle equilibria and real transient flights; synthetic capability only."""
import csv
import json
import math
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'results'


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main():
    OUT.mkdir(exist_ok=True)
    subprocess.run(['make', '-j2', 'build/glide-trim-tests', 'build/glide-probe'], cwd=ROOT, check=True)
    subprocess.run(['build/glide-trim-tests', 'data/validation-material.airframe',
                    'results/glide-trim-summary.json'], cwd=ROOT, check=True)
    reference = json.loads((ROOT / 'data/spitfire-va-naca-reference.json').read_text())
    wing = reference['configuration']['reported_wing']
    source = (ROOT / 'data/validation-material.airframe').read_text()
    cells = [line.split() for line in source.splitlines() if line.startswith('cell ')]
    halfspan = max(abs(float(c[6])) + float(c[10])/2 for c in cells if c[-1] == 'wing')
    records, bases = [], []
    for line in source.splitlines():
        c = line.split()
        if c and c[0] == 'cell' and c[-1] == 'wing':
            fraction = abs(float(c[6])) / halfspan
            c[11] = format(wing['root_incidence_from_thrust_axis_deg'] * (1-fraction) +
                           wing['tip_incidence_from_thrust_axis_deg'] * fraction, '.17g')
            angle = math.radians(wing['leading_edge_dihedral_deg'])/2
            side = 1 if float(c[6]) > 0 else -1
            bases.append('cell_basis ' + c[1] + ' ' + ' '.join(format(v, '.17g') for v in
                         (math.cos(angle), -side*math.sin(angle), 0, 0)))
            line = ' '.join(c)
        records.append(line)
    variant = ROOT / 'build/glide-angle-variant.airframe'
    variant.write_text('\n'.join(records + bases) + '\n')
    grids = {}
    for name, path in [('base', ROOT/'data/validation-material.airframe'), ('angle', variant)]:
        target = OUT / f'glide-{name}-grid.csv'
        command = ['build/glide-probe', str(path), str(OUT/'glide-angle')]
        if name == 'base':
            command.append('grid-only')
        with target.open('w') as stream:
            subprocess.run(command, cwd=ROOT, stdout=stream, check=True)
        grids[name] = rows(target)
    with (OUT/'glide-sustained-grid.csv').open('w') as stream:
        subprocess.run(['build/glide-probe', str(variant), str(OUT/'glide-sustained'), 'sustained'],
                       cwd=ROOT, stdout=stream, check=True)
    traces, controls = [], []
    for study in ('angle', 'sustained'):
        for suffix, target in [('traces', traces), ('authority', controls)]:
            for row in rows(OUT/f'glide-{study}-{suffix}.csv'):
                row['study'] = study
                target.append(row)
    for row in traces + controls:
        assert all(math.isfinite(float(v)) for k, v in row.items() if k not in ('snapshot', 'study'))
    flights = {}
    for study, speeds, modes in [('angle', (70, 140), range(4)), ('sustained', (60, 140), (1, 3))]:
      for speed in speeds:
        for mode in modes:
            for hz in (120, 240):
                group = [r for r in traces if r['study'] == study and float(r['entry_tas']) == speed and
                         int(r['case']) == mode and int(r['hz']) == hz]
                assert group
                first = lambda field: next((float(r['time_s']) for r in group if float(r[field]) > 0), None)
                flights[f'{study}-{speed}-{mode}-{hz}'] = {
                    'duration_s': float(group[-1]['time_s']),
                    'first_root_stall_flag_s': first('root_stalled'),
                    'first_outer_stall_flag_s': first('outer_stalled'),
                    'peak_root_stalled_cells': max(int(r['root_stalled']) for r in group),
                    'peak_outer_stalled_cells': max(int(r['outer_stalled']) for r in group),
                    'first_edge_failure_s': first('failed_edges'), 'first_separation_s': first('debris'),
                    'peak_aero_body_z_g': max(float(r['aero_body_z_g']) for r in group),
                    'peak_front_strain': max(float(r['front_strain']) for r in group),
                    'max_structure_balance_error_Nm': max(float(r['balance_error_Nm']) for r in group),
                    'tas_change_mps': float(group[-1]['tas_mps']) - speed,
                    'path_change_rad': float(group[-1]['air_path_rad']) - float(group[0]['air_path_rad'])}
                sample = min(group, key=lambda r: abs(float(r['time_s'])-1.1))
                loads = [float(sample[k]) for k in ('front_bend_Nm', 'rear_bend_Nm', 'skin_bend_Nm')]
                flights[f'{study}-{speed}-{mode}-{hz}']['root_load_fractions_at_1p1s'] = [v/sum(loads) for v in loads]
    for hz in (120, 240):
        assert flights[f'sustained-60-3-{hz}']['first_edge_failure_s'] is None
        assert flights[f'sustained-140-1-{hz}']['first_edge_failure_s'] is None
        damaged = flights[f'sustained-140-3-{hz}']
        assert 1 < damaged['first_edge_failure_s'] <= damaged['first_separation_s']
        assert damaged['peak_front_strain'] > 0
    assert all(float(r['disconnected_moment_difference_Nm']) == 0 for r in controls)
    result = {
        'meaning': 'Synthetic capability experiments; zero historical performance matches run.',
        'reference': 'data/spitfire-va-naca-reference.json; L353 printed pp.4-7 / PDF6-9',
        'source_condition': 'Gliding: throttle closed, flaps/gear up, hood/radiator closed, covered gun ports. Gliding propeller RPM/pitch unknown; cruising 2650 engine RPM is not a glide input.',
        'model_condition': '1000 m model altitude; inputs are TAS, not source indicated stall speeds. Zero throttle uses existing friction/windmilling shaft model, not calibrated Merlin idle.',
        'angle_variant': 'Synthetic fixture with only wing incidence/basis changed. Linear root 2.1 to tip 0 deg interpolation is an assumption, not reported washout distribution. Bases use +/-6 deg body-X rotation; positions, span/area, masses, polars, material and propeller remain synthetic.',
        'cases': {'0': 'unforced coast', '1': 'intact pull/recovery',
                  '2': 'same pull; at 1 s left chain range .6, hinge .7, lag .15, friction 4; eight left outer cells lost',
                  '3': 'case 2 plus front-root remaining section .3 at 1 s'},
        'studies': {'angle': '6 s; pitch ramp 2-3 s, hold to 4 s, recover to 5.5 s',
                    'sustained': '16 s or first separation; pitch ramp 2-3 s, hold to 12 s, recover to 15.5 s; 60/140 m/s, intact and combined damage'},
        'timing': '120 Hz authoritative experiment and 240 Hz sensitivity comparison. Cell/force diagnostics freshly evaluated on endpoint copies; structure loads/strain and aero-body-Z load factor are actual last-step samples. Stop at first separation.',
        'stall_definition': 'Existing absolute effective cell alpha >15 deg telemetry flag; no dynamic stall, buffet or inter-cell wake claim.',
        'authority_definition': '2 s real control-chain settling on held-body/rotor/structure copies; +/-0.2 and +/-1 command secant total roll moments. Not a free-flight roll rate or pilot-force match. Disconnected roll commands must produce identical loads.',
        'static_flow_sweep': rows(OUT/'glide-angle-static.csv'),
        'static_flow_meaning': 'Prescribed body-relative flow on isolated copies, 0..24 degrees in 0.5 degree steps. Entry rotor/elastic state held, real control chains evaluated. Not a maneuver or coupled equilibrium, and not historical stall validation.',
        'trim_grids': grids, 'flights': flights, 'authority_samples': controls,
        'limits': 'Failed bounded Newton solves do not prove stall speed or nonexistence of equilibrium. Short coasts do not establish global stability. Static synthetic polars/material law and absent calibrated downwash, idle map and fracture data preclude historical validation.'}
    (OUT/'glide-flight-summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'flights': flights, 'summary': 'results/glide-flight-summary.json'}, indent=2))


if __name__ == '__main__':
    main()
