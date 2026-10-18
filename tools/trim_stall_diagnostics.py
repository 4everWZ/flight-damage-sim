#!/usr/bin/env python3
"""Controlled lookup/roll/pitch diagnosis and dynamic simulated flights.

All aerodynamic interventions are isolated counterfactuals, not adopted data.
"""
import bisect
import csv
import json
import math
import subprocess
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results'


def read(suffix):
    with (OUT/f'trim-stall-{suffix}.csv').open() as stream:
        return list(csv.DictReader(stream))


def summarize():
    tables = defaultdict(list)
    for line in (ROOT/'data/validation-material.airframe').read_text().splitlines():
        fields = line.split()
        if fields and fields[0] == 'curve':
            tables[fields[1]].append([math.radians(float(fields[2])), *map(float, fields[3:])])

    def coefficients(name, alpha):
        table = tables[name]
        if alpha <= table[0][0]:
            return table[0][1:]
        if alpha >= table[-1][0]:
            return table[-1][1:]
        right = bisect.bisect_right([p[0] for p in table], alpha)
        a, b = table[right-1:right+1]
        weight = (alpha-a[0])/(b[0]-a[0])
        return [(1-weight)*u+weight*v for u, v in zip(a[1:], b[1:])]

    lookup = read('lookup')
    coefficient_errors = [0., 0., 0.]
    for row in lookup:
        expected = coefficients(row['table'], float(row['effective_alpha_rad']))
        for i, field in enumerate(('inferred_CL', 'inferred_CD', 'inferred_CM')):
            coefficient_errors[i] = max(coefficient_errors[i], abs(float(row[field])-expected[i]))
    assert max(coefficient_errors) < 1e-10, coefficient_errors
    rho = 1.225*math.exp(-1000/8500)
    pair = read('pair')
    pair_error = 0.
    for row in pair:
        speed, alpha = float(row['tas']), math.radians(float(row['alpha_deg']))
        q = rho*speed**2/2
        pg = 1/math.sqrt(1-speed**2/340**2)
        left = coefficients('wing', alpha+.8*float(row['left_plus_rad']))
        right = coefficients('wing', alpha-.8*float(row['right_plus_rad']))
        expected = 4*q*((left[0]-right[0])*pg*math.cos(alpha)+(left[1]-right[1])*math.sin(alpha))
        pair_error = max(pair_error, abs(float(row['plus_moment_Nm'])-expected),
                         abs(float(row['minus_moment_Nm'])+expected))
    assert pair_error < 1e-7, pair_error

    roll = read('roll')
    for speed in (60, 140):
        for mode in ('native', 'rigid_no_prop', 'no_wing_drag'):
            small = next(r for r in roll if float(r['tas']) == speed and r['mode'] == mode and
                         r['alpha_deg'] == '20' and float(r['amplitude']) == .001)
            assert float(small['gain_Nm']) < 0
        for row in roll:
            if row['mode'] == 'disconnect':
                assert float(row['gain_Nm']) == 0
            if row['mode'] == 'linear_wing_CL':
                assert float(row['gain_Nm']) > 0
            assert float(row['other_delta_Nm']) == 0
    assert next(float(r['gain_Nm']) for r in roll if r['tas'] == '60' and r['mode'] == 'native' and
                r['alpha_deg'] == '20' and r['amplitude'] == '1') > 0
    assert next(float(r['gain_Nm']) for r in roll if r['tas'] == '140' and r['mode'] == 'native' and
                r['alpha_deg'] == '20' and r['amplitude'] == '1') < 0

    trims, pitch = read('trim'), read('pitch')
    envelopes = {}
    for trim in trims:
        speed = trim['tas']
        group = [r for r in pitch if r['tas'] == speed and r['command'] == '1']
        crossings = [[float(a['alpha_deg']), float(b['alpha_deg'])] for a, b in zip(group, group[1:])
                     if float(a['moment_y_Nm']) >= 0 > float(b['moment_y_Nm'])]
        at12 = next(r for r in group if r['alpha_deg'] == '12')
        assert float(at12['moment_y_Nm']) < 0
        assert -15 < float(at12['min_elevator_alpha_deg']) < 15
        envelopes[speed] = {'trim': trim, 'full_command_pitch_zero_alpha_brackets_deg': crossings,
                            'held_flow_at_12_deg': at12}

    groups = defaultdict(list)
    for row in read('flights'):
        assert all(math.isfinite(float(v)) for k, v in row.items() if k != 'mode')
        groups[f"{row['tas']}-{row['mode']}-{row['hz']}"].append(row)
    flights = {}
    for name, group in groups.items():
        full = [r for r in group if float(r['time_s']) >= 3.1]
        assert all(float(r['command']) == 1 for r in full)
        if group[0]['mode'] == 'captured_jam':
            captured = next(r for r in group if float(r['time_s']) >= 1.01)['elevator_rad']
            assert all(r['elevator_rad'] == captured for r in group if float(r['time_s']) >= 1.01)
        if group[0]['mode'] == 'disconnect':
            assert all(float(r['elevator_rad']) == 0 for r in full)
        flights[name] = {'duration_s': float(group[-1]['time_s']),
                         'peak_wing_alpha_deg': max(float(r['max_wing_alpha_deg']) for r in group),
                         'peak_stalled_cells': max(int(r['stalled']) for r in group),
                         'peak_failed_edges': max(int(r['failed']) for r in group),
                         'final_elevator_rad': float(group[-1]['elevator_rad']),
                         'final_TAS_mps': float(group[-1]['actual_tas'])}
    result = {
        'meaning': 'Synthetic controlled diagnostics plus 16 dynamic simulated flights; no real-world flight tests or historical calibration.',
        'baseline': 'PR17 f25e9c75355bae4958b83e1bdbc17761a4d6a085; core/data force and control laws unchanged.',
        'lookup_checks': {'samples': len(lookup), 'max_CL_CD_CM_errors': coefficient_errors,
                          'coverage': 'Every table knot/midpoint with actual neutral/+/-full controls, including endpoint-clamped effective angles outside table range.'},
        'independent_pair': {'samples': len(pair), 'max_moment_error_Nm': pair_error,
                             'formula': 'Lx=4*q*[PG*(CL_left-CL_right)*cos(alpha)+(CD_left-CD_right)*sin(alpha)], 1 m^2 surfaces at y=+/-4 m. Independent scalar table interpolation; actual chain angles.'},
        'roll_causality': 'Negative small-command post-stall gain survives zero elasticity/prop removal and drag removal; continuing CL linearly removes it. Both 60 and 140 m/s show it. Full-command secants cross different table intervals because pressure-limited travel differs. This follows the adopted effective-alpha table law; it is not established physical/aeroelastic reversal.',
        'counterfactuals': 'rigid_no_prop zeroes held node rotations/removes prop; no_wing_drag zeroes wing CD; linear_wing_CL substitutes CL=5.2*alpha on diagnostic copies only. No production definitions or force laws are changed.',
        'roll_samples': roll, 'pitch_envelopes': envelopes, 'dynamic_simulated_flights': flights,
        'pitch_limits': 'Held-flow pitch zeros are conditional on entry elastic/rotor/other-control state. They are not a six-equation glide solution or proof of nonexistence. The 50..59 m/s bounded trim failures retain residuals. Elevator command headroom near 60 m/s is almost consumed by trim; fixed-tail lift opposes the controlled tail. No sampled 12-degree elevator cell is stalled.',
        'sources': [
            {'url': 'https://www.grc.nasa.gov/www/k-12/VirtualAero/BottleRocket/airplane/alr.html', 'use': 'Low-angle lift imbalance and torque direction; not post-stall calibration.'},
            {'url': 'https://www.nasa.gov/news-release/nasa-dryden-flight-research-center-news-room-news-releases-active-aeroelastic-wing-completes-first-phase-of-flight-tests/', 'use': 'AAW release 03-25 describes aeroelastic reversal through pressure-driven wing twist. Frozen/zero-elastic diagnostics here do not establish that mechanism.'},
            {'url': 'https://ntrs.nasa.gov/api/citations/19830002803/downloads/19830002803.pdf#page=10', 'use': 'NASA differential-aileron wind-tunnel report printed p.8/PDF10: apparent high-angle reversal attributed to asymmetric stall, warning against misidentifying it as true reversal. Different swept research model; no values imported.'}],
        'next_choice': 'No established sign/interpolation implementation bug was found. Quantitative separation/aileron behavior or larger pitch authority needs separately surfaced aerodynamic/control/geometry assumptions; do not hide negative gain, increase authority or rewrite post-stall data as a bug fix.'}
    (OUT/'trim-stall-summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'lookup_checks': result['lookup_checks'], 'independent_pair': result['independent_pair'],
                      'dynamic_simulated_flights': flights, 'summary': 'results/trim-stall-summary.json'}, indent=2))


def main():
    OUT.mkdir(exist_ok=True)
    subprocess.run(['make', '-j2', 'build/trim-stall-probe'], cwd=ROOT, check=True)
    subprocess.run(['build/trim-stall-probe', 'data/validation-material.airframe',
                    'results/trim-stall'], cwd=ROOT, check=True)
    summarize()


if __name__ == '__main__':
    main()
