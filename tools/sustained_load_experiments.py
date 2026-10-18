#!/usr/bin/env python3
"""Measure sustained control/structure coupling in the actual C++ simulation."""
import argparse
import csv
import itertools
import json
import math
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'results'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reuse', action='store_true', help='analyze existing raw measurements')
    args = parser.parse_args()
    RESULTS.mkdir(exist_ok=True)
    if not args.reuse:
        subprocess.run(['make', '-j2', 'build/sustained-load-probe'], cwd=ROOT, check=True)
        with (RESULTS/'sustained-load-summary.csv').open('w') as summary:
            subprocess.run([str(ROOT/'build/sustained-load-probe'),
                            str(ROOT/'data/validation-material.airframe'),
                            str(RESULTS/'sustained-load-traces.csv')],
                           cwd=ROOT, stdout=summary, check=True)
    with (RESULTS/'sustained-load-summary.csv').open() as source:
        summaries = list(csv.DictReader(source))
    with (RESULTS/'sustained-load-traces.csv').open() as source:
        traces = list(csv.DictReader(source))

    def key(row):
        return (int(row['entry_speed']), row['maneuver'], int(row['weakened']), int(row['hz']))

    expected = set(itertools.product((70, 140), ('mouse', 'pull'), (0, 1), (120, 240)))
    assert len(summaries) == 16 and {key(r) for r in summaries} == expected
    grouped = {k: [] for k in expected}
    for row in traces:
        grouped[key(row)].append(row)
    cases = []
    events = []
    for raw in summaries:
        k = key(raw)
        case = {name: value if name in ('maneuver', 'first_failed_edge') else float(value)
                for name, value in raw.items()}
        for name in ('first_failure_s', 'separation_s'):
            if case[name] < 0:
                case[name] = None
        for name, value in case.items():
            if isinstance(value, float) and not math.isfinite(value):
                case[name] = None  # Unobserved/censored sample, never a synthetic zero.
        case['entry_converged'] = bool(case['entry_converged'])
        case['weakened'] = bool(case['weakened'])
        rows = grouped[k]
        assert len(rows) == round(case['end_s'] * k[3])
        assert abs(float(rows[-1]['time_end_s']) - case['end_s']) < 1e-12
        assert case['balance_error_Nm'] < 1e-6
        assert case['constitutive_error_Nm'] < 1e-6
        assert case['plastic_error'] < 1e-12 and case['release_jump'] < 1e-13
        # Failed edges retain their last diagnostic loads in the core. Rebuild
        # the BEFORE-step active mask, so stale values are not called live loads.
        failed = set()
        for row in rows:
            names = row['new_failures'].split(';') if row['new_failures'] else []
            if names:
                events.append({'case': list(k), 'time_end_s': float(row['time_end_s']),
                               'aero_body_z_g': float(row['aero_body_z_g']),
                               'new_failures': names,
                               'step_bending_Nm': {
                                   part: 0.0 if 'left_root_'+part in failed else float(row[part+'_bend_Nm'])
                                   for part in ('front', 'rear', 'skin')},
                               'debris': int(row['debris'])})
            failed.update(names)
        case['peak_stalled_cells_in_2_to_6s'] = max(
            (int(r['stalled_cells']) for r in rows if 2 < float(r['time_end_s']) <= 6), default=0)
        cases.append(case)

    # Future pull input must not affect the shared flight/damage/handoff history.
    for speed, weak, hz in itertools.product((70, 140), (0, 1), (120, 240)):
        a, b = [[{n: v for n, v in r.items() if n != 'maneuver'} for r in grouped[(speed, mode, weak, hz)]
                 if float(r['time_end_s']) <= 2] for mode in ('mouse', 'pull')]
        assert a == b, 'maneuver arms must have identical pre-pull physical histories'

    def selected(speed, maneuver, weak):
        return [c for c in cases if c['entry_speed'] == speed and c['maneuver'] == maneuver and c['weakened'] == weak]

    low = selected(70, 'pull', True)
    high = selected(140, 'pull', True)
    intact = selected(140, 'pull', False)
    # Parallel paths share one rotation difference, so their elastic moment
    # fractions are k_i/sum(k), independently of the applied aero/inertial load.
    stiffness = {}
    for line in (ROOT/'data/validation-material.airframe').read_text().splitlines():
        fields = line.split()
        if fields and fields[0] == 'edge' and fields[1] in (
                'left_root_front', 'left_root_rear', 'left_root_skin'):
            stiffness[fields[1]] = float(fields[4])
    front, rear, skin = [stiffness['left_root_'+p] for p in ('front', 'rear', 'skin')]
    fractions = {'intact_front': front/(front+rear+skin),
                 'weakened_front': .3**2*front/(.3**2*front+rear+skin)}
    for c in cases:
        expected_share = fractions['weakened_front' if c['weakened'] else 'intact_front']
        assert abs(c['root_front_share_2s']-expected_share) < 1e-10
    report = {
        'checks_passed': True,
        'scope': 'Current synthetic production model, not real-aircraft validation. All arms use unchanged polars, actuator/structure parameters and failure thresholds. The high-energy 140 m/s entry is NOT equilibrium.',
        'sampling': 'Forces/cell loads are interval-midpoint samples labeled by time_end_s; graph updates/events occur at the interval end. Events have tick resolution; traces stop at first separation or 8 s. G is aerodynamic body-Z/weight used at force evaluation (before any end-of-tick split), not requested G or total flight-path-normal load.',
        'case_count': len(cases), 'trace_rows': len(traces),
        'common_pre_pull_histories_identical': True,
        'parallel_path_elastic_reference': fractions,
        'cases': cases, 'failure_events': events,
        'observed_hypotheses': {
            'low_energy_weak_pull_survives_observed_window': all(c['first_failure_s'] is None and c['end_s'] == 8 for c in low),
            'full_pitch_guarantees_5g_at_70mps': all(c['peak_g_2_to_6s'] >= 5 for c in low),
            'high_energy_weak_pull_fails_after_load_rise': all(c['first_failure_s'] is not None and c['first_failure_s'] > 2 and c['peak_g_2_to_6s'] > c['peak_g_before_2s'] for c in high),
            'high_energy_intact_pull_survives_observed_window': all(c['first_failure_s'] is None and c['end_s'] == 8 for c in intact),
        },
        'maximum_equilibrium_residual_Nm': max(c['balance_error_Nm'] for c in cases),
        'maximum_plastic_increment_residual': max(c['plastic_error'] for c in cases),
        'limits': 'Equation closure validates this reduced generalized-moment graph only, not local beam/FEM stress, material calibration, aeroelastic energy, finite-span/stall fidelity, a universal convergence order or human handling. Hypothesis flags are observations; CI does not require the weak aircraft to break.'
    }
    (RESULTS/'sustained-load-summary.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('cases', 'failure_events')}, indent=2))


if __name__ == '__main__':
    main()
