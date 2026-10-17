#!/usr/bin/env python3
"""Control stage ablations at aligned physical times in the actual C++ core."""
import argparse
import json
import math
import subprocess
from pathlib import Path
from experiments import ROOT, RESULTS


def parse(raw):
    cases = {str(c): {'identity': {}, 'internal': {}, 'sensor_hold': {},
                     'actuator': {}, 'request_hold': {}} for c in [0, 1]}
    plant = {}
    locked = {}
    for line in raw.splitlines():
        f = line.split()
        if f[0] == 'identity':
            cases[f[1]]['identity'] = {'request_error': float(f[2]),
                                      'endpoint_surface_error_rad': float(f[3])}
        elif f[0] in ['plant', 'locked']:
            target = plant if f[0] == 'plant' else locked
            target[f[1]] = {'effect_vector_m': list(map(float, f[2:5])),
                           'effect_norm_m': float(f[6]),
                           'native_120hz_identity_error_m': float(f[8])}
        else:
            mode, case, update_hz = f[:3]
            metrics = {f[i]: float(f[i+1]) for i in range(3, len(f), 2)}
            if mode.startswith('sensor_'):
                group, rate = 'sensor_hold', mode.split('_')[-1]
            elif mode.startswith('request_hold_'):
                group, rate = 'request_hold', mode.split('_')[-1]
            else:
                group, rate = mode, update_hz
            metrics['update_hz'] = int(update_hz)
            cases[case][group][rate] = metrics
    for runs in [plant, locked]:
        last = None
        for hz in ['120', '240', '480', '960']:
            v = runs[hz]['effect_vector_m']
            runs[hz]['adjacent_effect_difference_m'] = None if last is None else math.dist(v, last)
            last = v
    return cases, plant, locked


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--history-prefix', type=Path, default=RESULTS/'sensor-history')
    parser.add_argument('--reuse-history', action='store_true',
                        help='Keep previously recorded sensor/request/applied-surface histories fixed.')
    parser.add_argument('--output-prefix', type=Path, default=RESULTS/'sensor-history')
    parser.add_argument('--before-replay', type=Path, help='Earlier-core replay output from the same fixed histories.')
    args = parser.parse_args()
    args.history_prefix = args.history_prefix.resolve()
    args.output_prefix = args.output_prefix.resolve()
    args.history_prefix.parent.mkdir(parents=True, exist_ok=True)
    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['make', '-j2', 'build/surface-sampling-tests', 'build/sensor-history-probe'], cwd=ROOT, check=True)
    entry = [str(ROOT/'build/sensor-history-probe'), str(ROOT/'data/validation-material.airframe'),
             str(args.history_prefix)]
    boundary = json.loads(subprocess.run([str(ROOT/'build/surface-sampling-tests'),
                                         str(ROOT/'data/validation-material.airframe')],
                                        capture_output=True, text=True, check=True).stdout)
    if not args.reuse_history:
        record = subprocess.run(entry+['record'], cwd=ROOT, capture_output=True, text=True, check=True)
        Path(str(args.output_prefix)+'-record.txt').write_text(record.stdout)
    raw = subprocess.run(entry+['replay'], cwd=ROOT, capture_output=True, text=True, check=True).stdout
    Path(str(args.output_prefix)+'-replay.txt').write_text(raw)
    trace = Path(str(args.history_prefix)+'-release.csv')
    output_trace = Path(str(args.output_prefix)+'-release.csv')
    if trace != output_trace:
        output_trace.write_text(trace.read_text())
    cases, plant, locked = parse(raw)
    observations = {
        'held_sensor_PI_endpoints_agree_1e12': all(
            row['endpoint_bias_max'] < 1e-12 for c in cases.values() for row in c['internal'].values()),
        'held_target_rate_only_actuator_endpoints_agree_1e12': all(
            row['endpoint_surface_max'] < 1e-12 for c in cases.values() for row in c['actuator'].values()),
        'midpoint_local_area_error_below_endpoint_rule': all(
            row['local_area_error'] < row['endpoint_local_area_error']
            for c in cases.values() for row in c['actuator'].values()),
    }
    report = {'boundary_checks': boundary, 'histories_reused': args.reuse_history,
              'cases': cases, 'fixed_applied_surface_plant': plant,
              'locked_120hz_fresh_sensor_and_controller': locked,
              'observations': observations,
              'time_contract': {
                  'sensor': 'Complete controller_loads_v3 inputs: pre-step body, current mass, aerodynamic/propeller vector-force caches and aerodynamic body-Z load factor; zero-order held on exact integer ticks. Scalar-only old histories are rejected, not reconstructed.',
                  'controller': 'Requests are emitted at the left boundary and held over the output interval; stored PI biases are compared at common right boundaries.',
                  'actuator': 'End states compare at common 1/120 s right boundaries. Applied values are the current interval midpoint; the prior endpoint force rule is retained as a numerical ablation.',
                  'norms': 'L1 columns integrate the per-time maximum absolute channel error. Local area errors sum per-1/120 s maximum surface-position time-integral error against the exact held-target zero-lag rate ramp.',
              },
              'scope': 'Each intervention holds all other stage inputs fixed. Fresh sensor sampling, PI update, request hold and actuator updates are separate interventions. Prescribed applied-surface flight uses an offline mechanical-jam fixture to remove controller/actuator feedback; it validates neither a real jam maneuver nor available authority. The locked-loop intervention samples fresh endpoint aerodynamics at fixed 120 Hz and holds actual normalized requests through real actuators; its 120 Hz identity is against the explicitly refreshed native variant, not the default stale-load loop. Interventions interact and their errors are not additive. The original native full-roll 10% hypothesis remains in control-summary.json and is unchanged.'}
    if args.before_replay:
        before_cases, before_plant, before_locked = parse(args.before_replay.read_text())
        report['fixed_history_before_after'] = {
            'actuator_120_local_area_error_rad_s': {
                c: [before_cases[c]['actuator']['120']['local_area_error'],
                    cases[c]['actuator']['120']['local_area_error']] for c in cases},
            'prescribed_surface_120_240_effect_difference_m': [
                before_plant['240']['adjacent_effect_difference_m'],
                plant['240']['adjacent_effect_difference_m']],
            'fresh_locked_loop_120_240_effect_difference_m': [
                before_locked['240']['adjacent_effect_difference_m'],
                locked['240']['adjacent_effect_difference_m']],
            'note': 'Source sensors/requests are fixed for stage replays. Closed-loop physical feedback deliberately changes with the core; its fresh sensor/controller clock is fixed at 120 Hz in both versions.'}
    Path(str(args.output_prefix)+'-summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
