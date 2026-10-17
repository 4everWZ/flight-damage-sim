#!/usr/bin/env python3
"""Exercise moving-heading feedforward in the authoritative C++ flight core."""
import json
import math
import subprocess
from experiments import ROOT, RESULTS, run


def error(row):
    delta = row['air_heading'] - .08 * row['time']
    return math.atan2(math.sin(delta), math.cos(delta))


def measure(rows):
    tail = [r for r in rows if r['time'] >= 20]
    assert all(math.isfinite(v) for r in rows for v in r.values())
    return {
        'tail_heading_rms_deg': math.degrees(math.sqrt(sum(error(r)**2 for r in tail) / len(tail))),
        'final_heading_error_deg': math.degrees(error(rows[-1])),
        'tail_path_rms_deg': math.degrees(math.sqrt(sum(r['path_angle']**2 for r in tail) / len(tail))),
        'minimum_altitude_change_m': min(r['altitude'] for r in rows) - rows[0]['altitude'],
        'final_altitude_change_m': rows[-1]['altitude'] - rows[0]['altitude'],
        'maximum_normal_g': max(r['g'] for r in rows),
        'max_stalled_cells': max(r['stalled'] for r in rows),
        'max_failed_edges': max(r['failed'] for r in rows),
        'final_speed_mps': rows[-1]['speed'],
        'final_aim_rate_rad_s': rows[-1]['aim_heading_rate'],
    }


def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make', 'build/flight-run'], cwd=ROOT, check=True)
    measurements, checks, convergence = {}, {}, {}
    for frame in ('validation.airframe', 'validation-material.airframe'):
        label = frame.removesuffix('.airframe')
        cases, traces = {}, {}
        for name, hz, cadence, enabled, damage in (
                ('feedback', 120, 0, False, False),
                ('continuous', 120, 0, True, False),
                ('sample30', 120, 30, True, False),
                ('sample60', 120, 60, True, False),
                ('fine30', 240, 30, True, False),
                ('damaged30', 120, 30, True, True)):
            scenario = 'mouse_circle_damaged' if damage else 'mouse_circle'
            rows, _ = run(scenario, hz=hz, seconds=60,
                          suffix=f'_aim_{label}_{name}', airframe=frame,
                          aim_hz=cadence, aim_rate_ff=enabled)
            cases[name], traces[name] = measure(rows), rows
        coarse, fine = traces['sample30'][-1], traces['fine30'][-1]
        convergence[label] = {k: abs(coarse[k] - fine[k])
                              for k in ('altitude', 'speed', 'air_heading')}
        checks[label] = {
            'feedback_only_exhibits_predicted_gap': 12 < cases['feedback']['tail_heading_rms_deg'] < 15,
            'feedforward_removes_systematic_gap': cases['continuous']['tail_heading_rms_deg'] < 1
                and cases['continuous']['tail_heading_rms_deg'] < .1 * cases['feedback']['tail_heading_rms_deg'],
            'sampled_input_tracks': all(cases[k]['tail_heading_rms_deg'] < 1 for k in ('sample30', 'sample60', 'fine30')),
            'level_path_remains_tracked': all(cases[k]['tail_path_rms_deg'] < .15 for k in ('continuous', 'sample30', 'sample60')),
            'damaged_wing_still_uses_available_controls': cases['damaged30']['tail_heading_rms_deg'] < 3,
            'no_stall_or_structure_failure_in_these_turns': all(c['max_stalled_cells'] == 0 and c['max_failed_edges'] == 0 for c in cases.values()),
            'sampled_120_240_convergence': convergence[label]['altitude'] < .5
                and convergence[label]['air_heading'] < .002,
        }
        # Same fixed direction/throttle/damage inputs; enabling the observer
        # must not change the existing stationary-aim flight response.
        stationary = [run('mouse', seconds=30, airframe=frame,
                          suffix=f'_aim_{label}_static_{int(enabled)}',
                          aim_rate_ff=enabled)[0] for enabled in (False, True)]
        checks[label]['stationary_aim_identical_flight'] = stationary[0] == stationary[1]
        measurements[label] = cases
    report = {
        'mechanism': 'Without feedforward a constant target heading rate requires e=rate/heading_gain; 0.08/0.35 predicts 13.096 degrees. Filtered world-heading rate adds that missing requested rate through the existing bank/body-rate/actuator path.',
        'predicted_feedback_lag_deg': math.degrees(.08/.35),
        'measurements': measurements, 'sample30_120_240_difference': convergence,
        'checks': checks,
        'scope': 'Two synthetic propeller fixtures, fixed 120Hz flight with a 240Hz comparison; continuous or 30/60Hz sample-and-hold input from the same 0.08rad/s horizontal path. Local aero loss removes eight left outer-wing cells at 1s using the existing damage API. RMS uses the common continuous requested path after 20s, including the sampling delay. Constant aim, ownership/handoff, wrap/reset and all-links-cut invariants are checked separately. This is no claim of universal altitude improvement, historical handling, vertical-aim-rate tracking, or repair of finite-span coupling/the original 10% failure.',
    }
    (RESULTS/'aim-tracking-summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    if not all(v for group in checks.values() for v in group.values()):
        raise SystemExit('Moving aim check failed; retain the measurements and inspect the mechanism')


if __name__ == '__main__':
    main()
