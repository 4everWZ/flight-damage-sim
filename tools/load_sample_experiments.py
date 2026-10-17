#!/usr/bin/env python3
"""Fix the original raw-force sample age while refining the physical plant."""
import csv
import json
import math
import subprocess
from experiments import ROOT, RESULTS


def analyze(raw, trace_path):
    cases = {}
    for line in raw.splitlines():
        f = line.split()
        assert f[0] == 'aged'
        cases[f'{f[1]}:{f[2]}'] = {f[i]: float(f[i+1]) for i in range(3,len(f),2)}
    effects = {}
    previous = None
    for hz in [120,240,480,960]:
        c,d = cases[f'0:{hz}'], cases[f'1:{hz}']
        v = [d[k]-c[k] for k in ['x','y','z']]
        effects[str(hz)] = {'effect_vector_m':v, 'effect_norm_m':math.sqrt(sum(a*a for a in v)),
                            'adjacent_effect_difference_m':None if previous is None else math.dist(v,previous)}
        previous = v
    rows = [{k:float(v) for k,v in r.items()} for r in csv.DictReader(trace_path.open())]
    max_age_error = max(abs((r['control_time']-r['force_time'])-1/240) for r in rows)
    assert max_age_error < 1e-14
    assert all(r['right_time'] < r['control_time'] for r in rows), 'sensor reads must be causal'
    assert all(r['left_time'] <= r['force_time'] <= r['right_time'] for r in rows)
    assert all(abs(r['normal_g'] + r['raw_fz']/(r['mass']*9.80665)) < 1e-12 for r in rows)
    event = [r for r in rows if r['case']==1 and r['control_time']==1]
    assert len(event)==4
    assert all(r['mass']==cases[f"1:{int(r['hz'])}"]['event_mass_after'] for r in event)
    assert all(cases[f'1:{h}']['event_mass_after'] < cases[f'1:{h}']['event_mass_before']
               and cases[f'1:{h}']['event_cached_force_change']==0
               and cases[f'1:{h}']['event_cached_g_error']==0 for h in [120,240,480,960])
    report = {'cases':cases, 'matched_effects':effects,
              'protocol_checks':{'exact_ramp_and_missing_bracket_checks':True,
                                 'default_120hz_per_step_identity':all(
                                     v==0 for case in ['0:120','1:120'] for k,v in cases[case].items()
                                     if k.endswith('_error') or k=='pause_mismatches'),
                                 'max_sample_age_error_s':max_age_error,
                                 'causal_force_brackets':True,'current_mass_normalization':True},
              'mass_event_samples':event,
              'diagnostic_observation':{'120_240_difference_m':effects['240']['adjacent_effect_difference_m'],
                                        'fraction_of_120hz_effect':effects['240']['adjacent_effect_difference_m']/effects['120']['effect_norm_m']},
              'contract':'Controller/request clock 120 Hz. At t, reconstruct raw aerodynamic and propeller body-force vectors at t-1/240 s from completed physical midpoint samples; normalize the aerodynamic body-Z load and projection contributions by CURRENT mass*gravity. Exact samples are preserved; finer grids linearly interpolate past brackets for every component. Initial prehistory extends the trim caches as constants. No off-tick force evaluation and no plant pose writes.',
              'limits':'This is an independent diagnostic. It does not modify production physics/control/sensor timing or the original native 10% acceptance hypothesis in control-summary.json. Linear force reconstruction and one synthetic material-removal maneuver do not establish universal closed-loop fidelity.'}
    return report


def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make', 'build/sensor-history-probe'], cwd=ROOT, check=True)
    prefix = RESULTS/'load-sample'
    raw = subprocess.run([str(ROOT/'build/sensor-history-probe'),
                          str(ROOT/'data/validation-material.airframe'), str(prefix), 'age'],
                         cwd=ROOT, capture_output=True, text=True, check=True).stdout
    (RESULTS/'load-sample-cases.txt').write_text(raw)
    report = analyze(raw, RESULTS/'load-sample-trace.csv')
    (RESULTS/'load-sample-summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
