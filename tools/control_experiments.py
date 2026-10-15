#!/usr/bin/env python3
"""PI headroom invariants and matched material perturbations in the actual C++ core."""
import csv
import io
import json
import math
import subprocess
from experiments import ROOT, RESULTS


def isolation(raw):
    """Fixed normalized requests separate control sampling from plant stepping.

    Replays use real PilotIntent/actuators, not direct surface or pose writes.
    Same-case replay keeps the damage compensation recorded at 120 Hz; common
    replay deliberately removes that compensation to expose plant response.
    """
    modes=['closed_loop', 'same_case_120hz_requests', 'common_clean_120hz_requests']
    flights={name:{} for name in modes}
    events={}
    lift={}
    norm=lambda a: math.sqrt(sum(x*x for x in a))
    for line in raw.splitlines():
        f=line.split()
        if f[0]=='effect':
            mode,hz=int(f[1]),int(f[2]); effect=list(map(float,f[3:6]))
            flights[modes[mode]][str(hz)]={
                'effect_vector_m':effect, 'material_effect_m':float(f[7])}
            if mode==1 and hz==120:
                identity=float(f[9])
        elif f[0]=='event':
            events[f[1]]=list(map(float,f[2:5]))
        elif f[0]=='purelift':
            lift[f[1]]={'initial_aero_power_W':float(f[3]), 'energy_error_J':float(f[5])}
    for runs in flights.values():
        last=None
        for hz in [120,240,480]:
            r=runs[str(hz)]; e=r['effect_vector_m']
            r['adjacent_effect_difference_m']=None if last is None else norm([a-b for a,b in zip(e,last)])
            last=e
    center=events['1']
    changes={t:norm([a-b for a,b in zip(e,center)]) for t,e in events.items()}
    assert identity<1e-9, '120 Hz requests must reproduce the native trajectories'
    assert all(abs(v['energy_error_J'])<.001 for v in lift.values()), 'pure lift must not add discrete energy'
    return {'modes':flights, 'replay_120hz_position_identity_error_m':identity,
            'event_shift_480hz_effect_vectors_m':events,
            'event_shift_effect_change_m':changes, 'pure_lift_half_second':lift,
            'scope':'Trace replay is an offline causal ablation, not a production controller or general 120 Hz accuracy bound. All physical chains remain active. Event offsets are +/- 1/120 and +/- 1/240 s.'}


def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make', '-j2', 'build/control-tests', 'build/control-probe', 'build/sensitivity-probe'], cwd=ROOT, check=True)
    def core(name):
        return subprocess.run([str(ROOT/'build'/name), str(ROOT/'data/validation-material.airframe')],
                              cwd=ROOT, capture_output=True, text=True, check=True).stdout
    boundary=json.loads(core('control-tests'))
    isolated=core('sensitivity-probe')
    (RESULTS/'sensitivity-isolation.txt').write_text(isolated)
    causal=isolation(isolated)
    raw=core('control-probe')
    (RESULTS/'control-sensitivity.csv').write_text(raw)
    rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(io.StringIO(raw))]
    cases={}
    for refresh in [0,1]:
        last=None
        for hz in [120,240,480,960]:
            clean,damaged=[r for r in rows if r['refresh']==refresh and r['hz']==hz]
            effect=[damaged[k]-clean[k] for k in ['x','y','z']]
            norm=lambda a:math.sqrt(sum(x*x for x in a))
            cases[f'{refresh}:{hz}']={'material_effect_m':norm(effect),
                'effect_vector_m':effect,'adjacent_effect_difference_m':None if last is None else norm([a-b for a,b in zip(effect,last)]),
                'pitch_saturation_s':[clean['pitch_saturated_s'],damaged['pitch_saturated_s']],
                'pitch_integral_pause_s':[clean['pitch_integral_paused_s'],damaged['pitch_integral_paused_s']]}
            last=effect
    report={'boundary':boundary,'isolation':causal,'matched_flights':cases,
            'hypotheses':{'default_120_240_material_effect_agrees_10pct':
                cases['0:240']['adjacent_effect_difference_m']<.1*cases['0:120']['material_effect_m'],
                'refresh_120_240_material_effect_agrees_10pct':
                cases['1:240']['adjacent_effect_difference_m']<.1*cases['1:120']['material_effect_m']},
            'scope':'Normalized-command PI headroom plus causal trace/event ablations and midpoint aerodynamic work. Gains and actual actuator semantics are unchanged. Fresh-load evaluation is an experiment only. General closed-loop sensitivity and historical fidelity remain separate hypotheses.'}
    (RESULTS/'control-summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
