#!/usr/bin/env python3
"""PI headroom invariants and matched material perturbations in the actual C++ core."""
import csv
import io
import json
import math
import subprocess
from experiments import ROOT, RESULTS


def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make', '-j2', 'build/control-tests', 'build/control-probe'], cwd=ROOT, check=True)
    def core(name):
        return subprocess.run([str(ROOT/'build'/name), str(ROOT/'data/validation-material.airframe')],
                              cwd=ROOT, capture_output=True, text=True, check=True).stdout
    boundary=json.loads(core('control-tests'))
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
    report={'boundary':boundary,'matched_flights':cases,
            'hypotheses':{'default_120_240_material_effect_agrees_10pct':
                cases['0:240']['adjacent_effect_difference_m']<.1*cases['0:120']['material_effect_m'],
                'refresh_120_240_material_effect_agrees_10pct':
                cases['1:240']['adjacent_effect_difference_m']<.1*cases['1:120']['material_effect_m']},
            'scope':'Normalized-command PI headroom, unchanged gains/actuators/physics. Fresh-load evaluation is an experiment only. This does not establish general closed-loop material sensitivity or historical fidelity.'}
    (RESULTS/'control-summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
