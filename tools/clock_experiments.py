#!/usr/bin/env python3
"""Discriminate controller, sensor and output clocks on the same 960 Hz plant."""
import csv
import json
import math
import subprocess
from experiments import ROOT, RESULTS

AXES = ['roll', 'pitch', 'yaw']
PAIRS = [('A','B','internal update'), ('B','C','sensor read'),
         ('C','D','request output'), ('D','E','force age')]


def vector(row, group):
    return tuple(row[f'{group}_{a}'] for a in AXES)


def effect(cases):
    v = [cases['1'][a]-cases['0'][a] for a in ['x','y','z']]
    return {'effect_vector_m':v, 'effect_norm_m':math.sqrt(sum(x*x for x in v))}


def analyze(raw, trace_path, reference_path):
    cases, native = {}, {}
    for line in raw.splitlines():
        f = line.split()
        if f[0]=='native':
            native.setdefault(f[2], {})[f[1]] = {f[i]:float(f[i+1]) for i in range(3,len(f),2)}
        else:
            assert f[0]=='clock'
            cases.setdefault(f[1], {})[f[2]] = {f[i]:float(f[i+1]) for i in range(3,len(f),2)}
    effects = {a:effect(c) for a,c in cases.items()}
    native_effects = {h:effect(c) for h,c in native.items()}
    rows = []
    for row in csv.DictReader(trace_path.open()):
        rows.append({k:v if k=='arm' else float(v) for k,v in row.items()})
    indexed = {(r['arm'],int(r['case']),int(r['tick'])):r for r in rows}
    profiles = {}
    for arm, c in cases.items():
        for case, meta in c.items():
            cr = [r for r in rows if r['arm']==arm and int(r['case'])==int(case)]
            events = {int(r['tick']):vector(r,'applied') for r in cr if r['output_event']}
            assert sorted(events)==list(range(0,4800,960//int(meta['output_hz'])))
            current = None
            wave = []
            for tick in range(4800):
                current = events.get(tick,current)
                assert current is not None
                wave.append(current)
            profiles[arm,case] = wave
            assert meta['reads']==5*meta['sensor_hz']
            assert meta['updates']==5*meta['internal_hz']
            assert meta['outputs']==5*meta['output_hz']
            age_ticks = round(meta['force_age_s']*1920)
            for r in cr:
                tick=int(r['tick'])
                for field,rate in [('read_event','sensor_hz'),('update_event','internal_hz'),('output_event','output_hz')]:
                    assert bool(r[field])==(tick%(960//int(meta[rate]))==0)
                assert r['computed_tick']<=tick and r['sensor_tick']<=r['computed_tick']
                assert r['force_half_tick']==2*r['sensor_tick']-age_ticks
                assert r['left_half_tick']<=r['force_half_tick']<=r['right_half_tick']<2*r['sensor_tick']
                assert abs(r['normal_g']+r['raw_fz']/(r['mass']*9.80665))<1e-12
                assert vector(r,'applied')==wave[tick]
                if r['output_event']:
                    assert vector(r,'applied')==vector(r,'computed')
            before=next(r['mass'] for r in cr if r['tick']==0)
            after=next(r['mass'] for r in cr if r['tick']==960)
            assert after<before if case=='1' else after==before
            assert all(r['mass']==(before if r['tick']<960 else after) for r in cr)
    ref = {(int(r['case']),round(float(r['control_time'])*960)):r
           for r in csv.DictReader(reference_path.open())}
    anchor_differences = []
    for r in rows:
        if r['arm']=='A' and r['output_event']:
            old=ref[int(r['case']),int(r['tick'])]
            for a,b in [('raw_fz','raw_fz'),('normal_g','normal_g'),('mass','mass')]:
                anchor_differences.append(abs(r[a]-float(old[b])))
            assert abs(r['force_half_tick']/1920-float(old['force_time']))<1e-14
    assert max(anchor_differences)<1e-9
    path=[]
    for a,b,factor in PAIRS:
        delta=[y-x for x,y in zip(effects[a]['effect_vector_m'],effects[b]['effect_vector_m'])]
        signals={}
        for case in ['0','1']:
            differences=[max(abs(x-y) for x,y in zip(u,v)) for u,v in zip(profiles[a,case],profiles[b,case])]
            first=next((i/960 for i,d in enumerate(differences) if d>1e-12),None)
            signals[case]={'l1_max_axis_normalized_s':sum(differences)/960,
                           'max_axis_normalized':max(differences),'first_difference_s':first}
        path.append({'from':a,'to':b,'changed_factor':factor,'effect_vector_delta_m':delta,
                     'effect_vector_shift_m':math.sqrt(sum(x*x for x in delta)),
                     'applied_request_differences':signals})
    # Same input/state prefix: these observations locate each mechanism before
    # changed physical requests can feed back into the compared trajectories.
    early={}
    a,b=indexed['A',0,0],indexed['B',0,0]
    integral=[]
    for axis in AXES:
        if not a[f'paused_{axis}'] and not b[f'paused_{axis}'] and abs(a[f'computed_{axis}'])<.9 and abs(b[f'computed_{axis}'])<.9:
            ia=a[f'bias_after_{axis}']-a[f'bias_before_{axis}']
            ib=b[f'bias_after_{axis}']-b[f'bias_before_{axis}']
            if abs(ib)>1e-12:
                assert a[f'error_{axis}']==b[f'error_{axis}']
                assert abs(ia-8*ib)<1e-14
                du=a[f'computed_{axis}']-b[f'computed_{axis}']
                assert abs(du-(a[f'bias_after_{axis}']-b[f'bias_after_{axis}']))<1e-14
                integral.append({'axis':axis,'same_error':a[f'error_{axis}'],
                                 'increments':[ia,ib],'increment_ratio':ia/ib,'request_difference':du})
    early['internal_update_at_t0']=integral
    b,c=indexed['B',0,8],indexed['C',0,8]
    assert profiles['B','0'][:8]==profiles['C','0'][:8]
    assert vector(b,'error')==vector(c,'error')
    early['sensor_read_at_first_common_output']={
        'time_s':8/960,'same_prior_applied_requests':True,'same_current_error':True,
        'bias_before_difference':[c[f'bias_before_{a}']-b[f'bias_before_{a}'] for a in AXES],
        'computed_request_difference':[c[f'computed_{a}']-b[f'computed_{a}'] for a in AXES]}
    c,d=indexed['C',0,1],indexed['D',0,1]
    assert vector(c,'computed')==vector(d,'computed')
    assert vector(c,'applied')==vector(indexed['C',0,0],'applied')
    assert vector(d,'applied')==vector(d,'computed')
    early['output_hold_at_first_fine_update']={
        'time_s':1/960,'same_computed_request':True,'output_events':[c['output_event'],d['output_event']],
        'applied_request_difference':[d[f'applied_{a}']-c[f'applied_{a}'] for a in AXES]}
    d,e=indexed['D',0,1],indexed['E',0,1]
    assert profiles['D','0'][:1]==profiles['E','0'][:1]
    early['force_age_before_physical_feedback_diverges']={
        'time_s':1/960,'force_sample_times_s':[d['force_half_tick']/1920,e['force_half_tick']/1920],
        'normal_g':[d['normal_g'],e['normal_g']]}
    original=math.dist(native_effects['120']['effect_vector_m'],native_effects['240']['effect_vector_m'])
    fixed=math.dist(effects['A']['effect_vector_m'],effects['F']['effect_vector_m'])
    return {'clock_cases':cases,'matched_effects':effects,'native_effects':native_effects,
            'conditional_clock_path':path,'early_mechanism_evidence':early,
            'protocol_checks':{'timestamp_and_hold_checks':True,'current_mass_normalization':True,
                               'known_t0_force_anchor':True,
                               'prior_fixed_age_force_trace_max_difference':max(anchor_differences),
                               'native_960_identity_max_error':max(v for case in cases['E'].values() for k,v in case.items() if k.endswith('_error')),
                               'prior_fixed_age_final_identity_max_error':max(v for case in cases['A'].values() for k,v in case.items() if k.startswith('anchor_'))},
            'original_pair_comparison':{'native_120_240_vector_difference_m':original,
                                        'native_10pct_hypothesis':original<.1*native_effects['120']['effect_norm_m'],
                                        'same_960hz_physics_120_vs_240_system_difference_m':fixed,
                                        'per_system_physical_refinement_difference_m':{
                                            '120':math.dist(effects['A']['effect_vector_m'],native_effects['120']['effect_vector_m']),
                                            '240':math.dist(effects['F']['effect_vector_m'],native_effects['240']['effect_vector_m'])}},
            'time_contract':'Physical kernel, real actuators, rotor and structure always 960 Hz. At each sensor read, capture body/density and raw force at read_time minus the declared age. Hold that packet; each internal update normalizes its raw force with current mass. Sensor read precedes controller update, then request output, then the physical step. Inputs and the mass event share aligned epochs. Known initial force is anchored at t=0 with constant trim prehistory.',
            'scope':'Conditional one-factor clock interventions and two endpoint identity checks, not additive attribution percentages. F is a same-physics 240 Hz sampled-system cross-check. The production kernel, gains and original acceptance test remain unchanged. No universal fidelity or GUI claim; user review is required before changing the intended sampled/continuous target or interpreting a different convergence criterion.'}


def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make','build/sensor-history-probe'],cwd=ROOT,check=True)
    raw=subprocess.run([str(ROOT/'build/sensor-history-probe'),str(ROOT/'data/validation-material.airframe'),
                        str(RESULTS/'clock'),'clock'],cwd=ROOT,capture_output=True,text=True,check=True).stdout
    (RESULTS/'clock-cases.txt').write_text(raw)
    report=analyze(raw,RESULTS/'clock-trace.csv',RESULTS/'clock-age-reference.csv')
    (RESULTS/'clock-summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
