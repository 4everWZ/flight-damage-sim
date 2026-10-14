#!/usr/bin/env python3
"""Build and measure the one authoritative C++ simulation; standard library only."""
import csv
import json
import math
import subprocess
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT/'results'
SCENARIOS = ['clean','roll','pull','mouse','mixed','aero_damage','symmetric_damage','control_damage','compensate','structure','weakened_pull','highspeed_pull','highspeed_weakened','stall']

def run(scenario, hz=120, seconds=10, suffix=''):
    prefix = RESULTS/(scenario+suffix)
    completed = subprocess.run([str(ROOT/'build'/'flight-run'), str(ROOT/'data'/'validation.airframe'), scenario, str(prefix), str(hz), str(seconds)], check=True, capture_output=True, text=True)
    records = [{k:float(v) for k,v in row.items()} for row in csv.DictReader((prefix.with_suffix('.csv')).open())]
    return records, completed.stdout.strip()

def at(rows, t):
    return min(rows,key=lambda r:abs(r['time']-t))

def measure(rows):
    if 'load_g' in rows[0]:
        failed = next((r['time'] for r in rows if r['failed']>0),None)
        detached = next((r['time'] for r in rows if r['debris']>0),None)
        return {'first_failure_s':failed, 'detachment_s':detached, 'low_g_failed':at(rows,2.9)['failed'], 'initial_front_Nm':rows[0]['front'], 'initial_rear_Nm':rows[0]['rear'], 'initial_skin_Nm':rows[0]['skin']}
    return {'altitude_change_m':rows[-1]['altitude']-rows[0]['altitude'], 'speed_change_mps':rows[-1]['speed']-rows[0]['speed'],
            'max_abs_roll_rate_deg_s':max(abs(r['p']) for r in rows)*180/math.pi, 'max_g':max(r['g'] for r in rows),
            'final_heading_deg':rows[-1]['heading']*180/math.pi, 'max_stalled_cells':max(r['stalled'] for r in rows),
            'first_failure_s':next((r['time'] for r in rows if r['failed']>0),None),
            'detachment_s':next((r['time'] for r in rows if r['debris']>0),None)}

def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make','-j2'],cwd=ROOT,check=True)
    subprocess.run([str(ROOT/'build/core-tests'),str(ROOT/'data/validation.airframe')],check=True)
    measurements={}
    outputs={}
    for name in SCENARIOS:
        rows, info=run(name, seconds=30 if name=='mouse' else 10)
        measurements[name]=measure(rows);outputs[name]=rows
        if name=='clean':measurements['trim']=info
    repeat,_=run('mixed',suffix='_repeat')
    measurements['repeatability_same_binary']=repeat==outputs['mixed']
    fine,_=run('mixed',hz=240,suffix='_240hz')
    coarse=outputs['mixed'][-1];finer=fine[-1]
    measurements['120_240_final_difference']={k:abs(coarse[k]-finer[k]) for k in ['x','y','altitude','speed','p','q','r','heading']}
    baseline=at(outputs['clean'],1.1);damaged=at(outputs['aero_damage'],1.1)
    measurements['aero_damage_0_1s_response']={k:damaged[k]-baseline[k] for k in ['lift_z','roll_moment','p']}
    measurements['roll_at_2s_deg_s']={n:at(outputs[n],2)['p']*180/math.pi for n in ['roll','control_damage']}
    release=at(outputs['mixed'],2);after=at(outputs['mixed'],2.01);settled=at(outputs['mixed'],2.26)
    measurements['mixed_release']={ 'command_before':release['roll_cmd'], 'first_release_command':after['roll_cmd'], 'settled_direct_weight':settled['roll_owner'],
                                   'pitch_direct_weight':release['pitch_owner'], 'yaw_direct_weight':release['yaw_owner']}
    checks={
      'clean_10s_near_equilibrium':abs(measurements['clean']['altitude_change_m'])<2 and abs(measurements['clean']['speed_change_mps'])<.3,
      'full_authority_roll':all(r['roll_cmd']==-1 and r['roll_owner']==1 for r in outputs['mixed'] if 1.02<r['time']<1.99),
      'smooth_release':abs(after['roll_cmd']-release['roll_cmd'])<.02 and settled['roll_owner']==0,
      'local_loss_rolls_left':measurements['aero_damage_0_1s_response']['p']<-.01,
      'reduced_control_authority':abs(measurements['roll_at_2s_deg_s']['control_damage'])<abs(measurements['roll_at_2s_deg_s']['roll'])*.9,
      'low_load_survives_high_load_fails':measurements['structure']['low_g_failed']==0 and measurements['structure']['detachment_s'] is not None,
      'local_stall_entry_recovery':measurements['stall']['max_stalled_cells']>0 and outputs['stall'][-1]['stalled']==0,
      'finite_telemetry':all(math.isfinite(v) for rows in outputs.values() for row in rows for v in row.values()),
      'same_binary_replay':measurements['repeatability_same_binary'],
      'timestep_convergence':measurements['120_240_final_difference']['speed']<.1 and measurements['120_240_final_difference']['heading']<.02,
      'coupled_delayed_failure':measurements['highspeed_weakened']['first_failure_s'] is not None and measurements['highspeed_weakened']['first_failure_s']>3.1 and measurements['highspeed_weakened']['detachment_s']>measurements['highspeed_weakened']['first_failure_s'] and measurements['highspeed_pull']['detachment_s'] is None and at(outputs['highspeed_weakened'],2.9)['failed']==0,
      '90deg_mouse_tracking':abs(measurements['mouse']['final_heading_deg']-90)<10,
    }
    highfine,_=run('highspeed_weakened',hz=240,suffix='_240hz')
    fine_failure=next((r['time'] for r in highfine if r['failed']>0),None)
    coarse_failure=measurements['highspeed_weakened']['first_failure_s']
    measurements['120_240_failure_time_difference_s']=abs(fine_failure-coarse_failure) if fine_failure is not None and coarse_failure is not None else None
    checks['failure_timestep_convergence']=measurements['120_240_failure_time_difference_s'] is not None and measurements['120_240_failure_time_difference_s']<.1
    report={'measurements':measurements,'checks':checks}
    (RESULTS/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not all(checks.values()):raise SystemExit('Some acceptance experiments failed; see results/summary.json')
if __name__=='__main__':main()
