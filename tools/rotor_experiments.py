#!/usr/bin/env python3
"""Signed prop loads, finite pitch governor and free-flight rotor ledger, all from C++."""
import csv
import io
import json
import math
import subprocess
from experiments import ROOT,RESULTS,run,at


def parse(text):
    return [{k:v if k=='case' else float(v) for k,v in row.items()} for row in csv.DictReader(io.StringIO(text))]


def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make','-j2'],cwd=ROOT,check=True)
    test=subprocess.run([str(ROOT/'build/propeller-tests'),str(ROOT/'data/validation-bem.airframe')],capture_output=True,text=True,check=True)
    print(test.stdout.strip())
    probe=subprocess.run([str(ROOT/'build/governor-probe'),str(ROOT/'data/validation-bem.airframe')],capture_output=True,text=True,check=True)
    (RESULTS/'governor.csv').write_text(probe.stdout)
    rows=parse(probe.stdout);groups={(name,hz):[r for r in rows if r['case']==name and r['hz']==hz] for name in ['fixed','governed','windmill','jammed','limited'] for hz in [120,240]}
    measurements={}
    for name in ['fixed','governed','windmill','jammed','limited']:
        r=groups[name,120];last=r[-1]
        measurements[name]={'final_rpm':last['rpm'],'target_rpm':last['target'],'final_pitch_deg':math.degrees(last['pitch']),
          'rpm_before_speed_step':at(r,14.9)['rpm'],'rpm_after_speed_step':at(r,24.9)['rpm'],
          'final_error_rpm':last['rpm']-last['target'],'max_pitch_rate_deg_s':max(abs(b['pitch']-a['pitch'])/(b['time']-a['time'])*180/math.pi for a,b in zip(r,r[1:])),
          'final_saturated':bool(last['saturated']),'max_rpm':max(a['rpm'] for a in r)}
    measurements['governor_final_120_240_rpm_difference']=abs(groups['governed',120][-1]['rpm']-groups['governed',240][-1]['rpm'])
    flight={}
    for name in ['clean','mouse','mixed','aero_damage','highspeed_pull','highspeed_weakened']:
        r,info=run(name,seconds=30 if name=='mouse' else 10,suffix='_bem',airframe='validation-bem.airframe')
        # Separation is a mass partition boundary; this single-parent ledger excludes post-split samples.
        intact=[a for a in r if a['debris']==0]
        energy0=intact[0]['mechanical_energy'];engine=sum(a['engine_work'] for a in intact[1:]);external=sum(a['external_work'] for a in intact[1:]);delta=intact[-1]['mechanical_energy']-energy0
        residual=delta-engine-external
        h0=[intact[0][k] for k in ['Hx','Hy','Hz']];h1=[intact[-1][k] for k in ['Hx','Hy','Hz']]
        impulses=[sum(a[k] for a in intact[1:]) for k in ['angular_impulse_x','angular_impulse_y','angular_impulse_z']]
        herror=math.sqrt(sum((b-a-i)**2 for a,b,i in zip(h0,h1,impulses)))
        flight[name]={'altitude_change_m':r[-1]['altitude']-r[0]['altitude'],'min_altitude_change_m':min(a['altitude'] for a in r)-r[0]['altitude'],'final_velocity_heading_deg':r[-1]['air_heading']*180/math.pi,
          'energy_ledger_residual_J':residual,'energy_relative':abs(residual)/max(abs(engine)+abs(external),1),
          'angular_impulse_residual_Nms':herror,'max_governor_error_rpm':max(abs(a['rpm']-a['target_rpm']) for a in intact),
          'first_failure_s':next((a['time'] for a in r if a['failed']>0),None),'detachment_s':next((a['time'] for a in r if a['debris']>0),None),'initialization':info,
          'max_prop_tip_mach':max(a['prop_tip_mach'] for a in intact),
          'max_induction_residual_N':max(abs(a['induction_residual']) for a in intact),'induction_converged':all(a['induction_converged'] for a in intact)}
    fine,_=run('mixed',hz=240,seconds=10,suffix='_bem_240',airframe='validation-bem.airframe')
    coarse,_=run('mixed',seconds=10,suffix='_bem_repeat',airframe='validation-bem.airframe')
    measurements['free_flight_120_240_difference']={k:abs(fine[-1][k]-coarse[-1][k]) for k in ['altitude','speed','rpm','air_heading']}
    checks={
      'governor_tracks_with_pitch_actuation':abs(measurements['governed']['final_error_rpm'])<25 and abs(measurements['governed']['rpm_before_speed_step']-measurements['governed']['rpm_after_speed_step'])<80,
      'fixed_load_changes_with_speed':abs(measurements['fixed']['rpm_before_speed_step']-measurements['fixed']['rpm_after_speed_step'])>200,
      'wind_starts_and_turns':measurements['windmill']['max_rpm']>1000,
      'jammed_pitch_cannot_restore_target':abs(measurements['jammed']['final_error_rpm'])>100,
      'travel_limit_cannot_restore_target':abs(measurements['limited']['final_error_rpm'])>300 and measurements['limited']['final_saturated'],
      'finite_rate_limit':max(v['max_pitch_rate_deg_s'] for k,v in measurements.items() if isinstance(v,dict) and 'max_pitch_rate_deg_s' in v)<=10.001,
      'governor_timestep_convergence':measurements['governor_final_120_240_rpm_difference']<5,
      'free_flight_energy_ledger':all(a['energy_relative']<1e-7 for a in flight.values()),
      'free_flight_angular_impulse':all(a['angular_impulse_residual_Nms']<1e-3 for a in flight.values()),
      'free_flight_induction_converges':all(a['induction_converged'] for a in flight.values()),
      'mixed_timestep_convergence':measurements['free_flight_120_240_difference']['altitude']<1 and measurements['free_flight_120_240_difference']['rpm']<1,
      'load_driven_damage_retained':flight['highspeed_pull']['first_failure_s'] is None and flight['highspeed_weakened']['first_failure_s'] is not None and 3<flight['highspeed_weakened']['first_failure_s']<flight['highspeed_weakened']['detachment_s'],
      'surface_driven_turn':abs(flight['mouse']['final_velocity_heading_deg']-90)<5,
    }
    report={'component':measurements,'free_flight':flight,'checks':checks,
      'limits':'Synthetic incompressible polars and geometry, uniform axial induction, no swirl/tip/Re/Mach corrections; extreme numerical tests are not transonic performance validation. Single-parent ledger ends before separation.'}
    (RESULTS/'rotor-summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not all(checks.values()):raise SystemExit('Rotor/governor experiment failed: inspect measured trajectories')

if __name__=='__main__':main()
