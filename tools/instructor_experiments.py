#!/usr/bin/env python3
"""Compare corrected dynamics with old/off/new guidance, using the same pilot inputs."""
import csv
import json
import math
import subprocess
from experiments import ROOT,RESULTS,run,at


def angular_error(angle,target):
    return math.atan2(math.sin(angle-target),math.cos(angle-target))


def metrics(rows,scenario):
    heading_target=.08*rows[-1]['time'] if scenario=='mouse_circle' else 0 if scenario=='mouse_climb' else math.pi/2
    path_target=math.radians(8) if scenario=='mouse_climb' else 0
    tail=[r for r in rows if r['time']>=20]
    within=[r for r in rows if abs(angular_error(r['air_heading'],.08*r['time'] if scenario=='mouse_circle' else heading_target))<math.radians(5)]
    return {'final_altitude_change_m':rows[-1]['altitude']-rows[0]['altitude'],
            'minimum_altitude_change_m':min(r['altitude'] for r in rows)-rows[0]['altitude'],
            'final_air_heading_deg':math.degrees(rows[-1]['air_heading']),
            'final_nose_heading_deg':math.degrees(rows[-1]['heading']),
            'peak_absolute_altitude_change_m':max(abs(r['altitude']-rows[0]['altitude']) for r in rows),
            'final_heading_error_deg':math.degrees(angular_error(rows[-1]['air_heading'],heading_target)),
            'first_within_5deg_s':within[0]['time'] if within else None,
            'final_sideslip_deg':math.degrees(rows[-1]['beta']),
            'final_path_angle_deg':math.degrees(rows[-1]['path_angle']),
            'tail_path_error_rms_deg':math.degrees(math.sqrt(sum((r['path_angle']-path_target)**2 for r in tail)/len(tail))),
            'max_normal_g':max(r['g'] for r in rows),
            'max_bank_deg':math.degrees(max(abs(r['bank']) for r in rows)),
            'pitch_saturation_s':sum(r['pitch_saturated']*rows[1]['time'] for r in rows),
            'final_speed_mps':rows[-1]['speed'],
            'altitude_change_at_10s_m':at(rows,10)['altitude']-rows[0]['altitude'],
            'reference_ground_crossing_s':next((r['time'] for r in rows if r['altitude']<=0),None)}


def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make','-j2'],cwd=ROOT,check=True)
    table={}
    tracks={}
    for mode in ['off','legacy','flightpath-p','flightpath']:
        for scenario in ['mouse','mouse_damaged','mouse_disconnected','mouse_climb']:
            key=scenario+'_'+mode
            rows,_=run(scenario,seconds=30,suffix='_'+mode,controller=mode)
            table[key]=metrics(rows,scenario);tracks[key]=rows
    for bank in [50,60]:
        for scenario in ['mouse','mouse_damaged']:
            key=scenario+'_bank'+str(bank)
            rows,_=run(scenario,seconds=30,suffix='_bank'+str(bank),bank=bank)
            table[key]=metrics(rows,scenario)
    fine,_=run('mouse',hz=240,seconds=30,suffix='_new_240hz')
    coarse=tracks['mouse_flightpath']
    differences={k:abs(fine[-1][k]-coarse[-1][k]) for k in ['altitude','speed','air_heading','beta']}
    clean=table['mouse_flightpath'];old=table['mouse_legacy'];damaged=table['mouse_damaged_flightpath'];cut=table['mouse_disconnected_flightpath']
    checks={
        'turn_tracks_air_direction':abs(clean['final_heading_error_deg'])<2 and abs(clean['final_sideslip_deg'])<2,
        'smaller_full_turn_height_loss':clean['minimum_altitude_change_m']>old['minimum_altitude_change_m']+10,
        'small_final_level_error':abs(clean['final_altitude_change_m'])<5 and clean['tail_path_error_rms_deg']<1,
        'local_damage_still_tracks_with_real_surfaces':abs(damaged['final_heading_error_deg'])<3 and damaged['minimum_altitude_change_m']>-35,
        'cut_elevator_cannot_restore_lift':cut['altitude_change_at_10s_m']<-100 and abs(tracks['mouse_disconnected_flightpath'][-1]['elevator'])<1e-8,
        'no_assistance_does_not_follow_aim':abs(table['mouse_off']['final_air_heading_deg'])<1,
        'climb_intent_changes_flight_path':table['mouse_climb_flightpath']['final_path_angle_deg']>2 and table['mouse_climb_flightpath']['final_altitude_change_m']>100,
        'timestep_convergence':differences['altitude']<.5 and differences['air_heading']<.002,
        'normal_loop_ablation_worse':table['mouse_flightpath-p']['minimum_altitude_change_m']<clean['minimum_altitude_change_m']-10,
    }
    for mode in ['legacy','flightpath']:
        rows,_=run('mouse_circle',seconds=60,suffix='_'+mode,controller=mode)
        table['mouse_circle_'+mode]=metrics(rows,'mouse_circle')
    circle=table['mouse_circle_flightpath'];circle_old=table['mouse_circle_legacy']
    checks['sustained_turn_path_tracking']=circle['tail_path_error_rms_deg']<.15 and circle['tail_path_error_rms_deg']<circle_old['tail_path_error_rms_deg']*.7 and abs(circle['final_heading_error_deg'])<20
    hypotheses={'every_turn_has_less_minimum_height_loss':circle['minimum_altitude_change_m']>circle_old['minimum_altitude_change_m'],
                'note':'The broad height-loss hypothesis failed: legacy circle climbs while the new controller initially descends. Compare air-path error and the entire trajectory; no zero-loss claim.'}
    report={'aim_semantics':'world-frame air-relative velocity direction; positive elevation requests climb, no altitude setpoint',
            'hypotheses':hypotheses,'measurements':table,'120_240_final_difference':differences,'checks':checks}
    (RESULTS/'instructor-summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not all(checks.values()):raise SystemExit('Instructor acceptance failed; inspect fresh telemetry')

if __name__=='__main__':main()
