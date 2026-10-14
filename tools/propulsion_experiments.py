#!/usr/bin/env python3
"""Probe the same authoritative propulsion step used in flight; retain failed model hypotheses."""
import csv
import io
import json
import math
import subprocess
from experiments import ROOT,RESULTS


def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make','-j2'],cwd=ROOT,check=True)
    completed=subprocess.run([str(ROOT/'build/propulsion-probe'),str(ROOT/'data/validation.airframe')],check=True,capture_output=True,text=True)
    (RESULTS/'propulsion.csv').write_text(completed.stdout)
    rows=[{k:v if k=='case' else float(v) for k,v in row.items()} for row in csv.DictReader(io.StringIO(completed.stdout))]
    powered=[r for r in rows if r['case']=='powered' and r['hz']==120]
    stopped=next(r for r in rows if r['case']=='stopped_in_flow' and r['hz']==120)
    absent=next(r for r in rows if r['case']=='removed_prop' and r['hz']==120)
    rotor_shutdown=next(r for r in rows if r['case']=='shutdown' and r['hz']==120)
    steady_rpm_difference=max(abs(a['rpm']-b['rpm']) for a in rows if a['case']=='powered' and a['hz']==120 for b in rows if b['case']=='powered' and b['hz']==240 and a['speed']==b['speed'])
    checks={
        'same_shaft_torque_applied_to_rotor_and_mount':max(r['max_torque_pair_error_Nm'] for r in rows)<1e-7,
        'powered_useful_energy_not_above_shaft_budget':all(r['useful_power']<=r['shaft_power']+1e-6 for r in powered),
        'shutdown_has_no_engine_power':rotor_shutdown['shaft_power']==0 and rotor_shutdown['rpm']>0,
        'rotor_timestep_convergence':steady_rpm_difference<.1,
        'finite_probe':all(math.isfinite(v) for r in rows for k,v in r.items() if k!='case'),
    }
    hypotheses={
        'stopped_intact_prop_generates_drag_in_flow':stopped['thrust']<-.01,
        'wind_starts_windmilling_from_rest':stopped['rpm']>1,
        'fixed_pitch_load_changes_with_airspeed':max(r['prop_aero_torque'] for r in powered)-min(r['prop_aero_torque'] for r in powered)>1,
    }
    measurements={
        'max_torque_pair_error_Nm':max(r['max_torque_pair_error_Nm'] for r in rows),
        'steady_powered_120_240_rpm_difference':steady_rpm_difference,
        'powered_sample_max_useful_efficiency':max(r['useful_power']/r['shaft_power'] for r in powered),
        'intact_stopped_thrust_N':stopped['thrust'],'removed_stopped_thrust_N':absent['thrust'],
        'shutdown_30s_rpm':rotor_shutdown['rpm'],'shutdown_30s_drag_N':rotor_shutdown['thrust'],
        'powered_airspeed_rpm_spread':max(r['rpm'] for r in powered)-min(r['rpm'] for r in powered),
    }
    report={'measurements':measurements,'checks':checks,'hypotheses':hypotheses,
            'scope':'isolated axial component bench, not flight, governor or full gyro/rotor-body energy validation',
            'next_experiment':'Signed CT/CQ versus advance ratio and blade pitch, or explicit blade elements, before adding a constant-speed governor; no synthetic drag patch.'}
    (RESULTS/'propulsion-summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not all(checks.values()):raise SystemExit('Propulsion implementation consistency failed; inspect telemetry')

if __name__=='__main__':main()
