#!/usr/bin/env python3
"""Isolated CR-3439 measured section holdout through the existing native kernel.

Checks input/operator fidelity; reports every physical-data residual without
tuning coefficients, digitization allowances, controls or default aircraft.
"""
import copy
import csv
import json
import math
import subprocess
from bisect import bisect_right

from control_polar_calibration import ROOT, OUT, BUILD, export_control_rows, kernel_file, replay

COEFFICIENTS = ('cl', 'cd', 'cm')


def interpolation_weights(train, query):
    aa = sorted({r['alpha_deg'] for r in train})
    dd = sorted({r['delta_deg'] for r in train})
    a, d = query['alpha_deg'], query['delta_deg']
    if not aa[0] <= a <= aa[-1] or not dd[0] <= d <= dd[-1]:
        raise ValueError('outside preselected rectangle; no extrapolation')
    ai = min(max(bisect_right(aa, a), 1), len(aa)-1)
    di = min(max(bisect_right(dd, d), 1), len(dd)-1)
    al, ah = aa[ai-1:ai+1]; dl, dh = dd[di-1:di+1]
    wa, wd = (a-al)/(ah-al), (d-dl)/(dh-dl)
    return {p:w for p,w in [((al,dl),(1-wa)*(1-wd)), ((ah,dl),wa*(1-wd)),
                            ((al,dh),(1-wa)*wd), ((ah,dh),wa*wd)] if w > 0}


def check_scan_trace(dataset):
    allowance = dataset['uncertainty']['coefficient_reading_halfwidth']
    axes = dataset['scan_calibration']
    max_rounding = {k:0. for k in COEFFICIENTS}
    for row in dataset['rows']:
        for key, panel in zip(COEFFICIENTS, ('lift','drag','moment')):
            x,y = row['pixels'][panel]; axis = axes[panel]
            y0 = axis['zero_y_px']+(x-axis['zero_line_reference_x_px'])*axis['zero_line_dy_per_dx']
            value = (y0-y)/axis['pixels_per_unit']
            difference = abs(row[key]-value)
            assert difference <= allowance[key], (row['id'],key,difference)
            max_rounding[key] = max(max_rounding[key],difference)
            if key == 'cl':
                # Approximate grid anchors locate the requested faired-curve
                # station, not necessarily an original balance measurement.
                read_alpha = (x-axis['alpha_zero_x_px'])/axis['alpha_pixels_per_deg']
                assert abs(read_alpha-row['alpha_deg']) < .06
            else:
                read_cl = (x-axis['cl_zero_x_px'])/axis['cl_pixels_per_unit']
                assert abs(read_cl-row['cl']) <= allowance['cl']
    return max_rounding


def main():
    data = json.loads((ROOT/'data/control-polar-cr3439-section.json').read_text())
    train = [r for r in data['rows'] if r['role']=='train']
    held = [r for r in data['rows'] if r['role']=='heldout']
    select = data['preselection']
    assert {(r['alpha_deg'],r['delta_deg']) for r in train} == {
        (a,d) for a in select['training_alpha_deg'] for d in select['training_delta_deg']}
    assert {(r['alpha_deg'],r['delta_deg']) for r in held} == {
        (a,d) for a in select['heldout_alpha_deg'] for d in select['heldout_entire_delta_curves_deg']}
    assert len(train)==6 and len(held)==6
    assert not {r['delta_deg'] for r in train} & {r['delta_deg'] for r in held}
    pixel_error = check_scan_trace(data)
    target = copy.deepcopy(data['conditions'])
    training_data = {**data, 'rows':train}
    records = export_control_rows(training_data, target, 'cr3439')
    assert 'cell_control_polar' not in records
    # Export metadata is an exact convention identity, not evidence that the
    # normalized native fixture physically has the source shape, Re or trips.
    rejected = []
    for key,value in [('mach',.14), ('reynolds',3300000), ('flap_gap','sealed'),
                      ('delta_positive','trailing_edge_up'), ('moment_positive','nose_down')]:
        try:
            export_control_rows(training_data, {**target,key:value}, 'invalid_cr3439')
        except ValueError:
            rejected.append(key)
        else:
            raise AssertionError('incompatible source condition silently accepted')
    OUT.mkdir(exist_ok=True); BUILD.mkdir(exist_ok=True)
    subprocess.run(['make','-j2','build/control-polar-reference-probe'], cwd=ROOT, check=True)
    fixture = kernel_file((ROOT/'data/validation-material.airframe').read_text(),records,'cr3439',True)
    # Keep the table and its six excluded observations separately reviewable.
    (OUT/'control-polar-cr3439-training.records').write_text(records)
    actual = replay(fixture,target['mach'],train+held,'cr3439')
    assert len(actual)==12
    # The strict native diagnostic also flags a one-ULP actuator roundoff
    # beyond a table endpoint. Preserve that observation instead of changing
    # runtime clamping or shrinking the preselected measurements.
    a_lo,a_hi = map(math.radians,select['training_alpha_deg'])
    d_lo,d_hi = map(math.radians,(min(select['training_delta_deg']),max(select['training_delta_deg'])))
    outside_distance = max(max(a_lo-r['flow_alpha_rad'],r['flow_alpha_rad']-a_hi,
                               d_lo-r['delta_eff_rad'],r['delta_eff_rad']-d_hi,0.) for r in actual)
    assert outside_distance < 1e-12
    for r in actual:
        if r['clamped']:
            assert min(abs(r['flow_alpha_rad']-a_lo),abs(r['flow_alpha_rad']-a_hi),
                       abs(r['delta_eff_rad']-d_lo),abs(r['delta_eff_rad']-d_hi)) < 1e-12
    angle_error = max(abs(r['delta_eff_rad']-math.radians(q['delta_deg'])) for r,q in zip(actual,train+held))
    flow_error = max(abs(r['flow_alpha_rad']-math.radians(q['alpha_deg'])) for r,q in zip(actual,train+held))
    assert angle_error < 1e-12 and flow_error < 1e-12
    knot_error = max(abs(r[k]-q[k]) for r,q in zip(actual[:len(train)],train) for k in COEFFICIENTS)
    assert knot_error < 1e-12
    by_point = {(r['alpha_deg'],r['delta_deg']):r for r in train}
    allowance = data['uncertainty']['coefficient_reading_halfwidth']
    comparisons = []
    operator_error = 0.
    for measured,native in zip(held,actual[len(train):]):
        weights = interpolation_weights(train,measured)
        for key in COEFFICIENTS:
            independent = sum(w*by_point[p][key] for p,w in weights.items())
            operator_error = max(operator_error,abs(native[key]-independent))
            residual = native[key]-measured[key]
            # Worst-case arithmetic envelope of digitization allowances only.
            # It omits angle errors and systematic corrections; no physical
            # pass/fail assertion depends on whether a residual lies inside.
            bound = allowance[key]*(1+sum(abs(w) for w in weights.values()))
            comparisons.append({'id':measured['id'],'alpha_deg':measured['alpha_deg'],
                                'delta_deg':measured['delta_deg'],'coefficient':key,
                                'observed':measured[key],'native_prediction':native[key],
                                'independent_bilinear_prediction':independent,
                                'residual_prediction_minus_observed':residual,
                                'reading_only_residual_halfwidth':bound,
                                'outside_reading_only_envelope':abs(residual)>bound+1e-12,
                                'training_ids':','.join(by_point[p]['id'] for p in weights)})
    assert operator_error < 1e-12
    summary = {
        'meaning':'Fixed-condition, low-angle measured-curve comparison with a normalized native force-kernel replay; no aircraft calibration.',
        'source':data['source'], 'conditions':target, 'conventions':data['conventions'],
        'preselection':select, 'training_count':len(train), 'heldout_count':len(held),
        'scan_rounding_max_difference':pixel_error, 'uncertainty':data['uncertainty'],
        'software_checks':{'normalized_training_knot_max_error':knot_error,
                           'heldout_independent_operator_max_error':operator_error,
                           'actual_control_angle_max_error_rad':angle_error,
                           'actual_flow_alpha_max_error_rad':flow_error,
                           'max_absolute_real_chain_command':max(abs(r['command']) for r in actual),
                           'strict_boundary_clamp_flags':sum(int(r['clamped']) for r in actual),
                           'max_angle_outside_table_rad':outside_distance,
                           'boundary_flag_meaning':'Strict native endpoint comparison includes one-ULP actuator roundoff; raw flags retained. No extrapolated angle beyond 1e-12 rad is accepted.',
                           'incompatible_conditions_rejected':rejected},
        'heldout_comparisons':comparisons,
        'max_absolute_heldout_residual':{k:max(abs(r['residual_prediction_minus_observed']) for r in comparisons if r['coefficient']==k) for k in COEFFICIENTS},
        'outside_reading_only_envelope_count':{k:sum(r['outside_reading_only_envelope'] for r in comparisons if r['coefficient']==k) for k in COEFFICIENTS},
        'limits':data['limits']+[
            'Knot/operator closure tests software preparation, bilinear lookup and the shared native load path; it cannot validate the imported coefficient law.',
            'Independent held-out curves share the same source experiment and systematic errors; they are not independent wind-tunnel repeats.',
            'CL is divided by the existing PG(M=.13) before export and restored by the native kernel; this is fixed-condition bookkeeping, not inferred incompressible data.',
            'The current native force law has no Reynolds/transition/gap axis; those are source table identities, not independently simulated conditions.',
            'No runtime/default-airframe, NACA0009 data/allowances/results, authority or stall-control law is changed.'],
        'physical_aircraft_validation':False,
        'next_experiment':'Retain these unseen-curve discrepancies. Before denser interpolation or section-to-aircraft adoption, preselect another comparison and explicitly address geometry/reference transfer; do not tune this held-out set into a pass.'}
    (OUT/'control-polar-cr3439-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    with (OUT/'control-polar-cr3439-heldout.csv').open('w') as stream:
        writer = csv.DictWriter(stream,fieldnames=list(comparisons[0])); writer.writeheader();writer.writerows(comparisons)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
