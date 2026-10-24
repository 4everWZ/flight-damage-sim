#!/usr/bin/env python3
"""Preselected measured-curve densification; never assumes a source Mach."""
import copy
import csv
import json
from bisect import bisect_right
from pathlib import Path

from control_polar_calibration import export_control_rows

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results'
COEFFICIENTS = ('cl', 'cd', 'cm')


def weights_at(train, observed):
    aa = sorted({r['alpha_deg'] for r in train})
    dd = sorted({r['delta_deg'] for r in train})
    points = {(r['alpha_deg'], r['delta_deg']) for r in train}
    if len(points) != len(train) or points != {(a,d) for a in aa for d in dd}:
        raise ValueError('training view must be a complete rectangle without duplicates')
    a, d = observed['alpha_deg'], observed['delta_deg']
    if not aa[0] < a < aa[-1] or not dd[0] <= d <= dd[-1]:
        raise ValueError('held-out query outside training coverage; no extrapolation')
    ai = min(bisect_right(aa,a), len(aa)-1)
    di = min(bisect_right(dd,d), len(dd)-1)
    al, ah = aa[ai-1:ai+1]; dl, dh = dd[di-1:di+1]
    wa, wd = (a-al)/(ah-al), (d-dl)/(dh-dl)
    return {p:w for p,w in { (al,dl):(1-wa)*(1-wd), (ah,dl):wa*(1-wd),
                            (al,dh):(1-wa)*wd, (ah,dh):wa*wd }.items() if w > 0}


def compare(train, observed, default_allowance, experiment):
    by_key = {(r['alpha_deg'], r['delta_deg']):r for r in train}
    weights = weights_at(train, observed)
    result = []
    for key in COEFFICIENTS:
        predicted = sum(w*by_key[p][key] for p,w in weights.items())
        bound = observed.get('reading_halfwidth', default_allowance)[key]
        bound += sum(abs(w)*by_key[p].get('reading_halfwidth',default_allowance)[key]
                     for p,w in weights.items())
        result.append({'experiment':experiment, 'heldout_id':observed['id'], 'coefficient':key,
                       'observed':observed[key], 'predicted':predicted,
                       'residual_predicted_minus_observed':predicted-observed[key],
                       'reading_only_residual_halfwidth':bound,
                       'training_ids':','.join(by_key[p]['id'] for p in weights)})
    return result


def main():
    base = json.loads((ROOT/'data/control-polar-naca0009-measured-subset.json').read_text())
    dense = json.loads((ROOT/'data/control-polar-naca0009-dense-slice.json').read_text())
    assert dense['conditions_inherited_from'] == 'control-polar-naca0009-measured-subset.json'
    assert base['conditions']['mach'] is None
    assert dense['preselected_training_additions'] == [[-4,5],[4,5],[-4,20],[4,20]]
    assert dense['preselected_heldout_additions'] == [[0,15]]
    added = dense['rows']
    old_train = [r for r in base['rows'] if r['role']=='train']
    old_hold = [r for r in base['rows'] if r['role']=='heldout']
    train = old_train+[r for r in added if r['role']=='train']
    heldout = old_hold+[r for r in added if r['role']=='heldout']
    assert {(r['alpha_deg'],r['delta_deg']) for r in train} == {(a,d) for a in (-4,4) for d in (0,5,10,20)}
    assert {(r['alpha_deg'],r['delta_deg']) for r in heldout} == {(0,5),(0,15)}
    assert not {r['id'] for r in train} & {r['id'] for r in heldout}
    allowance = base['uncertainty']['coefficient_reading_halfwidth']
    # Trace original manual pixel selections through the same declared local
    # scan axes. No image feature detector or fitted curve generates data.
    axes = base['scan_calibration']
    pixel_error = {key:0 for key in COEFFICIENTS}
    for row in added:
        x = row['pixels']['lift'][0]
        read = {'cl':(x-axes['cl_axis']['zero_x_px'])/axes['cl_axis']['pixels_per_unit']}
        for key,panel in [('cd','drag'),('cm','moment')]:
            x,y = row['pixels'][panel]; axis = axes[key+'_axis']
            y0 = axis['zero_y_at_x818_px']+(x-818)*axis['zero_line_dy_per_dx']
            read[key] = (y0-y)/axis['pixels_per_unit']
        for key in COEFFICIENTS:
            error = abs(read[key]-row[key]); pixel_error[key] = max(pixel_error[key],error)
            assert error <= row['reading_halfwidth'][key]
    results = compare(old_train,old_hold[0],allowance,'frozen_four_corner')
    assert all(abs(r['residual_predicted_minus_observed']-expected)<1e-12
               for r,expected in zip(results,(-.15,.00375,.0345)))
    results += compare(train,heldout[0],allowance,'dense_alpha_interpolation_at_seen_delta_5')
    results += compare(train,heldout[1],allowance,'dense_unseen_delta_15')
    try:
        weights_at(old_train,heldout[1])
    except ValueError:
        old_15 = 'Outside frozen delta coverage [0,10]; no extrapolated comparison.'
    else:
        raise AssertionError('frozen table must not extrapolate to delta 15')
    try:
        export_control_rows({**base,'rows':train},copy.deepcopy(base['conditions']),'not_eligible')
    except ValueError as error:
        blocked = str(error)
        assert 'positive fixed source Mach' in blocked
    else:
        raise AssertionError('densification must not silently supply unknown source Mach')
    summary = {'meaning':'Measured faired-curve reading/interpolation only; not native force replay or physical calibration.',
               'preselection':dense['preselection'], 'conditions':base['conditions'],
               'source_convention_review':dense['source_convention_review'],
               'training_count':len(train),'heldout_count':len(heldout),
               'source_uncertainty':base['uncertainty'],
               'new_reading_allowance':dense['reading_allowance_note'],
               'heldout_comparisons':results,'frozen_delta15_status':old_15,
               'new_pixel_rounding_max_difference':pixel_error,
               'runtime_export_blocked':blocked, 'physical_validation':False,
               'limits':['Delta 5 now appears at training alpha endpoints: its center tests alpha interpolation, not an unseen deflection.',
                        'Delta 15 remains absent from training. All readings share the same faired source experiment and correlated systematic uncertainty.',
                        'Reading-only bounds exclude source angle errors and unknown absolute-drag/tunnel corrections; no statistical confidence or physical pass criterion.',
                        'The source +/-.3 degree effective-alpha error is specified at zero lift, not a global bound for all alpha.',
                        'Mach, transition and sectional/effective-component transfer remain unqualified. The horizontal-section CM sign maps without reversal through the directly cited NACA721 convention; this does not resolve source Mach or a quarter-chord-to-CG load transfer.'],
               'next_experiment':'Resolve source flow/convention evidence before any native replay; if coefficient interpolation is extended, preselect additional unseen deflections and retain current held-out readings.'}
    OUT.mkdir(exist_ok=True)
    (OUT/'control-polar-dense-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    with (OUT/'control-polar-dense-heldout.csv').open('w') as stream:
        writer = csv.DictWriter(stream,fieldnames=list(results[0]));writer.writeheader();writer.writerows(results)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
