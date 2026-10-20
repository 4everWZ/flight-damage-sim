#!/usr/bin/env python3
"""Small coefficient-space held-out experiment; no assumed source Mach or flights."""
import copy
import csv
import json
from pathlib import Path

from control_polar_calibration import export_control_rows, normalized_rows

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results'
COEFFICIENTS = ('cl', 'cd', 'cm')


def main():
    source = json.loads((ROOT/'data/control-polar-naca0009-measured-subset.json').read_text())
    rows = source['rows']
    train = [r for r in rows if r['role'] == 'train']
    heldout = [r for r in rows if r['role'] == 'heldout']
    aa = sorted({r['alpha_deg'] for r in train})
    dd = sorted({r['delta_deg'] for r in train})
    assert len(train) == 4 and len(aa) == len(dd) == 2 and len(heldout) == 1
    by_key = {(r['alpha_deg'], r['delta_deg']): r for r in train}
    assert len(by_key) == 4
    allowance = source['uncertainty']['coefficient_reading_halfwidth']
    # Direct numerical check of the documented pixel/axis reading. This does
    # not independently validate curve identity or replace reading uncertainty.
    axes = source['scan_calibration']
    trace_errors = {k: 0 for k in COEFFICIENTS}
    for r in rows:
        x = r['pixels']['lift'][0]
        decoded = {'cl': (x-axes['cl_axis']['zero_x_px'])/axes['cl_axis']['pixels_per_unit']}
        for key, panel in [('cd', 'drag'), ('cm', 'moment')]:
            x, y = r['pixels'][panel]
            axis = axes[key+'_axis']
            y0 = axis['zero_y_at_x818_px'] + (x-818)*axis['zero_line_dy_per_dx']
            decoded[key] = (y0-y)/axis['pixels_per_unit']
        for key in COEFFICIENTS:
            trace_errors[key] = max(trace_errors[key], abs(decoded[key]-r[key]))
            assert abs(decoded[key]-r[key]) <= allowance[key]
    results = []
    for observed in heldout:
        wa = (observed['alpha_deg']-aa[0])/(aa[1]-aa[0])
        wd = (observed['delta_deg']-dd[0])/(dd[1]-dd[0])
        assert 0 < wa < 1 and 0 < wd < 1
        weights = {(aa[0],dd[0]): (1-wa)*(1-wd), (aa[1],dd[0]): wa*(1-wd),
                   (aa[0],dd[1]): (1-wa)*wd, (aa[1],dd[1]): wa*wd}
        for key in COEFFICIENTS:
            predicted = sum(w*by_key[p][key] for p, w in weights.items())
            # Positive interpolation weights: corner reading allowances add
            # linearly, then add the independent held-out reading allowance.
            reading_bound = (sum(abs(w) for w in weights.values())+1)*allowance[key]
            results.append({'heldout_id': observed['id'], 'coefficient': key,
                            'observed': observed[key], 'predicted': predicted,
                            'residual_predicted_minus_observed': predicted-observed[key],
                            'reading_only_residual_halfwidth': reading_bound})
    try:
        export_control_rows(source, copy.deepcopy(source['conditions']), 'not_eligible')
    except ValueError as error:
        blocked = str(error)
    else:
        raise AssertionError('unknown source Mach must not silently become a matched runtime export')

    # Separate software counterexample, with no measured coefficient content:
    # previously matching ignored a declared rear-flap linkage discrepancy.
    software = json.loads((ROOT/'data/control-polar-camrad-neutral-reference.json').read_text())
    software['evidence_kind'] = 'synthetic_integration'
    software['conditions']['airfoil'] = 'synthetic-configuration-guard'
    software['conditions']['geometry_role'] = 'manufactured two-hinge identity guard'
    software['rows'] = [{'alpha_deg': 0, 'delta_deg': 0, 'cl': 0, 'cd': .01, 'cm': 0}]
    software['conditions']['configuration'] = copy.deepcopy(source['conditions']['configuration'])
    normalized_rows(software, copy.deepcopy(software['conditions']))
    rejected = []
    for key, value in [('rear_relative_delta_ratio', 2), ('rear_flap_chord_over_c', .09), ('rear_gap', 'open')]:
        target = copy.deepcopy(software['conditions'])
        target['configuration'][key] = value
        try:
            normalized_rows(software, target)
        except ValueError:
            rejected.append(key)
        else:
            raise AssertionError('contradictory declared control configuration was accepted')
    summary = {'meaning': 'Measured faired-curve subset, coefficient-space interpolation only. Not a native force-kernel replay or physical aircraft calibration.',
               'source': source['source'], 'conditions': source['conditions'],
               'uncertainty': source['uncertainty'],
               'training_rows': [r['id'] for r in train], 'heldout_rows': [r['id'] for r in heldout],
               'heldout_comparison': results, 'pixel_rounding_max_difference': trace_errors,
               'runtime_export_blocked': blocked,
               'additional_unresolved_export_requirements': ['transition and measured Mach/temperature',
                   'consistent report-to-native coefficient/deflection signs',
                   'a complete rectangular grid; retain the held-out point for evaluation',
                   'explicit effective-component versus sectional/induced-drag mapping'],
               'configuration_guard_synthetic_rejections': rejected,
               'physical_validation': False,
               'interpretation': 'The coarse corner grid misses the central CL/CM curve reading beyond the reading-only allowances. Unknown systematic and angle errors prevent a total physical-error claim. No extra delta rows or corrections are fitted to make it pass.',
               'next_experiment': 'Read additional independently held-out deflection stations with the same linkage/configuration; qualify sign and flow-condition evidence before runtime export. Preserve absolute-drag correction uncertainty.'}
    OUT.mkdir(exist_ok=True)
    (OUT/'control-polar-measured-summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    with (OUT/'control-polar-measured-heldout.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0]))
        writer.writeheader(); writer.writerows(results)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
