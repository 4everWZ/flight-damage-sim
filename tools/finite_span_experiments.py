#!/usr/bin/env python3
"""Measure finite-span capability without changing the production polar/model."""
import csv
import io
import json
import math
import subprocess
from experiments import ROOT, RESULTS


def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make', 'build/finite-span-probe'], cwd=ROOT, check=True)
    raw = subprocess.check_output(
        [str(ROOT/'build/finite-span-probe'),
         str(ROOT/'data/validation-material.airframe')], text=True)
    (RESULTS/'finite-span.csv').write_text(raw)
    rows = [{k: v if k == 'case' else float(v) for k, v in row.items()}
            for row in csv.DictReader(io.StringIO(raw))]
    assert len(rows) == 36
    loaded_area = rows[0]['area_m2']
    assert abs(loaded_area - 15) < .001  # Text fixture rounds each cell area.
    # These validate isolation/measurement, not aerodynamic fidelity. The
    # hypotheses below remain ordinary reported booleans, including failures.
    for r in rows:
        assert all(math.isfinite(v) for k, v in r.items() if k != 'case')
        other = next(t for t in rows if t['case'] == r['case']
                     and t['alpha_rad'] == r['alpha_rad']
                     and t['span_scale'] == r['span_scale']
                     and t['hz'] != r['hz'])
        assert r['CL'] == other['CL'] and r['CD'] == other['CD']
        expected_area = loaded_area * (.75 if 'loss' in r['case'] else 1)
        assert abs(r['area_m2'] - expected_area) < 1e-10
        if r['case'] == 'refined':
            baseline = next(t for t in rows if t['case'] == 'geometry'
                            and t['span_scale'] == 1
                            and t['alpha_rad'] == r['alpha_rad']
                            and t['hz'] == r['hz'])
            assert r['cells'] == 2 * baseline['cells']
            assert abs(r['CL'] - baseline['CL']) < 1e-12
            assert abs(r['CD'] - baseline['CD']) < 1e-12
    groups = []
    for angle in (5, 10):
        cr = [r for r in rows if r['case'] == 'geometry' and r['hz'] == 120
              and abs(r['alpha_rad'] - math.radians(angle)) < 1e-12]
        zero = next(r for r in rows if r['case'] == 'geometry'
                    and r['alpha_rad'] == 0 and r['hz'] == 120)
        for r in cr:
            r['CD_minus_CD0'] = r['CD'] - zero['CD']
            r['effective_k_using_total_CL'] = r['CD_minus_CD0'] / r['CL']**2
            # Ideal elliptic-loading lower bound only. No assumed e for the
            # actual rectangle, no high-alpha/damage validity claimed.
            r['elliptic_min_CDi_at_measured_CL'] = r['CL']**2 / (
                math.pi * r['aspect_ratio'])
            r['implied_e_if_excess_CD_is_induced'] = (
                r['elliptic_min_CDi_at_measured_CL'] / r['CD_minus_CD0'])
        tip, distributed = (
            next(r for r in rows if r['case'] == case and r['hz'] == 120
                 and abs(r['alpha_rad'] - math.radians(angle)) < 1e-12)
            for case in ('tip_loss', 'distributed_loss')
        )
        intact = next(r for r in cr if r['span_scale'] == 1)
        groups.append({
            'alpha_deg': angle, 'geometry': cr,
            'equal_area_damage': {'tip_loss': tip, 'distributed_loss': distributed,
                                 'lift_difference_N': tip['lift_N']-distributed['lift_N'],
                                 'drag_difference_N': tip['drag_N']-distributed['drag_N']},
            'hypotheses': {
                'finite_span_changes_CL': max(r['CL'] for r in cr)-min(r['CL'] for r in cr) > 1e-6,
                'finite_span_changes_excess_CD': max(r['CD_minus_CD0'] for r in cr)-min(r['CD_minus_CD0'] for r in cr) > 1e-6,
                'tip_loss_changes_surviving_inner_cell_load': abs(tip['inner_Fz_over_qA']-intact['inner_Fz_over_qA']) > 1e-6,
                'total_CD_above_elliptic_induced_lower_bound_all_geometries': all(
                    r['CD'] >= r['elliptic_min_CDi_at_measured_CL'] for r in cr),
            }})
    summary = {
        'groups': groups,
        'measurement_checks': {'static_120_240_identity': True,
                               'coincident_cell_subdivision_identity': True,
                               'equal_planform_and_lost_area': True},
        'scope': 'Static isolated synthetic wing, zero rates/control/elasticity/propwash; no controller or integration. Total CL includes existing bounded compressibility. CD-CD0 is a measured lift-dependent polar increment, not a proven induced/profile split. Elliptic reference is an inviscid planar-wing lower bound, not calibrated rectangular-wing truth. Damage footprints report geometric span, not a fitted effective AR. No stall/spin/historical validation or acceptance reclassification.'}
    (RESULTS/'finite-span-summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps({str(g['alpha_deg']): g['hypotheses'] for g in groups}, indent=2))


if __name__ == '__main__':
    main()
