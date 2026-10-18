#!/usr/bin/env python3
"""Measure current synthetic capability and map source inputs, without fitting."""
import csv
import json
import math
import subprocess
from pathlib import Path
from spitfire_reference_summary import summarize

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT/'results'


def rows(path):
    with path.open() as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make', '-j2', 'build/reference-gap-probe', 'build/flight-run'], cwd=ROOT, check=True)
    prefix = RESULTS/'reference-gap'
    subprocess.run([str(ROOT/'build/reference-gap-probe'), str(ROOT/'data/validation-material.airframe'), str(prefix)], check=True)
    inventory = rows(prefix.with_name('reference-gap-inventory.csv'))[0]
    control = rows(prefix.with_name('reference-gap-controls.csv'))
    static = rows(prefix.with_name('reference-gap-static.csv'))
    basis = rows(prefix.with_name('reference-gap-basis.csv'))
    source = json.loads((ROOT/'data/spitfire-va-naca-reference.json').read_text())
    converted = summarize(source)
    output = subprocess.check_output([str(ROOT/'build/flight-run'), str(ROOT/'data/validation-material.airframe'),
                                     'clean', str(RESULTS/'reference-gap-flight'), '120', '4', 'off'], text=True)
    flight = rows(RESULTS/'reference-gap-flight.csv')
    assert all(math.isfinite(v) for group in (control, static, flight, basis) for r in group for v in r.values())
    # Equation closure and observable definitions only, never historical tolerances.
    effort_error = max(abs(r['left_chain_rad']-r['normalized_request']*r['left_limit_rad']/(1+r['q_Pa']/r['left_q_limit_Pa'])) for r in control)
    first_inner = next((r['body_alpha_deg'] for r in static if r['inner_stalled']), None)
    first_outer = next((r['body_alpha_deg'] for r in static if r['outer_stalled']), None)
    cfg = source['configuration']
    reference_rpm = {name: value['engine_rpm']*cfg['propeller']['reported_gear_ratio']
                     for name, value in cfg['engine'].items() if name.endswith('_rating')}
    report = {
        'scope': 'Unmodified synthetic fixture capability only; no historical flight-performance comparison is admissible yet.',
        'historical_comparisons_run': 0,
        'historical_acceptance_tolerances': None,
        'fixture': inventory,
        'source_dimensions_only': {k: converted[k] for k in ('test_mass_kg','wing_area_m2','span_m','test_wing_loading_kg_m2')},
        'dimension_inventory_comparison': {
            'fixture_wing_loading_kg_m2': inventory['mass_kg']/inventory['wing_area_m2'],
            'note': 'Same physical units permit an inventory comparison, not performance validation; datum/CG and inertia cannot be transferred by scaling total mass.'},
        'rpm_mapping': {'reference_prop_output_rpm_by_engine_rating': reference_rpm,
                        'reference_cruising_engine_rpm': 2650,
                        'reference_cruising_prop_output_rpm': 2650*cfg['propeller']['reported_gear_ratio'],
                        'gear_efficiency': None,
                        'note': 'Core RPM is propeller/output-shaft RPM. This is a kinematic conversion, not a calibrated governor/power map. No engine setting is applied to the fixture.'},
        'control_bench': {'rows': control, 'settled_declared_effort_equation_error_rad': effort_error,
                          'measured_stick_force_lbf': None,
                          'note': 'Two-second actuator-only bench at fixed TAS/density; no body/structure/rotor integration. Commands have no lbf mapping.'},
        'static_aero_bench': {'samples': len(static), 'first_inner_stall_body_alpha_deg': first_inner,
                              'first_outer_stall_body_alpha_deg': first_outer,
                              'peak_wing_CL': max(static,key=lambda r:r['wing_CL']),
                              'note': 'Fixed-state polar sweep, prop absent, undeformed structure and neutral surfaces. Not a throttled glide, trim solution, measured CLmax or buffet test. Stall flags use the current fixed 15-degree rule.'},
        'prescribed_panel_geometry_witness': {'rows': basis,
            'independent_analytic_angle_error_rad': max(abs(r['expected_geometric_alpha_rad']-
                math.atan2(r['side']*5*math.sin(math.radians(6)),70)) for r in basis),
            'note': 'Two prescribed flat panels at 6 degrees dihedral, zero incidence, flow (70,5,0) m/s. This is a geometry capability counterexample, not a historical aircraft baseline. Positions alone do not rotate local span/chord/normal frames.'},
        'synthetic_trimmed_flight': {'entry': output.strip(), 'hz': 120, 'seconds': 4,
                                    'altitude_change_m': flight[-1]['altitude']-flight[0]['altitude'],
                                    'speed_change_mps': flight[-1]['speed']-flight[0]['speed'],
                                    'initial_output_shaft_rpm': flight[0]['rpm'],
                                    'initial_shaft_power_W': flight[0]['shaft_power'],
                                    'note': 'Real 6DoF integration on existing synthetic data, calm wind, controller off; trim solves throttle and normalized trims, not report manifold pressure or physical trim tabs.'},
        'next_proposal': 'Per-cell reference frames are now implemented by PR16; the retained positions-only comparator intentionally omits them. Next: source-conditioned local stall/control-authority experiments with explicit table and propulsion semantics. No new polar, stall law or material calibration is adopted by this integration.'
    }
    (RESULTS/'reference-gap-summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
