#!/usr/bin/env python3
"""Convert the separate NACA reference proposal; does not load/run the core."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LB_KG = 0.45359237
FT_M = 0.3048
G0 = 9.80665
HP_W = 745.6998715822702  # mechanical horsepower, not metric PS


def summarize(profile):
    c = profile['configuration']
    mass = c['test_weight_lb']*LB_KG
    area = c['wing_area_ft2']*FT_M**2
    span = c['span_ft_in']
    span_m = (span[0]+span[1]/12)*FT_M if span else None
    cg = c['cg']
    computed_cg = 100*(cg['behind_root_leading_edge_in']-
                       cg['mac_leading_edge_behind_root_leading_edge_in'])/c['mean_aerodynamic_chord_in']
    return {
        'profile': profile['profile_id'],
        'status': 'source proposal conversion; no simulator comparison or historical validation',
        'test_mass_kg': mass,
        'standard_weight_N': mass*G0,
        'wing_area_m2': area,
        'span_m': span_m,
        'aspect_ratio_from_span_area': span_m**2/area if span_m else None,
        'reported_aspect_ratio': c['reported_aspect_ratio'],
        'test_wing_loading_kg_m2': mass/area,
        'test_wing_loading_N_m2': mass*G0/area,
        'test_wing_loading_lb_ft2': c['test_weight_lb']/c['wing_area_ft2'],
        'normal_gross_wing_loading_lb_ft2': c['normal_gross_weight_lb']/c['wing_area_ft2'],
        'reported_normal_gross_power_loading_lb_hp': c['reported_normal_gross_power_loading_lb_hp'],
        'normal_gross_weight_over_listed_normal_rating_lb_hp': c['normal_gross_weight_lb']/c['engine']['normal_rating']['brake_hp'],
        'reported_cg_percent_mac': cg['reported_percent_mac'],
        'cg_percent_from_rounded_inch_dimensions': computed_cg,
        'cg_note': 'Rounded approximate dimensions need not reproduce reported CG exactly; preserve both.',
        'propeller_diameter_m': (c['propeller']['diameter_ft_in'][0]+
                                 c['propeller']['diameter_ft_in'][1]/12)*FT_M,
        'specified_engine_ratings': {
            name: {'shaft_power_W': rating['brake_hp']*HP_W,
                   'specified_shaft_power_per_test_mass_W_kg': rating['brake_hp']*HP_W/mass,
                   'engine_rpm': rating['engine_rpm'],
                   'specified_altitude_m': rating['altitude_ft']*FT_M if rating['altitude_ft'] is not None else None}
            for name, rating in c['engine'].items() if name.endswith('_rating')
        },
        'rating_note': 'Specification power only; no installed shaft-power/thrust measurement or interpolated altitude map.',
        'historical_comparisons_run': 0,
        'material_calibration': None
    }


if __name__ == '__main__':
    proposal = json.loads((ROOT/'data/spitfire-va-naca-reference.json').read_text())
    result = summarize(proposal)
    dest = ROOT/'results/spitfire-source'
    dest.mkdir(parents=True, exist_ok=True)
    (dest/'reference-summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
