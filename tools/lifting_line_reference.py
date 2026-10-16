#!/usr/bin/env python3
"""Independent steady/incompressible/linear lifting-line experiment, never live FM.

Equations: MIT Unified Fluids 2008, lectures 7--9 (links in current decisions).
The declared section slope is an experiment input, not a reinterpretation of
the production polar. No production model, dynamics, controller or damage API
imports. Python standard library only.
"""
import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'results'
V, RHO = 70., 1.225 * math.exp(-1000 / 8500)
Q = .5 * RHO * V * V


def linear_solve(matrix, rhs):
    """Small dense collocation system; partial pivoting, no solver framework."""
    a = [list(row) + [value] for row, value in zip(matrix, rhs)]
    size = len(rhs)
    for i in range(size):
        pivot = max(range(i, size), key=lambda k: abs(a[k][i]))
        a[i], a[pivot] = a[pivot], a[i]
        if abs(a[i][i]) < 1e-12:
            raise ValueError('singular lifting-line collocation system')
        for j in range(i + 1, size):
            factor = a[j][i] / a[i][i]
            for k in range(i + 1, size + 1):
                a[j][k] -= factor * a[i][k]
    x = [0.] * size
    for i in reversed(range(size)):
        x[i] = (a[i][-1] - sum(a[i][j] * x[j]
                              for j in range(i + 1, size))) / a[i][i]
    return x


def solve(span, chord, angle, slope, modes):
    # y = center + b/2 cos(theta), Gamma = 2 b V sum A_n sin(n theta).
    # Section closure: 2 Gamma/(V c) = a0 (alpha - alpha_induced).
    theta = [math.pi * i / (modes + 1) for i in range(1, modes + 1)]
    matrix = [[math.sin(n * t) * (4 * span / (slope * chord(t))
                                 + n / math.sin(t))
               for n in range(1, modes + 1)] for t in theta]
    return linear_solve(matrix, [angle(t) for t in theta])


def local(coefficients, span, theta):
    gamma = 2 * span * V * sum(a * math.sin(n * theta)
                             for n, a in enumerate(coefficients, 1))
    induced = sum(n * a * math.sin(n * theta) / math.sin(theta)
                  for n, a in enumerate(coefficients, 1))
    return gamma, induced


def integrate(function, count=1024):
    # Independent span integrals in theta, including dy = b/2 sin(theta)dtheta.
    # Callers supply a vanishing integrand at tips; no singular tip evaluation.
    h = math.pi / count
    return h / 3 * sum((4 if i % 2 else 2) * function(i * h)
                       for i in range(1, count))


def measure(name, span, area, chord, angle, slope, modes, center=0):
    a = solve(span, chord, angle, slope, modes)
    ar = span * span / area
    cl = math.pi * ar * a[0]
    cdi = math.pi * ar * sum(n * value * value
                            for n, value in enumerate(a, 1))
    def density(t):
        gamma, induced = local(a, span, t)
        lift = RHO * V * gamma
        return lift, lift * induced, span / 2 * math.sin(t)
    lift_i = integrate(lambda t: density(t)[0] * density(t)[2])
    drag_i = integrate(lambda t: density(t)[1] * density(t)[2])
    roll_i = integrate(lambda t: -(center + span / 2 * math.cos(t))
                       * density(t)[0] * density(t)[2])
    roll = -math.pi * RHO * V * V * span**3 / 8 * (a[1] if len(a) > 1 else 0)
    roll -= Q * area * cl * center
    # Off-collocation residual distinguishes continuum approximation error
    # from merely solving the N equations exactly.
    residual = max(abs(2 * local(a, span, t)[0] / (V * chord(t))
                       - slope * (angle(t) - local(a, span, t)[1]))
                   for t in [math.pi * (i + .37) / 201 for i in range(1, 200)])
    result = {'case': name, 'modes': modes, 'span_m': span, 'area_m2': area,
              'center_y_m': center, 'AR': ar, 'section_slope_per_rad': slope,
              'CL': cl, 'CDi': cdi, 'lift_N': Q * area * cl,
              'induced_drag_N': Q * area * cdi,
              'induced_power_W': V * Q * area * cdi,
              'span_efficiency': cl * cl / (math.pi * ar * cdi),
              'roll_moment_Nm': roll, 'max_off_grid_section_cl_residual': residual,
              'lift_integral_relative_error': abs(lift_i - Q * area * cl) / (Q * area * cl),
              'drag_integral_relative_error': abs(drag_i - Q * area * cdi) / (Q * area * cdi),
              'roll_integral_error_Nm': abs(roll_i - roll),
              'coefficients': a}
    assert result['lift_integral_relative_error'] < 1e-10
    assert result['drag_integral_relative_error'] < 1e-10
    assert result['roll_integral_error_Nm'] < 1e-7
    assert 0 < result['span_efficiency'] <= 1 + 1e-12
    return result


def provenance():
    tables = []
    for name in ('validation.airframe', 'validation-bem.airframe',
                 'validation-material.airframe'):
        rows = [line.split() for line in (ROOT / 'data' / name).read_text().splitlines()]
        tables.append([[float(x) for x in row[2:]] for row in rows
                       if row[:2] == ['curve', 'wing']])
    assert tables[0] == tables[1] == tables[2]
    for deg, cl, cd, cm in tables[0]:
        expected_cl = (5.2 * math.radians(deg) if abs(deg) <= 15 else
                       math.copysign({18: .9, 25: .7, 45: .5, 90: 0., 180: 0.}[abs(deg)], deg))
        expected_cd = .016 + .07 * expected_cl**2
        if abs(deg) > 15:
            expected_cd += 1.4 * math.sin(math.radians(deg))**2
        assert abs(cl - expected_cl) <= 5.1e-9
        assert abs(cd - expected_cd) <= 5.1e-9 and cm == 0
    return {'all_three_loaded_wing_tables_match_generator_formula': True,
            'generator': 'tools/make_fixture.py',
            'meaning': 'Synthetic operational cell polar; no declared 2D/whole-wing or profile/induced split, Reynolds number or measured source. The .07 cl^2 term is not proven pure induced drag. No data is reclassified here.'}, tables[0]


def interpolated_cd(table, alpha):
    # Match the production lookup's linear-in-alpha coefficient interpolation
    # inside the low-angle range, rather than substituting the generator's
    # continuous quadratic between knots. No production imports or refactor.
    degrees = math.degrees(alpha)
    for lo, hi in zip(table, table[1:]):
        if lo[0] <= degrees <= hi[0]:
            weight = (degrees - lo[0]) / (hi[0] - lo[0])
            return lo[2] + weight * (hi[2] - lo[2])
    raise ValueError('reference experiment may not extrapolate loaded polar')


def main():
    RESULTS.mkdir(exist_ok=True)
    table_provenance, wing_polar = provenance()
    alpha, slope = math.radians(2), 2 * math.pi
    records, traces = [], []
    analytic_errors = []
    for modes in (16, 32, 64):
        for ar in (6., 10., 12.):
            span, area = 10., 100 / ar
            chord = lambda t, area=area: 4 * area / (math.pi * span) * math.sin(t)
            case = measure('elliptic', span, area, chord, lambda t: alpha, slope, modes)
            expected_cl = slope * alpha / (1 + slope / (math.pi * ar))
            err = abs(case['CL'] - expected_cl)
            assert err < 1e-12
            assert abs(case['CDi'] - expected_cl**2 / (math.pi * ar)) < 1e-12
            assert max(abs(local(case['coefficients'], span, t)[1]
                           - expected_cl / (math.pi * ar))
                       for t in (.2, .7, 1.2, 2.4, 2.9)) < 1e-12
            analytic_errors.append(err)
            records.append(case)
        # A constant-chord wing with a prescribed smooth aerodynamic twist
        # has known A1 and A2. This checks higher-mode/roll normalization too.
        a1, a2, span, chord_value = .008, .0005, 10., 1.5
        angle = lambda t: (4 * span / (slope * chord_value)
                           * (a1 * math.sin(t) + a2 * math.sin(2 * t))
                           + a1 + 2 * a2 * math.sin(2 * t) / math.sin(t))
        case = measure('manufactured_twisted_rectangle', span, span * chord_value,
                       lambda t: chord_value, angle, slope, modes)
        assert abs(case['coefficients'][0] - a1) < 1e-12
        assert abs(case['coefficients'][1] - a2) < 1e-12
        assert max(abs(a) for a in case['coefficients'][2:]) < 1e-12
        records.append(case)
    definitions = [('intact', 10., 1.5, 0.),
                   ('symmetric_tip_cut', 7.5, 1.5, 0.),
                   ('equal_area_chord_reduction_surrogate', 10., 1.125, 0.),
                   ('left_tip_cut', 8.75, 1.5, .625),
                   ('right_tip_cut', 8.75, 1.5, -.625)]
    for modes in (16, 32, 64, 96):
        for name, span, chord_value, center in definitions:
            case = measure(name, span, span * chord_value, lambda t: chord_value,
                           lambda t: alpha, slope, modes, center)
            records.append(case)
            if modes == 96:
                for i in range(1, 64):
                    t = math.pi * i / 64
                    gamma, induced = local(case['coefficients'], span, t)
                    traces.append({'case': name, 'y_m': center + span / 2 * math.cos(t),
                                   'chord_m': chord_value, 'gamma_m2_s': gamma,
                                   'downwash_m_s': V * induced,
                                   'effective_alpha_rad': alpha - induced,
                                   'lift_per_span_N_m': RHO * V * gamma})
    fine = {r['case']: r for r in records if r['modes'] == 96}
    refinement = {}
    for name in fine:
        coarse = next(r for r in records if r['modes'] == 64 and r['case'] == name)
        refinement[name] = {k: abs(fine[name][k] - coarse[k]) / abs(fine[name][k])
                            for k in ('CL', 'CDi')}
        assert max(refinement[name].values()) < 2e-6
    assert abs(fine['left_tip_cut']['roll_moment_Nm']
               + fine['right_tip_cut']['roll_moment_Nm']) < 1e-8
    intact = fine['intact']
    comparisons = {}
    for name in fine:
        case = fine[name]
        required_alpha = alpha * intact['lift_N'] / case['lift_N']
        assert required_alpha < math.radians(5)
        # Linear attached-flow rescaling, no post-stall polar used or extended.
        multiplier = required_alpha / alpha
        gamma, induced = local(case['coefficients'], case['span_m'],
                               math.acos(-2 * case['center_y_m'] / case['span_m']))
        comparisons[name] = {
            'same_alpha_lift_ratio': case['lift_N'] / intact['lift_N'],
            'centerline_load_N_m': RHO * V * gamma,
            'centerline_downwash_m_s': V * induced,
            'same_intact_lift_alpha_deg': math.degrees(required_alpha),
            'same_intact_lift_induced_drag_N': case['induced_drag_N'] * multiplier**2}
    # Explicit hypothetical reuse, not selection of a production interpretation.
    alternative = measure('hypothetical_old_5p2_as_section', 10, 15,
                          lambda t: 1.5, lambda t: alpha, 5.2, 96)
    records.append(alternative)
    # Even assuming a section interpretation, the CD partition remains an
    # independent choice. Quantify two hypothetical splits without adopting one.
    profile_if_all_old_cd = integrate(
        lambda t: interpolated_cd(wing_polar,
                                 2 * local(alternative['coefficients'], 10, t)[0]
                                 / (V * 1.5 * 5.2))
        * 1.5 * 5 * math.sin(t)) / 15
    ambiguity = {
        'old_unswept_uniform_CL_at_2deg_before_compressibility': 5.2 * alpha,
        'CL_if_5p2_is_section_slope_with_downwash': alternative['CL'],
        'CD_if_only_0p016_is_profile_plus_induced': .016 + alternative['CDi'],
        'CD_if_entire_old_CD_is_profile_plus_induced': profile_if_all_old_cd + alternative['CDi'],
        'loaded_interpolated_CD_at_2deg': interpolated_cd(wing_polar, alpha),
        'continuous_generator_CD_at_2deg': .016 + .07 * (5.2 * alpha)**2,
        'meaning': 'Hypothetical alternatives only. No evidence identifies either split or proves the old lift slope is sectional.'}
    findings = {
        'tip_cut_changes_surviving_centerline_load': comparisons['symmetric_tip_cut']['centerline_load_N_m'] < comparisons['intact']['centerline_load_N_m'],
        'equal_area_geometries_have_distinct_lift': fine['symmetric_tip_cut']['lift_N'] < fine['equal_area_chord_reduction_surrogate']['lift_N'],
        'tip_cut_costs_more_induced_drag_at_matched_lift': comparisons['symmetric_tip_cut']['same_intact_lift_induced_drag_N'] > comparisons['equal_area_chord_reduction_surrogate']['same_intact_lift_induced_drag_N'],
        'left_tip_cut_rolls_toward_lost_side_in_FRD': fine['left_tip_cut']['roll_moment_Nm'] < 0,
    }
    assert all(findings.values())
    summary = {'provenance': table_provenance, 'declared_reference': {
        'section_slope_per_rad': slope, 'alpha_deg': 2, 'speed_m_s': V,
        'profile_drag': 'Not inferred or added; only induced drag is reported.',
        'validity': 'Steady attached small-angle incompressible straight planar connected wings. No holes, fuselage/tail/wake interaction, control mechanics, structural solve, stalls/spins, transient or production integration.'},
        'elliptic_max_CL_absolute_error': max(analytic_errors),
        'refinement_64_to_96': refinement, 'same_alpha_cases': fine,
        'matched_lift_comparisons': comparisons,
        'reference_findings': findings, 'data_interpretation_ambiguity': ambiguity,
        'hypothetical_existing_slope_as_section': alternative,
        'cases': records}
    (RESULTS / 'lifting-line-summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    with (RESULTS / 'lifting-line-loads.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(traces[0]))
        writer.writeheader()
        writer.writerows(traces)
    print(json.dumps({'elliptic_max_CL_absolute_error': max(analytic_errors),
                      'refinement_64_to_96': refinement,
                      'matched_lift_comparisons': comparisons}, indent=2))


if __name__ == '__main__':
    main()
