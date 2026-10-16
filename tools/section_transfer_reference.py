#!/usr/bin/env python3
"""Isolated sectional fixture and conservative strip-to-cell boundary experiment.

No production imports or callbacks. Span-preserving chord reduction is a
geometric surrogate, never a skin-perforation interpretation.
"""
import csv
import json
import math
from lifting_line_reference import ROOT, RESULTS, V, RHO, Q, local, measure


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def scale(a, s):
    return tuple(x * s for x in a)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0])


def norm(a):
    return math.sqrt(dot(a, a))


def gauss(count):
    """Legendre nodes/weights on [-1,1], generated rather than a dependency."""
    result = []
    for i in range(1, count + 1):
        x = math.cos(math.pi * (i - .25) / (count + .5))
        for _ in range(20):
            p, previous = 1., 0.
            for n in range(1, count + 1):
                p, previous = ((2*n - 1)*x*p - (n - 1)*previous)/n, p
            derivative = count * (x*p - previous) / (x*x - 1)
            step = p / derivative
            x -= step
            if abs(step) < 1e-15:
                break
        result.append((x, 2 / ((1 - x*x) * derivative**2)))
    assert abs(sum(w for _, w in result) - 2) < 1e-12
    return result


def strips(reference, chord, leading_x, ac_fraction, section, count, quadrature):
    span, center, coefficients = (reference[k] for k in
                                  ('span_m', 'center_y_m', 'coefficients'))
    nodes, result = gauss(quadrature), []
    for i in range(count):
        lo = center - span/2 + span*i/count
        hi = center - span/2 + span*(i + 1)/count
        ta = math.acos(max(-1., min(1., 2*(hi-center)/span)))
        tb = math.acos(max(-1., min(1., 2*(lo-center)/span)))
        force, moment = (0.,)*3, (0.,)*3
        induced_drag = profile_drag = 0.
        for node, weight in nodes:
            theta = (ta + tb)/2 + (tb-ta)/2*node
            dy = weight*(tb-ta)/2 * span/2*math.sin(theta)
            y = center + span/2*math.cos(theta)
            gamma, induced = local(coefficients, span, theta)
            cl = 2*gamma/(V*chord)
            alpha_section = cl/section['cl_alpha_per_rad'] + math.radians(section['zero_lift_alpha_deg'])
            assert math.radians(section['valid_alpha_deg'][0]) <= alpha_section <= math.radians(section['valid_alpha_deg'][1])
            lift = RHO*V*gamma*dy
            di = lift*induced
            dp = Q*chord*(section['profile_cd0'] + section['profile_cd_cl_squared']*cl*cl)*dy
            f = (-di-dp, 0., -lift)
            r = (leading_x - ac_fraction*chord, y, 0.)
            force = add(force, f)
            # Zero cm is explicit fixture data, not an inferred pressure law.
            couple = (0., Q*chord*chord*section['section_cm_about_aerodynamic_center']*dy, 0.)
            moment = add(moment, add(cross(r, f), couple))
            induced_drag += di
            profile_drag += dp
        result.append({'index': i, 'y_m': (lo+hi)/2, 'force': force,
                       'moment_about_datum': moment, 'induced_drag_N': induced_drag,
                       'profile_drag_N': profile_drag, 'area_m2': chord*(hi-lo)})
    return result


def transfer(strip, chord, leading_x, chord_cells, preserve_couple):
    cells = []
    for j in range(chord_cells):
        r = (leading_x - chord*(j+.5)/chord_cells, strip['y_m'], 0.)
        weight = 1/chord_cells
        force = scale(strip['force'], weight)
        couple = (scale(add(strip['moment_about_datum'], scale(cross(r, strip['force']), -1)), weight)
                  if preserve_couple else (0.,)*3)
        cells.append({'position': r, 'force': force, 'couple': couple})
    return cells


def totals(cells, cg, velocity, omega):
    force, moment, power = (0.,)*3, (0.,)*3, 0.
    for cell in cells:
        r = add(cell['position'], scale(cg, -1))
        force = add(force, cell['force'])
        moment = add(moment, add(cross(r, cell['force']), cell['couple']))
        power += dot(cell['force'], add(velocity, cross(omega, r))) + dot(cell['couple'], omega)
    return force, moment, power


def main():
    fixture = json.loads((ROOT/'data/section-wing-reference.json').read_text())
    geo, section = fixture['geometry'], fixture['section_2d']
    RESULTS.mkdir(exist_ok=True)
    alpha = math.radians(2 - section['zero_lift_alpha_deg'])
    cases, raw = {}, []
    # All positions are in the declared flow frame, not a live aircraft pose.
    cg, velocity, omega = (.3, -.2, .1), (70., 2., -3.), (.13, -.2, .07)
    for name, span_scale, chord_scale, center_fraction in (
            ('intact', 1., 1., 0.), ('symmetric_tip_cut', .75, 1., 0.),
            ('span_preserving_chord_reduction_surrogate', 1., .75, 0.),
            ('left_tip_cut', .875, 1., .0625),
            ('right_tip_cut', .875, 1., -.0625)):
        span, chord = geo['span_m']*span_scale, geo['chord_m']*chord_scale
        center = geo['span_m']*center_fraction
        ref = measure(name, span, span*chord, lambda t: chord, lambda t: alpha,
                      section['cl_alpha_per_rad'], 96, center)
        count = round(geo['span_strips']*span_scale)
        sampled = strips(ref, chord, geo['leading_edge_x_m'],
                         geo['aerodynamic_center_chord_fraction'], section, count, 32)
        coarse = strips(ref, chord, geo['leading_edge_x_m'],
                        geo['aerodynamic_center_chord_fraction'], section, count, 16)
        expected_force, expected_moment = (0.,)*3, (0.,)*3
        for strip in sampled:
            expected_force = add(expected_force, strip['force'])
            expected_moment = add(expected_moment, strip['moment_about_datum'])
        moment_cg = add(expected_moment, scale(cross(cg, expected_force), -1))
        expected_power = dot(expected_force, velocity) + dot(moment_cg, omega)
        induced = sum(s['induced_drag_N'] for s in sampled)
        profile = sum(s['profile_drag_N'] for s in sampled)
        quadrature_difference = max(abs(sum(s['force'][k] for s in sampled)
                                        - sum(s['force'][k] for s in coarse))
                                    for k in range(3))
        assert abs(-expected_force[2]/ref['lift_N'] - 1) < 1e-9
        assert abs(induced/ref['induced_drag_N'] - 1) < 1e-9
        assert quadrature_difference/max(norm(expected_force), 1) < 1e-8
        errors = {}
        for subdivisions in (geo['chord_cells'], 2*geo['chord_cells']):
            cells = [cell for strip in sampled for cell in transfer(
                strip, chord, geo['leading_edge_x_m'], subdivisions, True)]
            f, m, power = totals(cells, cg, velocity, omega)
            errors[subdivisions] = {'force_N': norm(add(f, scale(expected_force, -1))),
                                    'moment_Nm': norm(add(m, scale(moment_cg, -1))),
                                    'rigid_velocity_power_W': abs(power-expected_power)}
            assert max(errors[subdivisions].values()) < 1e-8
        naive = [cell for strip in sampled for cell in transfer(
            strip, chord, geo['leading_edge_x_m'], geo['chord_cells'], False)]
        _, naive_moment, naive_power = totals(naive, cg, velocity, omega)
        # This intentionally fails: conserving only force can change pitch
        # torque and work when moving the force away from its aerodynamic center.
        naive_pitch_error = naive_moment[1] - moment_cg[1]
        assert abs(naive_pitch_error) > 100
        assert abs(naive_power-expected_power) > 10
        cases[name] = {'span_m': span, 'chord_m': chord, 'strips': count,
                       'force_N': expected_force, 'moment_about_cg_Nm': moment_cg,
                       'lift_N': -expected_force[2], 'induced_drag_N': induced,
                       'profile_drag_N': profile, 'total_drag_N': induced+profile,
                       'quadrature_16_to_32_force_max_difference_N': quadrature_difference,
                       'equivalent_transfer_errors': errors,
                       'area_only_transfer_pitch_error_Nm': naive_pitch_error,
                       'area_only_transfer_power_error_W': naive_power-expected_power}
        for strip in sampled:
            raw.append({'case': name, 'strip': strip['index'], 'y_m': strip['y_m'],
                        'lift_N': -strip['force'][2], 'induced_drag_N': strip['induced_drag_N'],
                        'profile_drag_N': strip['profile_drag_N']})
    summary = {'fixture': fixture,
               'test_state': {'aerodynamic_speed_m_s': V, 'density_kg_m3': RHO,
                              'geometric_incidence_deg': 2., 'cg_m': cg,
                              'rigid_power_check_velocity_m_s': velocity,
                              'rigid_power_check_rates_rad_s': omega,
                              'meaning': 'The arbitrary rigid velocity field tests equivalent-wrench power; it is not fed back to the static aerodynamic solve.'},
               'cases': cases,
               'tested_boundary': 'Equivalent total force, moment and rigid-velocity power only. Area weights plus explicit couples preserve the chosen aerodynamic-center wrench. Chordwise pressure, local stress, real actuator/structure coupling and live dynamics are not inferred or tested. A chord-reduction surrogate is not skin perforation. Production/old polars remain untouched.'}
    (RESULTS/'section-transfer-summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    with (RESULTS/'section-transfer-strips.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(raw[0]))
        writer.writeheader()
        writer.writerows(raw)
    print(json.dumps(cases, indent=2))


if __name__ == '__main__':
    main()
