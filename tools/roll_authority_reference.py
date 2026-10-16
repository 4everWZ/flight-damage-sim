#!/usr/bin/env python3
"""Static roll moment/authority from actual actuators and explicit section data.

No live flight-model adapter. Aerodynamic damage is the previous synthetic
section-slope intervention; mechanical chains remain identical between arms.
"""
import csv
import io
import json
import math
import subprocess
from lifting_line_reference import ROOT, RESULTS, Q, solve, local
from local_section_coupling import metrics, projected_frozen
from section_transfer_reference import strips, transfer, totals, add, scale, cross, dot, norm


def actuator_rows(commands=None):
    args = [str(ROOT/'build/section-controls-probe'),
            str(ROOT/'data/validation-material.airframe')]
    if commands is not None:
        args += [str(x) for x in commands]
    raw = subprocess.check_output(args, text=True)
    return [{k: float(v) for k, v in row.items()}
            for row in csv.DictReader(io.StringIO(raw))]


def checked_transfer(a, span, chord, geometry, section, slope, reference_metrics):
    reference = {'coefficients': a, 'span_m': span, 'center_y_m': 0.}
    sampled = strips(reference, chord, geometry['leading_edge_x_m'],
                     geometry['aerodynamic_center_chord_fraction'], section,
                     geometry['span_strips'], 32, slope)
    force, moment = (0.,)*3, (0.,)*3
    for strip in sampled:
        force = add(force, strip['force'])
        moment = add(moment, strip['moment_about_datum'])
    cg, velocity, omega = (.3, -.2, .1), (70., 2., -3.), (.13, -.2, .07)
    moment_cg = add(moment, scale(cross(cg, force), -1))
    power = dot(force, velocity) + dot(moment_cg, omega)
    errors = {'strip_integrals': {
        'lift_N': abs(-force[2]-reference_metrics['lift_N']),
        'roll_Nm': abs(moment[0]-reference_metrics['roll_Nm']),
        'induced_drag_N': abs(sum(s['induced_drag_N'] for s in sampled)
                              - reference_metrics['induced_drag_N'])}}
    assert max(errors['strip_integrals'].values()) < 1e-7
    for count in (geometry['chord_cells'], 2*geometry['chord_cells']):
        cells = [cell for strip in sampled for cell in transfer(
            strip, chord, geometry['leading_edge_x_m'], count, True)]
        f, m, p = totals(cells, cg, velocity, omega)
        errors[count] = {'force_N': norm(add(f, scale(force, -1))),
                         'moment_Nm': norm(add(m, scale(moment_cg, -1))),
                         'rigid_power_W': abs(p-power)}
        assert max(errors[count].values()) < 1e-8
    return errors


def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make', 'build/section-controls-probe'], cwd=ROOT, check=True)
    fixture = json.loads((ROOT/'data/section-wing-reference.json').read_text())
    geometry, section, control = (fixture[k] for k in
                                  ('geometry', 'section_2d', 'experimental_roll_control'))
    span, chord, a0 = geometry['span_m'], geometry['chord_m'], section['cl_alpha_per_rad']
    actuators = actuator_rows(control['tested_normalized_inputs'])
    assert [r['input'] for r in actuators] == control['tested_normalized_inputs']
    assert all(r['roll_ownership'] == 1 for r in actuators)
    assert all(abs(r['q_Pa']/Q-1) < 1e-12 for r in actuators)
    def fraction(t):
        y = span/2*math.cos(t)
        return 1-.5*math.exp(-((y+3)/.6)**2)
    def angle_for(row):
        def angle(t):
            y = span/2*math.cos(t)
            left = math.exp(-((y-control['left_center_y_m'])/control['gaussian_width_m'])**2)
            right = math.exp(-((y-control['right_center_y_m'])/control['gaussian_width_m'])**2)
            return (math.radians(2-section['zero_lift_alpha_deg'])
                    + control['incidence_per_surface_rad']
                    * (row['left_rad']*left-row['right_rad']*right))
        return angle
    def run(row, modes, include_frozen=False):
        angle = angle_for(row)
        intact = solve(span, lambda t: chord, angle, a0, modes)
        damaged = solve(span, lambda t: chord, angle, lambda t: a0*fraction(t), modes)
        coefficients = {'intact': intact, 'damaged_recoupled': damaged}
        if include_frozen:
            coefficients['damaged_frozen_ablation'] = projected_frozen(intact, span, fraction, modes)
        result = {}
        for name, a in coefficients.items():
            slope = (lambda t: a0) if name == 'intact' else lambda t: a0*fraction(t)
            data = metrics(a, span, chord, angle, slope, section)
            if name != 'damaged_frozen_ablation':
                assert data['interior_section_cl_closure_residual'] < 1e-4
                data['transfer_errors'] = checked_transfer(a, span, chord, geometry, section, slope, data)
            else:
                assert data['interior_section_cl_closure_residual'] > .01
            result[name] = data
        return result
    cases = {str(row['input']): run(row, 128, row['input'] == 0) for row in actuators}
    neutral = cases['0.0']
    low, high = min(control['tested_normalized_inputs']), max(control['tested_normalized_inputs'])
    authority = {}
    for name in ('intact', 'damaged_recoupled'):
        before, after = cases[str(low)][name]['roll_Nm'], cases[str(high)][name]['roll_Nm']
        gain = (after-before)/(high-low)
        bias = neutral[name]['roll_Nm']
        assert abs((before+after)/2-bias) < 1e-7
        authority[name] = {'neutral_roll_Nm': bias, 'roll_gain_Nm_per_input': gain,
                           'negative_tested_roll_Nm': before, 'positive_tested_roll_Nm': after,
                           'positive_increment_from_neutral_Nm': after-bias,
                           'estimated_zero_roll_input': -bias/gain,
                           'zero_roll_inside_tested_interval': low <= -bias/gain <= high}
    # A finer solve must not merely change the apparent damage/control result.
    coarse = run(actuators[-1], 96, False)
    refinement = {name: abs(coarse[name]['roll_Nm']-cases[str(high)][name]['roll_Nm'])
                   / max(abs(cases[str(high)][name]['roll_Nm']), 1.) for name in coarse}
    assert max(refinement.values()) < 1e-5
    target = authority['damaged_recoupled']['estimated_zero_roll_input']
    cancellation = None
    if low <= target <= high:
        row = actuator_rows([target])[0]
        cancellation = {'input': target, 'actuators': row,
                        'measured': run(row, 128, False)['damaged_recoupled']}
        assert abs(cancellation['measured']['roll_Nm']) < 1e-6
    retained = authority['damaged_recoupled']['roll_gain_Nm_per_input']/authority['intact']['roll_gain_Nm_per_input']
    # Preserve the first exploratory failure: the frozen ablation at +0.2
    # exceeds the declared section domain. Do not relax the physical-arm check
    # or manufacture an apparent frozen control-authority curve from it.
    positive_angle = angle_for(actuators[-1])
    positive_intact = solve(span, lambda t: chord, positive_angle, a0, 128)
    frozen_positive = projected_frozen(positive_intact, span, fraction, 128)
    frozen_max_angle = math.degrees(max(abs(positive_angle(t)-local(frozen_positive, span, t)[1])
                                       for t in [math.pi*(i+.31)/513 for i in range(1, 512)]))
    assert frozen_max_angle > section['valid_alpha_deg'][1]
    neutral_check = {
        'frozen_vs_recoupled_lift_difference_N': neutral['damaged_recoupled']['lift_N']-neutral['damaged_frozen_ablation']['lift_N'],
        'frozen_vs_recoupled_roll_difference_Nm': neutral['damaged_recoupled']['roll_Nm']-neutral['damaged_frozen_ablation']['roll_Nm']}
    assert abs(neutral_check['frozen_vs_recoupled_lift_difference_N']) < 20
    assert abs(neutral_check['frozen_vs_recoupled_roll_difference_Nm']) > 150
    summary = {'actuator_bench': actuators, 'declared_control_mapping': control,
               'cases': cases, 'authority': authority, 'retained_incremental_gain_fraction': retained,
               'neutral_reference_recheck': neutral_check,
               'rejected_frozen_control_arm': {'input': high, 'maximum_effective_angle_deg': frozen_max_angle,
                                               'reason': 'Outside the unchanged +/-5 degree section domain; excluded from control-gain estimation. Only neutral frozen ablation is retained for the prior mechanism check.'},
               'relative_roll_refinement_96_to_128_at_positive_input': refinement,
               'static_roll_bias_cancellation': cancellation,
               'scope': 'Same real C++ direct-axis arbitration/actuator outputs feed each independent aerodynamic arm. Rigid hardware bench: 120Hz controls-only, body held, settled 1-to-2s comparison. Existing airframe aerodynamic tables are not used. Synthetic incidence mapping, local slope loss only, no mechanical damage in this comparison. Reported gain and cancellation apply only within the tested +/-0.2 input and attached linear domain, not full stick limits, achieved roll rate, 6DoF trim/handling, stall, structural stress or flight safety. Frozen arm is an inconsistent ablation, not a physical model. Wrench power is a rigid-field identity, not a dynamic energy ledger.'}
    (RESULTS/'roll-authority-summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    with (RESULTS/'roll-authority.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['input', 'arm', 'lift_N', 'roll_Nm',
                                                   'induced_drag_N', 'interior_cl_residual'])
        writer.writeheader()
        for u, arms in cases.items():
            for name, data in arms.items():
                writer.writerow({'input': u, 'arm': name, 'lift_N': data['lift_N'],
                                 'roll_Nm': data['roll_Nm'], 'induced_drag_N': data['induced_drag_N'],
                                 'interior_cl_residual': data['interior_section_cl_closure_residual']})
    print(json.dumps({'authority': authority, 'retained_incremental_gain_fraction': retained,
                      'static_roll_bias_cancellation_input': target,
                      'relative_refinement': refinement}, indent=2))


if __name__ == '__main__':
    main()
