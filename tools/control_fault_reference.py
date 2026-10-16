#!/usr/bin/env python3
"""One static 3-hardware x 2-aero fault comparison, no production semantics change."""
import csv
import json
import math
import subprocess
from lifting_line_reference import ROOT, RESULTS, Q, solve
from local_section_coupling import metrics
from roll_authority_reference import actuator_rows, control_angle, checked_transfer


def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make', 'build/section-controls-probe'], cwd=ROOT, check=True)
    fixture = json.loads((ROOT/'data/section-wing-reference.json').read_text())
    geometry, section, control = (fixture[k] for k in
                                  ('geometry', 'section_2d', 'experimental_roll_control'))
    span, chord, a0 = geometry['span_m'], geometry['chord_m'], section['cl_alpha_per_rad']
    commands = control['tested_normalized_inputs']
    low, high = min(commands), max(commands)
    def evaluate(row, damaged, modes=128):
        angle = control_angle(fixture, row)
        def slope(t):
            y = span/2*math.cos(t)
            return a0*(1-.5*math.exp(-((y+3)/.6)**2)) if damaged else a0
        a = solve(span, lambda t: chord, angle, slope, modes)
        data = metrics(a, span, chord, angle, slope, section)
        assert data['interior_section_cl_closure_residual'] < 1e-4
        data['transfer_errors'] = checked_transfer(a, span, chord, geometry, section, slope, data)
        return data
    hardware = {}
    cases, curves, refinement = {}, [], {}
    for fault in (None, 'left_disconnected', 'left_jammed'):
        name = fault or 'intact_hardware'
        rows = actuator_rows(commands, fault)
        hardware[name] = rows
        assert all(r['roll_ownership'] == 1 and abs(r['q_Pa']/Q-1) < 1e-12 for r in rows)
        assert [r['right_rad'] for r in rows] == [r['right_rad'] for r in hardware['intact_hardware']]
        if fault:
            assert all(r['pre_failure_left_rad'] > 0 and
                       r['immediate_left_rad'] == r['pre_failure_left_rad'] for r in rows)
        if fault == 'left_disconnected':
            assert all(r['left_connected'] == 0 and r['left_jammed'] == 0
                       and r['left_rad'] == 0 for r in rows)
        if fault == 'left_jammed':
            assert all(r['left_connected'] == 1 and r['left_jammed'] == 1
                       and r['left_rad'] == r['pre_failure_left_rad']
                       and r['jam_position_rad'] == r['left_rad'] for r in rows)
        for damaged in (False, True):
            key = name + ('+section_loss' if damaged else '+clean_aero')
            samples = {str(r['input']): evaluate(r, damaged) for r in rows}
            bias = samples['0.0']['roll_Nm']
            gain = (samples[str(high)]['roll_Nm']-samples[str(low)]['roll_Nm'])/(high-low)
            assert gain > 0
            assert abs((samples[str(high)]['roll_Nm']+samples[str(low)]['roll_Nm'])/2-bias) < 1e-7
            root = -bias/gain
            # Roundoff-only tolerance for an exactly-on-boundary cancellation.
            classification = ('boundary' if min(abs(root-low), abs(root-high)) < 1e-12
                              else 'interior' if low < root < high else 'outside_tested_interval')
            cancellation = None
            if classification != 'outside_tested_interval':
                command = min(high, max(low, root))
                actual = actuator_rows([command], fault)[0]
                cancellation = {'command': command, 'actuator': actual,
                                'measured': evaluate(actual, damaged)}
                assert abs(cancellation['measured']['roll_Nm']) < 1e-6
            cases[key] = {'neutral_bias_Nm': bias, 'incremental_gain_Nm_per_input': gain,
                          'sampled_negative_roll_Nm': samples[str(low)]['roll_Nm'],
                          'sampled_positive_roll_Nm': samples[str(high)]['roll_Nm'],
                          'estimated_zero_roll_input': root,
                          'cancellation_classification': classification,
                          'tested_input_headroom_from_zero_roll': {
                              'negative': root-low, 'positive': high-root}
                          if classification != 'outside_tested_interval' else None,
                          'cancellation': cancellation, 'samples': samples}
            coarse = evaluate(rows[-1], damaged, 96)
            refinement[key] = abs(coarse['roll_Nm']-samples[str(high)]['roll_Nm'])/max(abs(samples[str(high)]['roll_Nm']), 1)
            assert refinement[key] < 1e-5
            for u, data in samples.items():
                curves.append({'case': key, 'input': u, 'lift_N': data['lift_N'],
                               'roll_Nm': data['roll_Nm'], 'induced_drag_N': data['induced_drag_N'],
                               'max_effective_angle_deg': data['max_effective_angle_deg'],
                               'interior_cl_residual': data['interior_section_cl_closure_residual']})
    baseline = cases['intact_hardware+clean_aero']['incremental_gain_Nm_per_input']
    for key, data in cases.items():
        data['gain_fraction_of_clean_intact'] = data['incremental_gain_Nm_per_input']/baseline
    equal_gain = {}
    for suffix in ('+clean_aero', '+section_loss'):
        cut = cases['left_disconnected'+suffix]['incremental_gain_Nm_per_input']
        jam = cases['left_jammed'+suffix]['incremental_gain_Nm_per_input']
        equal_gain[suffix] = abs(cut-jam)
        assert abs(cut-jam) < 1e-7
    decomposition = {}
    for suffix in ('+clean_aero', '+section_loss'):
        total = cases['intact_hardware'+suffix]['incremental_gain_Nm_per_input']
        right = cases['left_disconnected'+suffix]['incremental_gain_Nm_per_input']
        decomposition[suffix] = {'both_actuators_Nm_per_input': total,
                                 'right_only_Nm_per_input': right,
                                 'inferred_left_increment_Nm_per_input': total-right,
                                 'meaning': 'Linear static superposition in this declared reference, not a general damage-percentage law.'}
    summary = {'hardware_bench': hardware, 'cases': cases,
               'control_sensitivity_decomposition': decomposition,
               'equal_disconnect_jam_gain_error_Nm_per_input': equal_gain,
               'relative_roll_refinement_96_to_128': refinement,
               'fault_semantics': 'Existing Simulation::disconnect only clears connected; immediate angle is retained and advance_surface targets zero afterward. Jam sets existing jammed/jam_position to the actual pre-fault angle, and the authoritative update freezes it regardless of subsequent input. Both fault arms preload +0.2 for 1s, then apply the same tested inputs for 1s/2s settling. No direct deflection assignment or production change.',
               'scope': 'Static held-body, same declared synthetic sectional/control mapping and +/-0.2 inputs. Clean/local slope loss crossed with normal/left disconnected/left jammed hardware. Headroom is within this tested interval, not mechanical stops or full-stick authority. Roll cancellation is not six-axis trim, dynamic recovery, pressure/stress validation, hinge free-floating physics or an energy transient. Original domain and acceptance thresholds are unchanged.'}
    (RESULTS/'control-fault-summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    with (RESULTS/'control-fault-curves.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(curves[0]))
        writer.writeheader()
        writer.writerows(curves)
    print(json.dumps({key: {k: v[k] for k in ('neutral_bias_Nm', 'incremental_gain_Nm_per_input',
                     'gain_fraction_of_clean_intact', 'estimated_zero_roll_input',
                     'cancellation_classification', 'tested_input_headroom_from_zero_roll')}
                      for key, v in cases.items()}, indent=2))


if __name__ == '__main__':
    main()
