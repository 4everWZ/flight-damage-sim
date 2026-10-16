#!/usr/bin/env python3
"""Held-rig fault transitions: physical continuity and torque quadrature.

Real C++ control states drive the existing *steady linear* sectional reference.
No body dynamics, unsteady wake or free-floating hinge law is introduced.
"""
import csv
import io
import json
import math
import subprocess
from lifting_line_reference import ROOT, RESULTS, V, RHO, solve
from local_section_coupling import metrics
from roll_authority_reference import control_angle, checked_transfer


END = .25


def healthy(t, rate, target):
    return min(rate*t, target)


def healthy_integral(t, rate, target):
    stop = target/rate
    return rate*t*t/2 if t <= stop else target*(t-stop/2)


def fault_angle(t, event, fault, rate, target):
    if fault == 'none' or t <= event:
        return healthy(t, rate, target)
    capture = healthy(event, rate, target)
    return max(capture-rate*(t-event), 0) if fault == 'disconnect' else capture


def fault_integral(event, fault, rate, target):
    if fault == 'none':
        return healthy_integral(END, rate, target)
    capture = healthy(event, rate, target)
    remainder = END-event
    after = (capture*min(remainder, capture/rate)-rate*min(remainder, capture/rate)**2/2
             if fault == 'disconnect' else capture*remainder)
    return healthy_integral(event, rate, target)+after


def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make', 'build/control-transition-probe'], cwd=ROOT, check=True)
    raw = subprocess.check_output([str(ROOT/'build/control-transition-probe'),
                                   str(ROOT/'data/validation-material.airframe')], text=True)
    (RESULTS/'control-transition-hardware.csv').write_text(raw)
    rows = [{k: v if k in ('fault', 'phase') else float(v) for k, v in row.items()}
            for row in csv.DictReader(io.StringIO(raw))]
    groups = {}
    for row in rows:
        groups.setdefault((row['hz'], row['fault'], row['event_s']), []).append(row)
    assert len(groups) == 15
    fixture = json.loads((ROOT/'data/section-wing-reference.json').read_text())
    geometry, section = fixture['geometry'], fixture['section_2d']
    span, chord, a0 = geometry['span_m'], geometry['chord_m'], section['cl_alpha_per_rad']
    scale = max(r['right_connected_target'] for r in rows)
    models, validation = {}, {}
    for damaged in (False, True):
        name = 'section_loss' if damaged else 'clean'
        slope = lambda t: a0*(1-.5*math.exp(-((span/2*math.cos(t)+3)/.6)**2)) if damaged else a0
        def coefficients(left, right):
            angle = control_angle(fixture, {'left_rad': left, 'right_rad': right})
            return solve(span, lambda t: chord, angle, slope, 128)
        base, left, right = coefficients(0, 0), coefficients(scale, 0), coefficients(0, scale)
        dl = [(x-y)/scale for x, y in zip(left, base)]
        dr = [(x-y)/scale for x, y in zip(right, base)]
        factor = -math.pi*RHO*V*V*span**3/8
        models[name] = {'base_Nm': factor*base[1], 'left_Nm_per_rad': factor*dl[1],
                        'right_Nm_per_rad': factor*dr[1]}
        # Verify the affine load response against fresh solves and the original
        # closure/transfer boundary at all four corners of the sampled angle box.
        # Effective angle and section residual are affine in this linear model;
        # convex combinations remain inside these corner bounds on the probe grid.
        max_residual = max_angle = max_transfer = max_coeff_error = 0.
        validated = set()
        def validate(l, r, boundary=False):
            nonlocal max_residual, max_angle, max_transfer, max_coeff_error
            key = (l, r)
            if key in validated:
                return
            fresh = coefficients(l, r)
            affine = [b+x*l+y*r for b, x, y in zip(base, dl, dr)]
            error = max(abs(x-y) for x, y in zip(fresh, affine))
            assert error < 1e-12
            max_coeff_error = max(max_coeff_error, error)
            if boundary:
                angle = control_angle(fixture, {'left_rad': l, 'right_rad': r})
                data = metrics(fresh, span, chord, angle, slope, section)
                max_residual = max(max_residual, data['interior_section_cl_closure_residual'])
                max_angle = max(max_angle, data['max_effective_angle_deg'])
                errors = checked_transfer(fresh, span, chord, geometry, section, slope, data)
                max_transfer = max(max_transfer, max(v for group in errors.values() for v in group.values()))
                assert data['interior_section_cl_closure_residual'] < 1e-4
            validated.add(key)
        for l, r in ((0., 0.), (scale, 0.), (0., scale), (scale, scale)):
            validate(l, r, True)
        for (hz, fault, event), trace in groups.items():
            for row in trace:
                if (row['phase'] == 'post_event' or row['time_s'] == END
                        or (row['phase'] == 'sample' and abs(row['time_s']-event-1/hz) < 1e-12)):
                    validate(row['left_rad'], row['right_rad'])
        validation[name] = {'max_affine_coefficient_error': max_coeff_error,
                            'fresh_solve_states': len(validated),
                            'corner_interior_cl_residual_bound': max_residual,
                            'corner_effective_angle_bound_deg': max_angle,
                            'corner_transfer_error_max': max_transfer}
    measurements, mapped = {}, []
    endpoint_error = common_error = shared_torque_error = 0.
    common, shared_torque = {}, {}
    for (hz, fault, event), trace in groups.items():
        first = trace[0]
        rl, rr = first['left_rate_bound'], first['right_rate_bound']
        tl, tr = first['left_connected_target'], first['right_connected_target']
        physical = [r for r in trace if r['phase'] == 'sample']
        assert len(physical) == round(END*hz)+1
        for row in trace:
            t = row['time_s']
            error = max(abs(row['left_rad']-fault_angle(t, event, fault, rl, tl)),
                        abs(row['right_rad']-healthy(t, rr, tr)))
            endpoint_error = max(endpoint_error, error)
            assert error < 1e-12 and row['body_event_equal'] == 1
            assert 0 <= row['left_rad'] <= scale+1e-12 and 0 <= row['right_rad'] <= scale+1e-12
            if row['phase'] == 'sample' and abs(t*120-round(t*120)) < 1e-12:
                key = (fault, event, round(t*120))
                if key in common:
                    common_error = max(common_error, abs(row['left_rad']-common[key][0]),
                                       abs(row['right_rad']-common[key][1]))
                else:
                    common[key] = (row['left_rad'], row['right_rad'])
        pre = next((r for r in trace if r['phase'] == 'pre_event'), None)
        post = next((r for r in trace if r['phase'] == 'post_event'), None)
        if pre:
            assert pre['left_rad'] == post['left_rad'] and pre['right_rad'] == post['right_rad']
        case = f'{fault}:{event:g}:{hz:g}'
        measurements[case] = {}
        for name, model in models.items():
            def torque(l, r):
                return model['base_Nm']+model['left_Nm_per_rad']*l+model['right_Nm_per_rad']*r
            exact = (model['base_Nm']*END
                     + model['left_Nm_per_rad']*fault_integral(event, fault, rl, tl)
                     + model['right_Nm_per_rad']*healthy_integral(END, rr, tr))
            end_sum = sum(torque(r['left_rad'], r['right_rad'])/hz for r in physical[1:])
            mid_sum = sum(torque(r['left_mid_rad'], r['right_mid_rad'])/hz for r in physical[1:])
            # At most two left and one right within-step slope corners; event
            # times are common grid boundaries. Midpoint error of one linear
            # slope corner is bounded by |slope_jump| h^2/8.
            bound = (2*abs(model['left_Nm_per_rad'])*rl+abs(model['right_Nm_per_rad'])*rr)/(8*hz*hz)
            assert abs(mid_sum-exact) <= bound+1e-8
            event_jump = 0 if pre is None else abs(torque(pre['left_rad'], pre['right_rad'])
                                                  - torque(post['left_rad'], post['right_rad']))
            assert event_jump == 0
            first_after = next((r for r in physical if event >= 0 and r['time_s'] > event+1e-12), None)
            measurements[case][name] = {
                'event_torque_jump_Nm': event_jump,
                'first_step_torque_change_Nm': None if first_after is None else torque(first_after['left_rad'], first_after['right_rad'])-torque(post['left_rad'], post['right_rad']),
                'exact_steady_reference_torque_integral_Nms': exact,
                'endpoint_integral_error_Nms': end_sum-exact,
                'midpoint_integral_error_Nms': mid_sum-exact,
                'midpoint_piecewise_linear_error_bound_Nms': bound,
                'captured_angle_deg': None if pre is None else math.degrees(pre['left_rad']),
                'first_step_left_angle_change_deg': None if first_after is None else math.degrees(first_after['left_rad']-post['left_rad']),
                'first_step_filtered_driver_change_deg': None if first_after is None else math.degrees(first_after['left_filtered_rad']-post['left_filtered_rad']),
                'final_roll_Nm': torque(physical[-1]['left_rad'], physical[-1]['right_rad'])}
            for row in trace:
                if row['phase'] == 'sample' and abs(row['time_s']*120-round(row['time_s']*120)) < 1e-12:
                    key = (name, fault, event, round(row['time_s']*120))
                    value = torque(row['left_rad'], row['right_rad'])
                    if key in shared_torque:
                        shared_torque_error = max(shared_torque_error, abs(value-shared_torque[key]))
                    else:
                        shared_torque[key] = value
                mapped.append({'case': case, 'aero': name, 'time_s': row['time_s'], 'phase': row['phase'],
                               'roll_Nm': torque(row['left_rad'], row['right_rad'])})
    assert common_error < 1e-12 and shared_torque_error < 1e-9
    summary = {'events_s': [1/60, .125], 'rates_Hz': [120, 240, 480],
               'held_input': .2, 'duration_s': END, 'measurements': measurements,
               'max_endpoint_angle_vs_continuous_law_error_rad': endpoint_error,
               'max_shared_time_angle_difference_rad': common_error,
               'max_shared_time_roll_torque_difference_Nm': shared_torque_error,
               'affine_reference_validation': validation,
               'scope': 'Real actuator trace on held body; fixed q and zero-lag/friction fixture. Event angle/body snapshots are continuous; raw filtered target may reset on the following update and is not physical angle. Quasi-steady sectional torque is affine and checked with fresh solves at corners/event/first-step/end states. Torque time integral is not body angular momentum or flight recovery. Jam reaction inertia/energy and unsteady wakes are not modeled. Midpoint error bound concerns this piecewise-linear actuator benchmark only; original 10% failure and production defaults remain unchanged.'}
    (RESULTS/'control-transition-summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    with (RESULTS/'control-transition-forces.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(mapped[0]))
        writer.writeheader()
        writer.writerows(mapped)
    print(json.dumps({'angle_endpoint_error_rad': endpoint_error, 'shared_time_error_rad': common_error,
                      'reference_validation': validation,
                      'clean_120_examples': {k: v['clean'] for k, v in measurements.items() if k.endswith(':120')}}, indent=2))


if __name__ == '__main__':
    main()
