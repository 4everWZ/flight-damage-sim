#!/usr/bin/env python3
"""Generate explicit synthetic fixture data, not an alternate simulation."""
from pathlib import Path
import math
root = Path(__file__).resolve().parents[1]
rows = ['# Synthetic validation airframe; SI, body x forward y right z down.',
        '# Not measured data for a historical aircraft. See current-decisions.md.',
        '# mass node kg datum_x datum_y datum_z Ixx Iyy Izz (kg m^2)',
        '# cell id node chain plane x y z area chord span incidence_deg control_sign control_gain table',
        '# edge name node_a node_b EI_over_L GJ_over_L Z_bend Z_torsion yield_Pa ultimate_Pa ductility plastic_rate_per_s',
        '# chain name axis(roll=0,pitch=1,yaw=2) limit_deg slew_deg_s pressure_effort_Pa',
        '# curve name alpha_deg CL CD CM; engine W rated_rpm radius_m rotor_I CT CQ advance_slope altitude_scale_m lag_s',
        'node fuselage', 'node left_inner', 'node left_outer', 'node right_inner', 'node right_outer', 'node tail',
        'mass 0 2100 0 0 0 900 2300 2700',
        'mass 1 160 0.15 -1.5 0 50 40 70', 'mass 2 130 0.15 -4 0 60 25 65',
        'mass 3 160 0.15 1.5 0 50 40 70', 'mass 4 130 0.15 4 0 60 25 65',
        'mass 5 120 -4.1 0 0 20 45 45',
        'engine 850000 2600 1.5 30 .085 .007 1.1 12000 .35',
        'prop_position 2 0 0', 'body_drag .42', 'handoff .25', 'trim 1 1 1',
        'chain left_aileron 0 22 100 18000', 'chain right_aileron 0 22 100 18000',
        'chain elevator 1 25 95 16000', 'chain rudder 2 25 100 16000']
for a, b, prefix, scale in [(0, 1, 'left_root', 1), (1, 2, 'left_outer', .65),
                          (0, 3, 'right_root', 1), (3, 4, 'right_outer', .65), (0, 5, 'tail', .5)]:
    for name, share in [('front', .55), ('rear', .30), ('skin', .15)]:
        # EI/L, GJ/L, section moduli; synthetic yield/ultimate stresses and ductility.
        rows.append(f'edge {prefix}_{name} {a} {b} {2e7*share*scale:g} {3e7*share*scale:g} {0.00042*share*scale:g} {0.00050*share*scale:g} 250000000 330000000 .035 .025')
# Aero tables include post-stall drop, drag rise, and recovery; static coefficient tables.
for name, slope, cd0 in [('wing', 5.2, .016), ('tail', 4.7, .014), ('fin', 4.2, .018)]:
    for deg in [-180, -90, -45, -25, -18, -15, -10, -5, 0, 5, 10, 15, 18, 25, 45, 90, 180]:
        a = math.radians(deg)
        if abs(deg) <= 15:
            cl = slope*a
        else:
            cl = math.copysign({18: .9, 25: .7, 45: .5, 90: 0., 180: 0.}[abs(deg)], deg)
        cd = cd0 + .07*cl*cl + (1.4*math.sin(a)**2 if abs(deg)>15 else 0)
        rows.append(f'curve {name} {deg} {cl:.8f} {cd:.8f} 0')
id_ = 0
# 16 span stations x 4 chord stations x 2 wings = 128 wing cells.
for side in [-1, 1]:
    for i in range(16):
        y = side*(i+.5)*5/16
        node = (1 if side < 0 else 3) + (i >= 8)
        for j in range(4):
            x = .55 - (j+.5)*1.5/4
            chain = (0 if side < 0 else 1) if i >= 8 and j >= 2 else -1
            # Controls affect only their actual trailing-edge cells.
            rows.append(f'cell {id_} {node} {chain} 0 {x:g} {y:g} 0 {1.5*5/16/4:g} 1.5 .3125 2 {-side if chain>=0 else 0} .8 wing')
            id_ += 1
for i in range(8):
    for j in range(2):
        y = -1.6+(i+.5)*.4
        rows.append(f'cell {id_} 5 {2 if j==1 else -1} 0 {-4-(j+.5)*.8/2:g} {y:g} 0 .16 .8 .4 -1 -1 .8 tail')
        id_ += 1
for i in range(8):
    rows.append(f'cell {id_} 5 3 1 -4.3 0 {-0.2-(i+.5)*.15:g} .15 .8 .15 0 1 .8 fin')
    id_ += 1
(root/'data'/'validation.airframe').write_text('\n'.join(rows)+'\n')
print(f'wrote {id_} cells')
