#!/usr/bin/env python3
"""Explicit material inventory and conservative mass transfer, authoritative C++ only."""
import json
import subprocess
from experiments import ROOT, RESULTS


def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make', 'build/material-tests'], cwd=ROOT, check=True)
    result = subprocess.run([str(ROOT/'build/material-tests'),
                             str(ROOT/'data/validation-material.airframe'),
                             str(ROOT/'data/validation-bem.airframe')],
                            cwd=ROOT, capture_output=True, text=True, check=True)
    report = json.loads(result.stdout)
    report['hypotheses'] = {'full_roll_mass_effect_agrees_within_10pct':
                            report.pop('full_roll_effect_agreement_10pct')}
    report['unresolved'] = ('Matched 120/240 Hz full-roll closed-loop material effects disagree; '
        'the recorded 10% agreement hypothesis failed. Do not infer resolved small-perturbation '
        'closed-loop fidelity from the separate passing fixed-trim and conservation checks.')
    report['previous_counterexample'] = ('A ~1e-11 kg parcel failed with singular inertia under the '
        'absolute pivot cutoff. Matrix/rhs normalization removes the mass-scale dependency; '
        'the same positive parcel now integrates without a material mass floor.')
    report['assumptions'] = ('Remaining fraction scales a declared element mass and intrinsic inertia '
        'uniformly at its fixed centroid. Removed material inherits the rigid velocity field, '
        'without an ejection impulse, and evolves as a separate body with gravity but no aero geometry.')
    report['limits'] = ('Explicit material events only; no automatic conversion from cell area or spar '
        'section to mass, no projectile/fragment shape/drag/thermal model. A non-removable carrier '
        'on every material-bearing node retains a finite support body when all local material is removed.')
    (RESULTS/'material-summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
