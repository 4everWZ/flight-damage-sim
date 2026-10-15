#!/usr/bin/env python3
"""Reproduce cell ownership, separated-body dynamics and control partition in C++."""
import json
import subprocess
from experiments import ROOT, RESULTS


def main():
    RESULTS.mkdir(exist_ok=True)
    subprocess.run(['make', 'build/detachment-tests'], cwd=ROOT, check=True)
    result = subprocess.run([str(ROOT/'build/detachment-tests'),
                             str(ROOT/'data/validation-bem.airframe'),
                             str(RESULTS/'detached-flight.csv')], cwd=ROOT,
                            capture_output=True, text=True, check=True)
    report = json.loads(result.stdout)
    report['scope'] = ('Shared synthetic cell law; own CG/rates/altitude/wind; '
                       'frozen separation deformation; unpowered neutral relaxation '
                       'or mechanical jam. No inter-body wake/collision, secondary '
                       'fracture, detached engines or detailed hinge dynamics.')
    report['partition_density_note'] = ('Force/moment reduction compares equal density. '
        'The existing one-point-per-body atmosphere slightly changes density samples '
        'when a tilted body is partitioned; the unscaled force jump is reported separately.')
    report['high_speed_entry'] = '140 m/s transient, not sustained trimmed cruise'
    (RESULTS/'detachment-summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
