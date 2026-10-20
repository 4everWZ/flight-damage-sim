#!/usr/bin/env python3
"""Fixed-condition coefficient preparation, not physical aircraft validation.

Published program output and synthetic integration controls stay separate.
No runtime physics, existing fixture, or frozen PR19 experiment is modified.
"""
import copy
import csv
import json
import math
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results'
BUILD = ROOT/'build'
IDENTITY = ('airfoil', 'geometry_role', 'flap_chord_ratio', 'mach', 'delta_positive',
            'moment_reference_x_over_c', 'moment_positive', 'coefficient_axes',
            'reynolds', 'transition', 'flap_gap')


def compression(mach):
    return 1/math.sqrt(max(1-mach*mach, .35))


def coverage(dataset):
    rows = dataset['rows']
    alphas = sorted({r['alpha_deg'] for r in rows})
    deltas = sorted({r['delta_deg'] for r in rows})
    complete = len(rows) == len(alphas)*len(deltas) == len({(r['alpha_deg'], r['delta_deg']) for r in rows})
    missing = [k for k in ('flap_chord_ratio', 'reynolds', 'transition', 'flap_gap') if dataset['conditions'][k] is None]
    return {'alpha_stations': alphas, 'delta_stations': deltas, 'rectangular': complete,
            'two_dimensional_coverage': complete and len(alphas) >= 2 and len(deltas) >= 2,
            'unknown_physical_conditions': missing,
            'experimental_validation_performed': False,
            'evidence_kind': dataset['evidence_kind']}


def normalized_rows(dataset, target):
    conditions = dataset['conditions']
    if dataset['evidence_kind'] not in ('published_program_output', 'synthetic_integration', 'measurement'):
        raise ValueError('declare the evidence kind explicitly')
    # No implicit transfers across Mach, geometry, moment origin, transition
    # or sign conventions. Unknowns remain unknown even when both are None.
    if (any(conditions[k] != target[k] for k in IDENTITY)
            or conditions.get('configuration') != target.get('configuration')):
        raise ValueError('source/target condition or convention mismatch')
    mach = conditions['mach']
    if not isinstance(mach, (int, float)) or not math.isfinite(mach) or mach <= 0:
        raise ValueError('a positive fixed source Mach is required for this replay')
    if conditions['lift_mach_basis'] != 'already_conditioned':
        raise ValueError('declare coefficient Mach content; never infer it')
    pg = compression(mach)
    result = []
    for r in dataset['rows']:
        if not all(math.isfinite(r[k]) for k in ('alpha_deg', 'delta_deg', 'cl', 'cd', 'cm')) or r['cd'] < 0:
            raise ValueError('invalid coefficient row')
        result.append({**r, 'cl': r['cl']/pg})
    return result


def export_control_rows(dataset, target, name):
    rows = normalized_rows(dataset, target)
    if not coverage(dataset)['two_dimensional_coverage']:
        raise ValueError('cannot export a 2D control polar from incomplete/one-delta data')
    if dataset['evidence_kind'] == 'measurement' and coverage(dataset)['unknown_physical_conditions']:
        raise ValueError('measured conditions/geometry are incomplete; no matched export')
    # Deliberately no cell binding: export alone cannot enable the model.
    header = (f"# Fixed-condition coefficient preparation: kind={dataset['evidence_kind']}; "
              f"Mach={target['mach']}; moment_x/c={target['moment_reference_x_over_c']}; "
              "no aircraft calibration claim\n")
    return header+'\n'.join(f"control_polar {name} {r['alpha_deg']:.17g} {r['delta_deg']:.17g} "
                     f"{r['cl']:.17g} {r['cd']:.17g} {r['cm']:.17g}"
                     for r in sorted(rows, key=lambda r: (r['delta_deg'], r['alpha_deg'])))+'\n'


def kernel_file(text, records, name, two_dimensional):
    lines = []
    for line in text.splitlines():
        f = line.split()
        if not two_dimensional and f and f[0] == 'cell' and f[1] == '34':
            f[-1] = name
            line = ' '.join(f)
        lines.append(line)
    if two_dimensional:
        records += f'cell_control_polar 34 {name}\n'
    path = BUILD/f'control-polar-calibration-{name}.airframe'
    path.write_text('# Generated normalized force-kernel replay only; not a calibrated aircraft.\n'+'\n'.join(lines)+'\n'+records)
    return path


def replay(path, mach, points, name):
    query = OUT/f'control-polar-calibration-{name}-queries.csv'
    with query.open('w') as stream:
        w = csv.writer(stream); w.writerow(('alpha_deg', 'delta_deg'))
        w.writerows((r['alpha_deg'], r['delta_deg']) for r in points)
    target = OUT/f'control-polar-calibration-{name}.csv'
    subprocess.run(['build/control-polar-reference-probe', str(path), str(mach), str(query), str(target)], cwd=ROOT, check=True)
    with target.open() as stream:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(stream)]


def main():
    OUT.mkdir(exist_ok=True)
    subprocess.run(['make', '-j2', 'build/control-polar-reference-probe'], cwd=ROOT, check=True)
    text = (ROOT/'data/validation-material.airframe').read_text()
    published = json.loads((ROOT/'data/control-polar-camrad-neutral-reference.json').read_text())
    target = copy.deepcopy(published['conditions'])
    readiness = coverage(published)
    rejected = 0
    for key, value in [('mach', .4), ('airfoil', 'NACA 2212'), ('moment_reference_x_over_c', .5),
                       ('delta_positive', 'trailing_edge_up')]:
        wrong = {**target, key: value}
        try:
            normalized_rows(published, wrong)
        except ValueError:
            rejected += 1
        else:
            raise AssertionError('mismatched conditions were silently accepted')
    try:
        export_control_rows(published, target, 'invalid_source_surface')
    except ValueError:
        rejected += 1
    else:
        raise AssertionError('missing flap data must not be fabricated')

    # Neutral-only reference replay through the common force kernel uses a
    # legacy one-dimensional curve. It never fabricates neighboring delta rows.
    train = [r for r in published['rows'] if r['alpha_deg'] in (-8, 0, 8)]
    ptrain = {**published, 'rows': train}
    records = lambda rows: '\n'.join(f"curve neutral {r['alpha_deg']} {r['cl']:.17g} {r['cd']:.17g} {r['cm']:.17g}" for r in rows)+'\n'
    naive = kernel_file(text, records(train), 'neutral', False)
    naive_rows = replay(naive, target['mach'], published['rows'], 'neutral-naive')
    corrected = kernel_file(text, records(normalized_rows(ptrain, target)), 'neutral', False)
    corrected_rows = replay(corrected, target['mach'], published['rows'], 'neutral-normalized')
    pg = compression(target['mach'])
    knot_error = max(abs(actual[k]-source[k]) for actual, source in zip(corrected_rows, published['rows'])
                     if source['alpha_deg'] in (-8, 0, 8) for k in ('cl', 'cd', 'cm'))
    assert knot_error < 1e-12
    excess = [actual['cl']/source['cl']-1 for actual, source in zip(naive_rows, published['rows'])
              if source['alpha_deg'] in (-8, 8)]
    assert all(abs(x-(pg-1)) < 1e-12 for x in excess)
    heldout = {k: max(abs(a[k]-r[k]) for a, r in zip(corrected_rows, published['rows'])
                     if r['alpha_deg'] not in (-8, 0, 8)) for k in ('cl', 'cd', 'cm')}

    # A complete manufactured target verifies preparation + the actual 2D path.
    # It is deliberately NOT labeled NACA data or experimental validation.
    def synthetic_row(alpha, delta):
        a, d = math.radians(alpha), math.radians(delta)
        return {'alpha_deg': alpha, 'delta_deg': delta,
                'cl': .1+2*a+.8*d+.3*a*d, 'cd': .04+.01*a+.02*d+.03*a*d,
                'cm': .02-.1*a+.1*d+.1*a*d}
    synthetic = {'evidence_kind': 'synthetic_integration', 'conditions':
                 {**target, 'airfoil': 'manufactured-bilinear-law', 'geometry_role': 'normalized force-kernel fixture', 'flap_chord_ratio': .25},
                 'rows': [synthetic_row(a, d) for a in (-8, 0, 8) for d in (-10, 0, 10)]}
    incomplete_measurement = {**synthetic, 'evidence_kind': 'measurement'}
    try:
        export_control_rows(incomplete_measurement, synthetic['conditions'], 'incomplete_measurement')
    except ValueError:
        rejected += 1
    else:
        raise AssertionError('unknown physical measurement conditions were accepted')
    prepared = export_control_rows(synthetic, synthetic['conditions'], 'prepared')
    enabled = kernel_file(text, prepared, 'prepared', True)
    queries = [synthetic_row(a, d) for a in (-8, -4, 0, 4, 8) for d in (-10, -5, 0, 5, 10)]
    actual = replay(enabled, .3, queries, 'synthetic-2d-normalized')
    error = max(abs(a[k]-r[k]) for a, r in zip(actual, queries) for k in ('cl', 'cd', 'cm'))
    assert error < 1e-12
    assert 'cell_control_polar' not in prepared
    summary = {'review': 'No demonstrated defect found in PR19 runtime law. The demonstrated pitfall is importing Mach-conditioned CL without removing the already-present core lift multiplier.',
               'frozen_implementation': '4d596b0a13effca62deba1dc915b7167772f40e1',
               'published_source': published['source'], 'published_coverage': readiness,
               'normalization': {'source_mach': .3, 'core_lift_multiplier': pg,
                                 'naive_relative_CL_excess': excess[0], 'normalized_knot_max_error': knot_error,
                                 'rule': 'At the same declared Mach, table CL = supplied Mach-conditioned CL / core PG. CD and CM pass through. This is an input convention adapter, not a new compressibility law.'},
               'published_output_heldout_interpolation_error': heldout,
               'synthetic_complete_2d': {'samples': len(actual), 'max_coefficient_error': error,
                                         'physical_validation': False},
               'mismatch_and_incomplete_exports_rejected': rejected,
               'scope': 'Static normalized section coefficient replay using actual control-chain angles; no new flights, production tables, physical model changes or empirical calibration.',
               'gaps': ['Published sample has only delta=0; flapped numerical output is elided.',
                        'The sample is program output, not an identified measured wind-tunnel dataset.',
                        'Reynolds number, transition, flap-gap state and uncertainty remain unknown.',
                        'NACA 0015 sample is not source-matched Spitfire wing/tail geometry.',
                        'Prepared data is fixed-Mach; no cross-Mach or new Reynolds model is introduced.'],
               'next_experiment': 'Obtain a complete measured alpha/delta section dataset with geometry, Mach/Re, transition, gap/seal, signs and moment origin. Normalize conventions, withhold interior rows, and compare coefficients before any aircraft fit.'}
    (OUT/'control-polar-calibration-summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
