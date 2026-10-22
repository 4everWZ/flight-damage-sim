#!/usr/bin/env python3
"""Frozen CR-3439 queries: same-load reference translation, not physical mapping."""
import csv
import json
import math
import subprocess

from control_polar_calibration import ROOT, OUT, BUILD, export_control_rows, kernel_file
from control_polar_cr3439 import interpolation_weights

COEFFICIENTS = ('cl', 'cd', 'cm')
ORIGINS = {'quarter':0., 'leading':.25, 'trailing':-.75}
TOLERANCE = 1e-12


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def check_wrench(value, p, alpha, identity):
    """Check an exported native wrench against the independent FRD reference."""
    q_area = value['q']*value['area']; q_area_chord = q_area*value['chord']
    sine,cosine = math.sin(alpha),math.cos(alpha)
    offset = ORIGINS[identity[1]]
    # Independent FRD component formula, not native span/flow or Wrench helpers.
    force = (q_area*(p['cl']*sine-p['cd']*cosine),0.,-q_area*(p['cl']*cosine+p['cd']*sine))
    couple = (0.,q_area_chord*p['cm'],0.)
    arm = (-offset*value['chord'],0.,0.)
    shift = cross(arm,force)
    moment = tuple(c+m for c,m in zip(couple,shift))
    assert all(abs(value['arm_'+axis]-arm[i])/value['chord']<=TOLERANCE for i,axis in enumerate('xyz'))
    errors = {}
    for kind,columns,expected,scale in [('force',('wfx','wfy','wfz'),force,q_area),
                                       ('pure_couple',('mqx','mqy','mqz'),couple,q_area_chord),
                                       ('moment',('wmx','wmy','wmz'),moment,q_area_chord)]:
        errors[kind] = []
        for column,target in zip(columns,expected):
            error = abs(value[column]-target)/scale
            assert error <= TOLERANCE, (identity,kind,column,error)
            errors[kind].append(error)
    return errors


def check_negative_wrenches(record, row, prediction):
    """Corrupt copies of one native result, not the source data or reference."""
    value = {k:float(v) for k,v in record.items() if k!='reference'}
    identity = (row['id'],record['reference'])
    assert row['role']=='train' and record['reference']=='leading'
    scale = value['q']*value['area']*value['chord']
    native_shift = value['wmy']-value['mqy']
    assert abs(native_shift)/scale > TOLERANCE  # This point discriminates the faults.
    rejected = []
    for name,wrong_moment in [('reversed_arm_term',value['mqy']-native_shift),
                              ('omitted_arm_term',value['mqy'])]:
        faulty = {**value,'wmy':wrong_moment}
        try:
            check_wrench(faulty,prediction,math.radians(row['alpha_deg']),identity)
        except AssertionError as error:
            detail = error.args[0]
            # A parse/identity/arm failure is not evidence that the moment test works.
            assert detail[0]==identity and detail[1:3]==('moment','wmy'), detail
            rejected.append({'fault':name,'id':row['id'],'reference':record['reference'],
                             'rejected_by':'moment/wmy','normalized_error':detail[3]})
        else:
            raise AssertionError('wrong reference transformation was accepted: '+name)
    return rejected


def main():
    data = json.loads((ROOT/'data/control-polar-cr3439-section.json').read_text())
    train = [r for r in data['rows'] if r['role']=='train']
    held = [r for r in data['rows'] if r['role']=='heldout']
    assert len(train)==len(held)==6 and len(data['rows'])==12
    assert {(r['alpha_deg'],r['delta_deg']) for r in train} == {(a,d) for a in (-4,4) for d in (-20,0,20)}
    assert {(r['alpha_deg'],r['delta_deg']) for r in held} == {(a,d) for a in (-4,0,4) for d in (-10,10)}
    by_point = {(r['alpha_deg'],r['delta_deg']):r for r in train}
    queries = train+held
    predicted = {}
    for row in queries:
        weights = interpolation_weights(train,row)
        predicted[row['id']] = {k:sum(w*by_point[p][k] for p,w in weights.items()) for k in COEFFICIENTS}

    OUT.mkdir(exist_ok=True); BUILD.mkdir(exist_ok=True)
    records = export_control_rows({**data,'rows':train},data['conditions'],'cr3439_reference_transfer')
    fixture = kernel_file((ROOT/'data/validation-material.airframe').read_text(),records,'cr3439_reference_transfer',True)
    query_path = OUT/'control-polar-transfer-queries.csv'
    with query_path.open('w') as stream:
        writer = csv.writer(stream); writer.writerow(('alpha_deg','delta_deg'))
        writer.writerows((r['alpha_deg'],r['delta_deg']) for r in queries)
    coeff_path = OUT/'control-polar-transfer-coefficients.csv'
    native_path = OUT/'control-polar-transfer-native.csv'
    subprocess.run(['make','-j2','build/control-polar-reference-probe'],cwd=ROOT,check=True)
    subprocess.run(['build/control-polar-reference-probe',str(fixture),str(data['conditions']['mach']),
                    str(query_path),str(coeff_path),str(native_path)],cwd=ROOT,check=True)
    coefficients = [{k:float(v) for k,v in r.items()} for r in read_csv(coeff_path)]
    native = read_csv(native_path)
    assert len(coefficients)==12 and len(native)==36
    ledger = []
    by_id = {}
    angle_error = 0.
    outside_distance = 0.
    for row,actual in zip(queries,coefficients):
        assert (actual['alpha_deg'],actual['delta_deg'])==(row['alpha_deg'],row['delta_deg'])
        by_id[row['id']] = actual
        angle_error = max(angle_error,abs(actual['delta_eff_rad']-math.radians(row['delta_deg'])))
        assert abs(actual['flow_alpha_rad']-math.radians(row['alpha_deg'])) <= TOLERANCE
        excess = max(abs(actual['flow_alpha_rad'])-math.radians(4),abs(actual['delta_eff_rad'])-math.radians(20),0.)
        outside_distance = max(outside_distance,excess)
        assert excess <= TOLERANCE
        if actual['clamped']:
            assert min(abs(abs(actual['flow_alpha_rad'])-math.radians(4)),
                       abs(abs(actual['delta_eff_rad'])-math.radians(20))) <= TOLERANCE
        for key in COEFFICIENTS:
            assert abs(actual[key]-predicted[row['id']][key]) <= TOLERANCE
            if row['role']=='heldout':
                residual = actual[key]-row[key]
                bound = 2*data['uncertainty']['coefficient_reading_halfwidth'][key]
                ledger.append({'id':row['id'],'coefficient':key,'residual':residual,
                               'reading_only_residual_halfwidth':bound,
                               'outside_reading_only_envelope':abs(residual)>bound+TOLERANCE})
    assert angle_error <= TOLERANCE

    comparisons = []
    worst = {k:{'value':-1.} for k in ('force','pure_couple','moment')}
    seen = set()
    for index,record in enumerate(native):
        row = queries[index//3]; name = record['reference']; offset = ORIGINS[name]
        value = {k:float(v) for k,v in record.items() if k!='reference'}
        identity = (row['id'],name)
        assert identity not in seen; seen.add(identity)
        actual = by_id[row['id']]
        assert (value['alpha_deg'],value['delta_deg'])==(row['alpha_deg'],row['delta_deg'])
        assert value['origin_offset_chords']==offset
        assert value['area']==value['chord']==1
        assert value['q']>0 and all(math.isfinite(v) for v in value.values())
        for field in ('flow_alpha_rad','delta_eff_rad','clamped'):
            assert value[field]==actual[field]
        q_area_chord = value['q']*value['area']*value['chord']
        alpha = math.radians(row['alpha_deg']); sine,cosine = math.sin(alpha),math.cos(alpha)
        errors = check_wrench(value,predicted[row['id']],alpha,identity)
        result = {'id':row['id'],'role':row['role'],'reference':name}
        for kind,components in errors.items():
            for component,error in enumerate(components):
                if error>worst[kind]['value']:
                    worst[kind] = {'value':error,'id':row['id'],'reference':name,'component':'xyz'[component]}
            result[kind+'_max_normalized_error'] = max(components)
        # Native force and pure couple must be identical for all three observers.
        for axis in 'xyz':
            assert value['wf'+axis]==value['f'+axis]
        if name=='quarter':
            for axis in 'xyz':
                assert value['wm'+axis]==value['mq'+axis]
        else:
            previous = native[index-(1 if name=='leading' else 2)]
            assert previous['reference']=='quarter'
            for field in ('q','area','chord','fx','fy','fz','mqx','mqy','mqz'):
                assert record[field]==previous[field]
        result['translated_CM_residual'] = ''
        result['translated_reading_only_residual_halfwidth'] = ''
        if row['role']=='heldout':
            observed = row['cm']-offset*(row['cl']*cosine+row['cd']*sine)
            widths = data['uncertainty']['coefficient_reading_halfwidth']
            result['translated_CM_residual'] = value['wmy']/q_area_chord-observed
            result['translated_reading_only_residual_halfwidth'] = 2*(widths['cm']+abs(offset)*(abs(cosine)*widths['cl']+abs(sine)*widths['cd']))
        comparisons.append(result)
    assert seen=={(r['id'],name) for r in queries for name in ORIGINS}
    negative = check_negative_wrenches(native[1],queries[0],predicted[queries[0]['id']])
    maxima = {k:max(abs(r['residual']) for r in ledger if r['coefficient']==k) for k in COEFFICIENTS}
    counts = {k:sum(r['outside_reading_only_envelope'] for r in ledger if r['coefficient']==k) for k in COEFFICIENTS}
    # Preserve the published frozen regression outcome, never turn it into a fit.
    assert all(abs(maxima[k]-v)<=TOLERANCE for k,v in zip(COEFFICIENTS,(.0875,.0095,.0125)))
    assert counts=={'cl':6,'cd':3,'cm':3}
    summary = {'meaning':'Algebraic same-load reference translation through the native Wrench reducer; not physical section-to-cell validation.',
               'queries':12,'comparisons':36,'training_points':6,'heldout_points':6,
               'origins_offset_chords':ORIGINS,'normalized_tolerance':TOLERANCE,
               'worst_normalized_errors':worst,'actual_angle_max_error_rad':angle_error,
               'strict_clamp_flags_unique_queries':sum(int(r['clamped']) for r in coefficients),
               'max_angle_outside_table_rad':outside_distance,
               'frozen_heldout_max_absolute_residual':maxima,'frozen_outside_reading_only_counts':counts,
               'frozen_heldout_ledger':ledger,
               'limits':['Only existing 12 points, six training knots and the unchanged native unit-area/chord diagnostic cell.',
                         'Holdout reuse is regression evidence, not fresh independent measurement validation.',
                         'Source CM sign remains the previously stated inference; reference conversion cannot validate it.',
                         'Translated reading envelopes exclude angle/systematic uncertainty and cannot replace the original failure ledger.',
                         'No new source, geometry, physical cell mapping, aircraft calibration, default model or damage/material assumption.'],
               'physical_section_to_cell_validation':False}
    with (OUT/'control-polar-transfer-comparisons.csv').open('w') as stream:
        writer = csv.DictWriter(stream,fieldnames=list(comparisons[0]));writer.writeheader();writer.writerows(comparisons)
    (OUT/'control-polar-transfer-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    negative_summary = {'meaning':'Fault injection into native-output copies; same independent checker and unchanged tolerance.',
                        'positive_comparisons':36,'negative_cases':negative,'normalized_tolerance':TOLERANCE,
                        'source_or_native_model_modified':False,'physical_validation':False}
    (OUT/'control-polar-transfer-negative-summary.json').write_text(json.dumps(negative_summary,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k!='frozen_heldout_ledger'},indent=2))
    print(json.dumps(negative_summary,indent=2))


if __name__=='__main__':
    main()
