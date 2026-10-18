#!/usr/bin/env python3
"""Real core/consumer ABI checks across incremental compiler-setting changes."""
import csv
import io
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
FAST = '-std=c++20 -O0 -g0 -Wall -Wextra -Wpedantic'
TARGETS = ['build/finite-span-probe', 'build/build-config-probe']


def main():
    report = {'passed': False, 'cases': [],
              'scope': 'Real C++ core with libstdc++; -O0/-g0 bounds regression cost. Standard optimized default builds are checked by the existing CI. No simulation source or fixture is altered.'}
    env = os.environ.copy()
    for key in ('MAKEFLAGS', 'MFLAGS', 'MAKELEVEL', 'GNUMAKEFLAGS'):
        env.pop(key, None)
    try:
        report['compiler'] = subprocess.check_output(['g++', '--version'], text=True).splitlines()[0]
        with tempfile.TemporaryDirectory(prefix='flight-build-config-') as directory:
            root = Path(directory)
            shutil.copy2(ROOT/'Makefile', root/'Makefile')
            for folder in ('include', 'src'):
                shutil.copytree(ROOT/folder, root/folder)
            (root/'tests').mkdir()
            for name in ('finite_span_probe.cpp', 'build_config_probe.cpp'):
                shutil.copy2(ROOT/'tests'/name, root/'tests'/name)
            (root/'data').mkdir()
            shutil.copy2(ROOT/'data/validation.airframe', root/'data/validation.airframe')
            cells = sum(line.startswith('cell ') for line in (root/'data/validation.airframe').read_text().splitlines())

            def command(args):
                result = subprocess.run(args, cwd=root, env=env, capture_output=True,
                                        text=True, timeout=180)
                if result.returncode:
                    raise RuntimeError(f'{args!r}: exit {result.returncode}\n{result.stdout}\n{result.stderr}')
                return result

            def settings(cpp='-Iinclude', cxx=FAST, compiler='g++'):
                return {'CXX': compiler, 'CPPFLAGS': cpp, 'CXXFLAGS': cxx}

            def make_args(config, goals=TARGETS, option='-j2'):
                return ['make', option, *[f'{k}={v}' for k, v in config.items()], *goals]

            base = settings()
            command(make_args(base, ['build/flight-run'], '-n'))
            assert not (root/'build').exists(), 'fresh dry-run must not write build files'
            command(make_args(base, ['build/flight-run']))

            def case(name, config, debug=False, cpp=0, cxx=0, label='default'):
                entry = {'name': name, 'settings': config, 'command': make_args(config)}
                report['cases'].append(entry)
                built = command(entry['command'])
                probe = command(['./build/build-config-probe', 'data/validation.airframe'])
                observed = json.loads(probe.stdout)
                entry['observed'] = observed
                assert observed['debug_stl'] == int(debug)
                assert observed['cpp_tag'] == cpp and observed['cxx_tag'] == cxx
                assert observed['label'] == label
                assert observed['cells'] == cells and observed['chains'] > 0
                finite = command(['./build/finite-span-probe', 'data/validation.airframe'])
                rows = list(csv.DictReader(io.StringIO(finite.stdout)))
                assert len(rows) == 36
                assert all(math.isfinite(float(v)) for r in rows for k, v in r.items() if k != 'case')
                entry['finite_span_rows'] = len(rows)
                entry['core_compiles'] = {s: built.stdout.count(f'-c src/{s}.cpp ') for s in ('core', 'propeller')}
                assert entry['core_compiles'] == {'core': 1, 'propeller': 1}, 'changed settings must compile both shared objects once'
                entry['passed'] = True

            # New consumers after ordinary objects exist: the reported ABI bug.
            cpp_debug = settings('-Iinclude -D_GLIBCXX_DEBUG -DBUILD_CONFIG_CPP=17')
            case('new_cpp_debug_consumers', cpp_debug, debug=True, cpp=17)
            # Existing paths must switch back too; config-specific object paths
            # alone would not guarantee relinking a newer, fixed-path binary.
            case('existing_consumers_back_to_base', base)
            case('cxx_debug_consumers', settings(cxx=FAST+' -D_GLIBCXX_DEBUG -DBUILD_CONFIG_CXX=23'),
                 debug=True, cxx=23)
            quoted = settings('-Iinclude '+shlex.quote('-DBUILD_CONFIG_LABEL="it\'s_safe"'))
            case('quoted_cpp_flags', quoted, label="it's_safe")
            before = {p: p.stat().st_mtime_ns for p in (root/'build').iterdir()
                      if p.suffix == '.o' or p.name in ('finite-span-probe', 'build-config-probe')}
            command(make_args(quoted))
            assert all(p.stat().st_mtime_ns == mtime for p, mtime in before.items()), 'unchanged settings must not rebuild'
            assert subprocess.run(make_args(quoted, option='-q'), cwd=root, env=env).returncode == 0
            compiler = settings(quoted['CPPFLAGS'], compiler='env g++')
            query = subprocess.run(make_args(compiler, option='-q'), cwd=root, env=env)
            assert query.returncode == 1, 'compiler command change must be out of date'
            case('changed_compiler_command', compiler, label="it's_safe")
            case('final_base_consumers', base)
            report['unchanged_build_reused'] = True
            report['fresh_dry_run_read_only'] = True
            report['compiler_change_query_out_of_date'] = True
            report['passed'] = True
    except Exception as exc:
        report['error'] = str(exc)
    destination = ROOT/'results/build-config-summary.json'
    destination.parent.mkdir(exist_ok=True)
    destination.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    if not report['passed']:
        raise SystemExit('Build configuration regression failed; inspect build-config-summary.json')


if __name__ == '__main__':
    main()
