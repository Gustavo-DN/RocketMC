#!/usr/bin/env python3
"""Run real simulator flights in isolated directories; failures are never masked."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def cases(suite):
    if suite in ('core', 'all', 'uncapped'):
        for mode in ('WATER', 'SHUTTLE'):
            for lag in range(5):
                for aero in (-0.03, 0, 0.03):
                    case = dict(MODE=mode, LAG=lag, KAERO=aero,
                                TX=140.9, TZ=169.2, TY=-2.3)
                    if suite != 'uncapped':
                        case['TQMAX'] = 1.2
                    yield case
    if suite in ('extended', 'all'):
        for mode in ('WATER', 'SHUTTLE'):
            for x, z in ((150, 0), (300, 0), (0, 300), (-400, 600),
                         (1500, -900), (3000, 0), (5000, 0), (0, 0)):
                if mode == 'WATER' and x == z == 0:
                    continue
                yield dict(MODE=mode, TX=x, TZ=z)
            for mass, inertia in ((4, 8), (4.5, 12), (5.5, 15.57), (7, 25)):
                for lag in (0, 4):
                    yield dict(MODE=mode, TX=300, MASS=mass, INERTIA=inertia, LAG=lag)
            for lag in range(5):
                for aero in (-0.1, -0.03, 0.03, 0.1):
                    yield dict(MODE=mode, TX=300, LAG=lag, KAERO=aero, TQMAX=1.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', choices=('core', 'extended', 'all', 'uncapped'), default='core')
    parser.add_argument('--program', type=Path, default=ROOT / 'Rocket.lua')
    parser.add_argument('--lua', default='lua5.1')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    lua = shutil.which(args.lua)
    if not lua:
        parser.error('Lua 5.1 is required; use --lua /path/to/lua5.1')
    output = args.output or Path(tempfile.mkdtemp(prefix='rocket-regression-'))
    output.mkdir(parents=True, exist_ok=True)
    sim_variables = set(re.findall(r'os.getenv\("([A-Z0-9_]+)"\)', (ROOT / 'simrocket.lua').read_text()))
    base_env = {key: value for key, value in os.environ.items() if key not in sim_variables}

    def run(item):
        index, case = item
        directory = Path(tempfile.mkdtemp(prefix=f'{index:03d}-', dir=output))
        env = dict(base_env, **{key: str(value) for key, value in case.items()},
                   CHECK_FLIGHT='1', MAX_MISS='20', TMAX='300')
        command = [lua, str(ROOT / 'simrocket.lua'), str(args.program.resolve())]
        try:
            result = subprocess.run(command, cwd=directory, env=env, capture_output=True,
                                    text=True, timeout=30)
            text = result.stdout + result.stderr
            status = result.returncode
        except subprocess.TimeoutExpired:
            text, status = 'WALL TIME LIMIT', 124
        (directory / 'output.txt').write_text(text)
        log_path = directory / 'sim_flight.log'
        log = log_path.read_text() if log_path.exists() else ''
        match = re.search(r' miss=([0-9.]+)', text)
        miss = float(match[1]) if match else None
        passed = (status == 0 and 'ok=true' in text and 'crash=-' in text
                  and 'result_miss=-' not in text and bool(log)
                  and not re.search(r'TERMERR|PRINT:|SIM TIME LIMIT', text)
                  and not re.search(r'^CUT ', log, re.M)
                  and miss is not None and math.isfinite(miss) and miss <= 20)
        return dict(case=case, passed=passed, exit_code=status, miss=miss,
                    output=text.strip(), directory=str(directory))

    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(run, enumerate(cases(args.suite))))
    (output / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
    passed = sum(result['passed'] for result in results)
    print(f'{args.suite}: {passed}/{len(results)} passed (physical splash miss <= 20 blocks, no crash/CUT/runtime error)')
    for result in results:
        if not result['passed']:
            print('FAIL', result['case'], result['output'])
    print(f'Logs: {output}')
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
