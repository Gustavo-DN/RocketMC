#!/usr/bin/env python3
"""Validate the standalone limit experiment, including termination and powered recovery."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lua', default='lua5.1')
    args = parser.parse_args()
    lua = shutil.which(args.lua)
    if not lua:
        parser.error('Lua 5.1 is required')
    output = Path(tempfile.mkdtemp(prefix='rocket-limit-'))
    variables = set(re.findall(r'os.getenv\("([A-Z0-9_]+)"\)', (ROOT / 'simrocket.lua').read_text()))
    base = {k: v for k, v in os.environ.items() if k not in variables}
    cases = [{}]
    cases += [dict(LAG=lag, KAERO=aero, TQMAX=1.2)
              for lag in range(5) for aero in (-0.03, 0.03)]
    cases += [dict(DRAG=0.01), dict(LAG=4, KAERO=-0.1, TQMAX=4),
              dict(MASS=4, INERTIA=8), dict(MASS=7, INERTIA=25),
              dict(CLI_HEADING=90, CLI_HEIGHT=200), dict(EMPTY_FUEL=1)]

    def run(item):
        index, case = item
        directory = output / str(index)
        directory.mkdir()
        env = dict(base, TMAX='1200', TX='1000000', TZ='1000000',
                   **{k: str(v) for k, v in case.items()})
        program = ROOT / 'RocketLimitTest.lua'
        if 'CLI_HEADING' in case or 'EMPTY_FUEL' in case:
            program = directory / 'fixture.lua'
            target = json.dumps(str(ROOT / 'RocketLimitTest.lua'))
            if 'CLI_HEADING' in case:
                program.write_text(f'assert(loadfile({target}))({case["CLI_HEADING"]}, {case["CLI_HEIGHT"]})\n')
            else:
                program.write_text('local original = peripheral.call\n'
                                   'peripheral.call = function(...) if T > 50 then return {} end return original(...) end\n'
                                   f'assert(loadfile({target}))(... )\n')
        result = subprocess.run([lua, str(ROOT / 'simrocket.lua'), str(program)],
                                env=env, cwd=directory, capture_output=True, text=True, timeout=30)
        text = result.stdout + result.stderr
        (directory / 'output.txt').write_text(text)
        summary = (directory / 'sim_rocketlimit.txt').read_text()
        log = (directory / 'sim_rocketlimit.log').read_text()
        rows = [line.split() for line in log.splitlines() if line and line[0].isdigit()]
        reason = re.search(r'^STOP (.+)$', summary, re.M)[1]
        problems = []
        if result.returncode or 'ok=true' not in text or re.search(r'ERROR|TERMERR|SIM TIME LIMIT', text):
            problems.append('runtime failure')
        if 'SINGLE THRUSTER LIMIT TEST - LANDED' not in summary:
            problems.append('did not land upright')
        if log.count('EVENT TEST_END ') != 1 or 'EVENT LANDED' not in log:
            problems.append('missing or duplicate test termination')
        recovery = [row for row in rows if row[1] == 'RECOVER']
        if not recovery or any(int(row[2]) <= 0 for row in recovery):
            problems.append('missing powered recovery')
        if not rows or any(len(row) != 30 for row in rows):
            problems.append('invalid telemetry columns')
        if any(row[26] == 'NA' or row[27] == 'NA' or float(row[28]) != 50 for row in rows):
            problems.append('missing gimbal reads or extra peripheral wait')
        if case == {}:
            run_rows = [row for row in rows if row[1] == 'RUN']
            if not run_rows or not all(100 <= float(row[6]) <= 200 for row in run_rows):
                problems.append('nominal run outside 100–200 block altitude band')
            changes = []
            previous = None
            for row in run_rows:
                if row[2] != previous:
                    changes.append(float(row[0]))
                    previous = row[2]
            if any(b - a < 9.9 for a, b in zip(changes, changes[1:])):
                problems.append('throttle increased before ten-second dwell')
            events = [line for line in log.splitlines() if line.startswith('EVENT STEP ')]
            if [int(line.split()[2]) for line in events] != list(range(4, 16)):
                problems.append('throttle did not progress one step at a time')
        if case.get('DRAG') == 0.01:
            speed = float(re.search(r'TEST_END_SPEED ([\d.]+)', summary)[1])
            if reason != '1000 M/S' or not 1000 <= speed < 1010:
                problems.append('speed ceiling not enforced promptly')
        if case.get('EMPTY_FUEL') and reason != 'FUEL EMPTY - LIMIT INVALID':
            problems.append('fuel starvation mistaken for a speed limit')
        if case.get('CLI_HEADING') and 'HEADING 1.0000 0.0000' not in summary:
            problems.append('explicit heading ignored')
        if case.get('TQMAX') == 4 and reason not in ('SPIN', 'HARD SPIN'):
            problems.append('unstable case not detected')
        return dict(case=case, reason=reason, passed=not problems, problems=problems)

    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(run, enumerate(cases)))
    (output / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
    for result in results:
        print('PASS' if result['passed'] else 'FAIL', result['case'], result['reason'], result['problems'])
    print(f'{sum(r["passed"] for r in results)}/{len(results)} passed; logs: {output}')
    return 0 if all(r['passed'] for r in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
