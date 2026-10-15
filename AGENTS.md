# AGENTS.md: handoff for Codex (or any coding agent)

Minecraft-only project. Create Aeronautics / Sable craft (rockets, a small plane) flown by CC:Tweaked computers.
Everything lives in `Rockets/`. Nothing here relates to real-world hardware.

## Read first, in this order
1. `Rockets/CLAUDE.md`: file-by-file history of every program, real flight results, the simulators, the mandatory checks and the hard rules. Each bullet is a dated finding. Read all of it.
2. `Rockets/ROCKET4_HANDOFF.md`: the Sable/CC API quirks, measured physics, the flight.log format and the rejected designs (for the 4-thruster rocket, but the API facts apply to every craft).
3. The program you are changing, in full. Then the matching simulator.

## Current goal: a better single-thruster rocket (`Rockets/Rocket.lua`)
State of the work:
- Hardware: 1 liquid vector thruster on `bottom`, filled by hand with 1 bucket of lava (top tank removed). Measured in `logs/2026-10-10_smalltest.*`: mass ~5.5, inertia pitch/yaw 15.6, roll 0.92, about 77.6 m/s² per unit throttle (very overpowered). Throttle and gimbal are quantized to 1/15. Gimbal lag ~0.23 s. There is no roll control with one thruster.
- Gimbal map (measured): X+ → ang_z +3.37, Y+ → ang_x −1.95 (with ang_z −0.94 coupling). Use the map; do not assume gimbal axes = body axes.
- Modes: WATER (ground launch, lands in a water pool at the target) and SHUTTLE (released from a carrier in flight).
- `tune()` reads mass and inertia at launch and scales thrust and steering gain.
- **Open problem:** the only real WATER flight (`logs/2026-10-11_smallrocket_water_tumble_flight.log`) climbed cleanly for 2.5 s, then a growing pitch oscillation at 10–20 m/s tumbled it. The simulator did not predict it. It only roughly reproduces it with extra command delay (`LAG=2..4`) or destabilizing aero (`KAERO` negative). The cause is not identified. Start by reading that log against the code.
- What worked on the real plane (`Plane.lua`, same author, same API) and is a strong candidate to port: α-space PID through the measured gimbal map (`steer()`), rate feed-forward, all reads/writes batched in one `parallel.waitForAll` per tick, inertia-based axis detection in `tune()`, the slow stepwise climb (VTAKE adds throttle only while climbing slower than a target speed), and the impact-detected flare. See the Plane.lua bullets in `Rockets/CLAUDE.md`.
- Simulator: `cd Rockets && lua5.1 simrocket.lua Rocket.lua` (env MODE TX TY TZ CLIMB GROUND MASS INERTIA LAG KAERO TQMAX …, listed at the top of `simrocket.lua`). Any new design must survive LAG 0–4 and KAERO ± before it is shipped, because the real failure was not predicted.
- `SmallTest.lua` is the on-ground/hover test for this rocket; extend it if you need a new real measurement (for example a step response at 10–20 m/s climb).

## Hard rules
- Lua 5.1 (CC:Tweaked). Max 200 locals per function. Put tunables in one `C = { ... }` table and check every `C.X` used is defined.
- Every peripheral or sublevel call in the flight loop goes in one batched `parallel.waitForAll` per tick. Each sequential call costs ~50 ms.
- Never cut the engine at altitude. Only ground-level safety cuts are allowed.
- After every edit: `luac5.1 -p <file>`, the missing-`C.` check and the simulator runs listed in `Rockets/CLAUDE.md`.
- When you learn something from a real flight, add a dated bullet to `Rockets/CLAUDE.md` and save the log in `Rockets/logs/` with a descriptive name.
- **Security:** `Transport.lua` stores only a salted, 200-round SHA-256 password hash. Never write the user's password in plaintext anywhere in the repo, in commits or in logs.
- The user copies the finished `.lua` file into the game by hand, then sends back `flight.log`. Keep the log format compatible.

## Setup
```bash
sudo apt-get install -y lua5.1
```

## Other craft (stable, don't break them)
- `Plane.lua`: works on the real plane (tail-sitter takeoff, glide-slope landing, impact flare). Waiting on a real log to tune `IMPACT_DECEL`.
- `Transport.lua` / `Rocket4.lua`: 4-thruster rocket. If roll runs away (`roll` pinned at ±0.30 while `wy` grows), reverse `ENGINE_ORDER`.
