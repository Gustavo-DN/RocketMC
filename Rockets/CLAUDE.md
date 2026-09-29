# CLAUDE.md: Rocket4 (CC:Tweaked rocket for Minecraft Create Aeronautics / Sable)

Minecraft-only project: a 4-thruster cargo rocket flown by a CC:Tweaked computer.
It is a video-game delivery vehicle. It carries items (food that spoils) between bases in a Minecraft world and arrives on a rope net (upright in NET mode, nose-down in ARC mode) or in a water pool. Nothing here relates to real-world hardware.

Naming: the destination is where the cargo lands. Phases are IGNITION, ASCENT, EXPRESS (fast level cruise; its constants use the `AB_` prefix), TRANSFER, SINK, DIVE, RELEASE. Delivery modes are WATER, ARC and NET.
Read `ROCKET4_HANDOFF.md` first. It has all hardware facts, measured physics, API quirks, the code architecture, the flight history and the rejected designs.

## Files
- `Rocket4.lua`: flight program (the deliverable).
- `VectorMap.lua`: thruster mapping test (already run).
- `sim4.lua`: offline simulator; loads Rocket4.lua with stubbed CC/Sable APIs.
- `ROCKET4_HANDOFF.md`: full knowledge base.
- `SpeedTest.lua`: separate test program. `SpeedTest <x> <z> [NET|ARC] [climb]` runs Rocket4 unchanged on a REMOTE mission and only listens to its thruster commands and sub-level reads (no extra peripheral calls). It writes `speedtest.log`: a summary of the level full-throttle cruise (max speed, time to 100–280 m/s, altitude swing, pitch wobble, nose vs flight path, spin, gimbal use, throttle, recoveries, fuel), then 0.25 s rows. Rocket4 must not depend on it.

## Hard rules for Rocket4.lua
- Lua 5.1 (CC:Tweaked CraftOS 1.9). No comments, no `print()`. Clean code.
- All tunables go in the `local C = { ... }` table. Every `C.X` used must be defined there.
- Keep upvalues per function under 60 and locals under 200 (current max is 28 upvalues).
- All peripheral and sublevel calls in the flight loop go through the batched `E.io()` (one `parallel.waitForAll`). Never add sequential peripheral calls to the loop: each costs 50 ms.
- Never cut engines at altitude. Only the ground rule cuts (tilt > 60° within CUT_LOW of the ground).
- Recovery must never add climbing thrust.
- NET route: EXPRESS → BRAKE → TRANSFER → SINK (upright soft landing).
- ARC route (ARC_FAST, default): EXPRESS → DIVE with no braking. The descent starts when the line of sight to the destination is ARC_DIVE_ANG below horizontal (plus ARC_DIVE_T lead) and flies nose-first at ARC_DIVE_THR with `approachDir`. A recovery returns to DIVE. The net is very large, so speed matters more than precision. With ARC_FAST = false, ARC brakes (BRAKE_STYLE) and uses the slow `arcDir` DIVE.
- Deliver the finished file; the user copies it into the game.

## Setup
```bash
sudo apt-get install -y lua5.1
```

## Mandatory checks after every edit
```bash
luac5.1 -p Rocket4.lua
for k in $(grep -o "C\.[A-Z_0-9]*" Rocket4.lua | sort -u | sed 's/C\.//'); do grep -q "^  $k = " Rocket4.lua || echo "MISSING $k"; done
luac5.1 -l Rocket4.lua | grep -E "^[0-9]+\+? params" | awk -F', ' '{print $3}' | sort -n -r | head -1
grep -n "print(" Rocket4.lua
```

## Regression test (run before delivering)
```bash
run() { out=$(env "$@" TMAX=300 timeout 250 lua5.1 sim4.lua Rocket4.lua 2>&1); echo "[$*] $(echo "$out" | grep -o 'TERMERR.*\|end_t=[0-9.]*\|miss_xz=[0-9.]*\|maxY=[0-9]*' | tr '\n' ' ') rec=$(grep -c '^RECOVER' sim_flight.log) $(grep '^CUT' sim_flight.log)"; }
for c in "TX=7000 TZ=0" "TX=4000 TZ=3000" "TX=1500 TZ=0" "TX=400 TZ=0" "TX=200 TZ=150" "TX=900 TZ=-600" "TX=12000 TZ=-2000" \
         "TX=7000 TZ=0 KAERO=0.045" "TX=7000 TZ=0 KAERO=0.015" "TX=7000 TZ=0 LATMAX=25" "TX=2500 TZ=2500 KLAT=1.0" \
         "TX=-3000 TZ=-5000 KLAT=0.8" "TX=5000 TZ=0 KAERO=0.04 KLAT=0.7" "TX=3000 TZ=-3000 TQMAX=1.8 LATMAX=16"; do run MODE=ARC $c; done
for c in "TX=300 TZ=0" "TX=-400 TZ=600" "TX=1500 TZ=-900" "TX=-2500 TZ=300" "TX=300 TZ=0 KAERO=0.045"; do run MODE=NET $c; done
```

Baseline to keep or beat:
- ARC (ARC_FAST): nose tip within about 1–10 blocks in most runs, arriving nose-down at 100–175 m/s. 400 blocks ≈ 25 s, 7,000 ≈ 62 s, 12,000 ≈ 83 s. The simulator prints `tip_xz`, `impV` and `landTilt`.
- Previous upright ARC (commit 4d8760b), for reference: 15 of 16 within 1.4 blocks, worst 2.8.
- NET: 0.2–0.4 blocks, no CUT.

Any `TERMERR` line is a runtime error (the program's top-level pcall hides it otherwise).

## Debugging
- Phase timeline: `awk 'NR>2 && $2!=p {print; p=$2}' sim_flight.log`
- Kinematics (pos, vel, nose): `sim_trace.txt`
- Real flight logs from the user use the same `flight.log` format (see the handoff, section 5).
- The simulator matches real flights well except at 240 m/s EXPRESS; treat high-speed results as approximate.

## Current open work
See section 9 of `ROCKET4_HANDOFF.md`. First priority: check the user's next real ARC flight.log with ARC_FAST. Known weak spot: the high-speed EXPRESS pitch wobble (real log `logs/2026-09-28_ARC7000_flight.log`, 15–32 s). Stress-test physics that reproduce that flight: `CAX=0.085 LATMAX=20..25 TQMAX=2.5..3 KAERO=0.03..0.04 TY=-36 CLIMB=200 MASS=67.4`.
