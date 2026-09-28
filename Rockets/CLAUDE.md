# CLAUDE.md: Rocket4 (CC:Tweaked rocket for Minecraft Create Aeronautics / Sable)

Minecraft-only project: a 4-thruster cargo rocket flown by a CC:Tweaked computer.
Read `ROCKET4_HANDOFF.md` first. It has all hardware facts, measured physics, API quirks, the code architecture, the flight history and the rejected designs.

## Files
- `Rocket4.lua`: flight program (the deliverable).
- `VectorMap.lua`: thruster mapping test (already run).
- `sim4.lua`: offline simulator; loads Rocket4.lua with stubbed CC/Sable APIs.
- `ROCKET4_HANDOFF.md`: full knowledge base.

## Hard rules for Rocket4.lua
- Lua 5.1 (CC:Tweaked CraftOS 1.9). No comments, no `print()`. Clean code.
- All tunables go in the `local C = { ... }` table. Every `C.X` used must be defined there.
- Keep upvalues per function under 60 and locals under 200 (current max is 28 upvalues).
- All peripheral and sublevel calls in the flight loop go through the batched `E.io()` (one `parallel.waitForAll`). Never add sequential peripheral calls to the loop: each costs 50 ms.
- Never cut engines at altitude. Only the ground rule cuts (tilt > 60° within CUT_LOW of the ground).
- Recovery must never add climbing thrust.
- ARC and NET share the same route (AFTERBURNER → BRAKE → TRANSFER → SINK). Keep the mode name ARC.
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
- ARC: all land; 15 of 16 within 1.4 blocks, worst 2.8. 400 blocks ≈ 75 s, 7,000 ≈ 111 s, 12,000 ≈ 130 s.
- NET: 0.2–0.4 blocks, no CUT.

Any `TERMERR` line is a runtime error (the program's top-level pcall hides it otherwise).

## Debugging
- Phase timeline: `awk 'NR>2 && $2!=p {print; p=$2}' sim_flight.log`
- Kinematics (pos, vel, nose): `sim_trace.txt`
- Real flight logs from the user use the same `flight.log` format (see the handoff, section 5).
- The simulator matches real flights well except at 240 m/s AFTERBURNER; treat high-speed results as approximate.

## Current open work
See section 9 of `ROCKET4_HANDOFF.md`. First priority: analyze the user's next real ARC flight.log (3,000–7,000 blocks), especially AFTERBURNER stability at 200+ m/s and any RECOVER events.
