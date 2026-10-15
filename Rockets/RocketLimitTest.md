# RocketLimitTest: top speed of the single-thruster rocket

Copy `RocketLimitTest.lua` to the rocket's computer. Thruster on `bottom` (any side works), rocket standing upright on flat ground, creative lava attached (it pulls from every attached fluid tank every 0.5 s).

```
RocketLimitTest [height 100-200] [north|south|east|west|F3 yaw]
RocketLimitTest 150 east
```
Default: 150 blocks, north. Backspace or Ctrl+T at any time ends the test and lands.

## What it does (v2, full throttle)
1. LIFT, then ID (a couple of blocks up): gimbal kicks to learn steering, and it measures the real thrust.
2. CLIMB at 6 m/s, HOLD 5 s (measures hover thrust again; goes anyway after 30 s if not calm).
3. RUN: FULL THROTTLE straight away. Nose pitches over at 15°/s; the thrust is kept just above what holds the height and the rest goes forward. No speed ramp, no track or altitude limits (logged only).
4. Ends only on: 1000 m/s, real top speed (< 2 m/s gained in 20 s at full throttle), loss of control (nose > 35° off for 1 s or > 60°, spin > 2.5 rad/s for 1 s or > 4), dropping to 30 blocks above the start, Backspace, or 15 min.
5. Then brakes (nose kept near the flight path at high speed), descends at 3 m/s and cuts the engine on the ground.

The first real run (v1) stopped at 73.8 m/s on a too-strict top-speed rule with a slow ramp; v2 replaces both.

## Results
- `rocketlimit.txt`: stop reason, speed at stop, peak speed and fastest clean speed. Clean means every speed band up to it had nose error < 10°, spin < 0.3 rad/s, < 10 blocks off track and < 10 blocks off height. It also has the measured gimbal map, the end position, and a table per speed band (10 m/s bands to 100, then 25) with max error, spin, off-track, altitude, angle of attack and gimbal use. Rewritten at every band, so it survives a crash.
- `rocketlimit.log`: rows every 0.25 s (0.5 s after 2 min), every tick during LIFT/ID, plus the last 4 s at full rate (`FAST` rows) when the test ends. It stops writing if the disk gets low. The previous run's files are kept as `.old`.

## Simulator (`simrocket.lua`, new `SLEW=` gimbal rate limit)
- All cases land after the test:
  - nominal: top speed 767;
  - LAG 0–6;
  - SLEW 2.1/s;
  - rotated assembly PROT ±1;
  - mass/inertia 4/8 (reaches 1000) and 7/25 (599);
  - DRAG 0.01 (reaches 1000).
- With aero torque KAERO ±0.03 (TQMAX 1.2): +0.03 reaches top speed, −0.03 stops as UNSTABLE/OFF TRACK at 120–133 m/s.
- With KAERO ±0.1 (TQMAX 4) it stops at 12–15 m/s. +0.1 is beyond steering authority and comes down on its side, with the engine cut on the ground.
- These are simulator numbers; the real limit is what this test measures.
