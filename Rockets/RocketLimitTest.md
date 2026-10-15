# RocketLimitTest: top speed of the single-thruster rocket

Copy `RocketLimitTest.lua` to the rocket's computer. Thruster on `bottom` (any side works), rocket standing upright on flat ground, creative lava attached (it pulls from every attached fluid tank every 0.5 s).

```
RocketLimitTest [height 100-200] [north|south|east|west|F3 yaw]
RocketLimitTest 150 east
```
Default: 150 blocks, north. Backspace or Ctrl+T at any time ends the test and lands.

## What it does
1. LIFT: ramps the throttle until it lifts off.
2. ID (about 2 s, a few blocks up): small gimbal kicks, measures how the gimbal turns the rocket (map). This makes it work whichever way the rocket was assembled.
3. CLIMB at 6 m/s straight up to the height, holding position.
4. HOLD 5 s steady (must be calm, else it stops).
5. RUN: accelerates along the heading, slowly at first (1.5 m/s² up to 100 m/s, then 1.5 % of speed per s, max 8 m/s²), holding height and the track line with the thrust direction.
6. Test ends on the first of:
   - 1000 m/s;
   - more than 40 blocks off the line or ±40 blocks off the height for 1 s;
   - nose more than 25° off for 0.5 s (45° at once);
   - spinning more than 1.5 rad/s for 0.5 s (3 at once);
   - steering maxed out for 2 s;
   - top speed reached (full throttle and less than 3 m/s gained in 10 s);
   - Backspace;
   - 15 minutes.
7. RECOVER: slows down with the nose kept close to the flight path at high speed (flips back only below about 40–150 m/s), then DESCEND at 3 m/s straight down and cuts the engine once it sits still on the ground (anywhere). It never cuts the engine in the air.

Expect a long trip. In the simulator, 0 → 767 m/s took ~4 min and ~75 km, and braking took another ~7 km. Keep the chunks loaded along the route.

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
