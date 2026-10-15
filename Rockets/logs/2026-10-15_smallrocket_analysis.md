# Single-thruster tumble analysis, 2026-10-15

Status: experimental controller improvement, not a validated replacement for real flights.
The uncapped aerodynamic stress suite still fails; do not treat a passing nominal
simulation as evidence that the real tumble is fixed.

## Evidence from the real flight

Source: `2026-10-11_smallrocket_water_tumble_flight.log`, flown version `a74618a`.
Target offset from launch: (+140.9, -2.3, +169.2), horizontal distance 220.2 blocks.

- Throttle stays at 0.22 throughout the initial instability. This is not a
  full-throttle cruise transition: the vehicle remains in ASCENT.
- At 2.30 s, speed is 11.8 m/s, error 3.1 degrees, gy +0.09, spin 0.12 rad/s.
- At 2.80 s, error shrinks to 1.2 degrees while spin increases to 0.44; gy
  reverses to -0.10. The vehicle is moving through the requested attitude.
- At 3.30 s, error grows to 15.0 degrees and gy reaches -0.52.
- At 3.80 s, error again shrinks to 1.1 degrees while spin is already 1.52;
  gy reverses to +0.79. At 4.05 s it saturates at +1.00 with 41-degree error;
  by 4.30 s the error is 94.1 degrees. This is a growing oscillation, then tumble.

The log stores only unsigned error and spin, requested gimbals, and sparse
position/speed information. It cannot distinguish delayed angular-rate feedback,
actuator lag, aerodynamic torque, or an incorrect map. No unique physical root
cause can be established from it. The current branch's mass tuning and stronger
ascent throttle were added after this flight; they must not be attributed to it.

A correction to the previous handoff: `2026-10-10_smalltest.txt` explicitly says
`GIMBAL t63 not seen, using 0.23`. That experiment did NOT measure a 0.23 s lag.
The map also came from a short identification maneuver with varying throttle,
not a clean steady-hover sweep. Its use as a map normalized at two throttle
steps is an approximation shared by the existing simulator. Real high-speed
aerodynamics and delay remain unmeasured.

## Controller changes and rationale

- Use the plane's acceleration-space PID approach, with the rocket's dominant
  measured map: ax = -1.95*gy, az = 3.37*gx - 0.94*gy. Include throttle
  quantization and inertia scaling when inverting this map. The small measured
  ax/gx coefficient (-0.10) is omitted, as in the existing simulator.
- KP=5, KD=3, KI=3 act in angular-acceleration space, rather than directly in
  gimbal space. Limit both gimbals together to preserve the requested direction.
- Damping uses angular rate projected 0.20 s ahead from a filtered derivative
  (0.15 s filter, acceleration bounded to 12 rad/s²). This adds damping before
  the delayed actuator finishes moving. It is a robustness measure, not proof
  of the real delay. No extra peripheral batch is introduced.
- Remove the ascent `aim()` velocity correction. The ascent command already
  turns at a limited rate; differentiating a velocity-dependent command in the
  rate feed-forward created another feedback path during the observed phase.
- Reset feed-forward on command discontinuities and reset controller state at
  launch. Keep the existing guidance, climb target, WATER and SHUTTLE modes.
- Restrict both stationary-flight shutdown paths to low altitude. Previously
  the `STILL` safety and CRUISE stuck detector could cut an engine at altitude.
- Move tunables into C. Preserve the first 11 flight.log columns and append
  signed rates, nose direction, velocity, command direction, actual gimbal
  getters (NA if unavailable), and actual loop milliseconds. Getter calls join
  the existing batch. `gx/gy` are next commands, not actual actuator positions.

Plane.lua, Transport.lua and Rocket4.lua are unchanged.

## Reproducible validation

From `Rockets`, with Lua 5.1 and Python 3:

```
luac5.1 -p Rocket.lua simrocket.lua
python3 tests/rocket_regression.py --suite core
python3 tests/rocket_regression.py --suite extended
python3 tests/rocket_regression.py --suite uncapped
```

The runner uses fresh output directories, a 300-second simulation limit, and
30-second wall timeout per case. It checks physical splash miss <=20 blocks,
no crash/CUT/runtime error, and a completed mission. It does not trust
`ok=true` or the program's closest-approach result alone. Failure suites return
nonzero; these are not skipped or marked expected-pass.

- Core: 30/30 pass. Both modes, LAG 0..4, KAERO -0.03/0/+0.03, TQMAX=1.2,
  translated real target geometry. Worst miss 14.8 blocks; max ASCENT error
  4.7 degrees. The previous branch controller passes 14/30 under the same
  checks and corrected simulator.
- Extended: 67/71 pass the same 20-block accuracy requirement. All 71 reach
  the modeled pool without crash/CUT/runtime errors. Covers 150..5000-block
  routes, bearings, a directly-below SHUTTLE target, mass/inertia pairs
  4/8, 4.5/12, 5.5/15.57, 7/25, and KAERO +/-0.03 and +/-0.1 with TQMAX=1.2.
  Max ASCENT error 8.6 degrees. Four accuracy failures: WATER TX=300 LAG=3
  KAERO=-0.1 (25.0 blocks); SHUTTLE TX=300 LAG=3 KAERO=-0.03 (21.0);
  SHUTTLE TX=300 LAG=4 KAERO=-0.1 (26.6); SHUTTLE TX=300 LAG=4 KAERO=-0.03
  (20.1). These demonstrate remaining late-approach error, not a clean bill
  of health for arbitrary pool sizes.
- Uncapped: 24/30 pass. Same geometry as core but no torque cap. SHUTTLE
  KAERO=+0.03 fails at every LAG; KAERO=-0.03 also fails at LAG=3. Five cases
  hit ground outside the pool; worst miss 83.5 blocks. WATER passes all 15.
  This is a real limitation of this candidate, not a reason to change the
  simulator's default torque cap or weaken the checks.
- Lua 5.1 syntax passes; all referenced C fields exist. Maximum function
  upvalues 31, locals 100 (below 60/200 limits).

Simulator reporting fixes: launch-pad contact is no longer counted as a crash;
a later ground collision is not hidden merely because the nose crossed the
target-height plane earlier. Optional CHECK_FLIGHT=1 returns nonzero for a
failed mission, hidden print/error, missing splash, ground collision or miss
above MAX_MISS (default 20). It leaves measurement-program invocation unchanged.

## Next evidence needed

The existing log cannot establish the missing physics. Before claiming this
candidate flight-ready, resolve the uncapped SHUTTLE failures and obtain a
controlled in-game measurement of actual gimbal response and signed angular
rates at 10–20 m/s climb. The appended telemetry is intended for that comparison;
retain the current log and compare requested versus actual gimbal timing and
angular acceleration. Do not describe the 0.23 s fallback as a measurement.
