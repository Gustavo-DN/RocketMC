# Single-thruster transport speed experiment

Copy **RocketLimitTest.lua** to the rocket's computer. It is standalone: it
neither loads nor changes Rocket.lua, Plane.lua or the four-thruster programs.
The vector thruster must be on `bottom`, with the rocket standing upright.
Attach the creative lava supply; the program pulls from accessible peripheral
fluid stores every 0.5 s. An externally fed engine also works. A creative tank
still changes mass/inertia: those are read at startup.

Run:

```
RocketLimitTest
```

Default: climb 150 blocks above the launch point, then fly along the initial
horizontal body +Z direction. Optional explicit world heading and climb height:

```
RocketLimitTest 90 150
```

Heading convention: 0 = +Z (south), 90 = +X (east), 180 = -Z (north),
270 = -X (west). This is NOT Minecraft's F3 yaw convention. Height must be
100–200. There is a five-second countdown. Backspace or Ctrl+T cancels on the
pad; in flight it ends the test and requests powered recovery.

## Experiment

The climb targets 10 m/s vertical speed and slows near the selected height.
During pitch-over, thrust is adjusted to avoid the old large climb overshoot.
The run follows a fixed horizontal line through the launch point and holds the
selected altitude. It does not chase a destination or turn back.

After leveling, start one usable throttle step above hover (normally 3/15).
Add only **one 1/15 throttle step per ten seconds**, and only after a clean
five-second tracking window. This is a slow *throttle* ramp, not a guaranteed
constant acceleration: measured speed/acceleration still depend on drag, mass
and the game's thrust quantization. No automatic gain retuning occurs during
the run, so the resulting log can be compared between builds.

Stop the speed experiment on:

- 1,000 m/s forward speed along the original track (not speed gained by falling).
- More than 50 blocks sideways from that line, more than 40 blocks above/below
  cruise height, path heading error above 15 degrees, commanded-attitude error
  above 35 degrees, or angular speed above 2 rad/s, sustained for 0.5 s.
- Angular speed above 4 rad/s immediately.
- No clean tracking window within 45 s at a throttle step, preventing indefinite
  operation at a marginally stable speed.
- Full-throttle plateau: speed range below 3 m/s over 30 s, sustained another
  10 s. This reports a terminal speed below 1,000 if the model/game has one.
- Setup timeout (120 s per setup phase), overall test timeout (600 s), manual
  stop, or an empty engine for one second. Empty fuel explicitly invalidates
  the speed-limit result.

All thresholds are at the top of the Lua file in C. A clean window means cross
track <25 blocks, altitude error <15, attitude error <10 degrees, spin <0.6
rad/s and heading error <15 degrees. If no such window was completed, the
summary reports zero instead of inventing a stable maximum.

## End behavior

The summary is saved immediately when the experiment stops. The engine stays
powered while the controller turns toward an upright braking attitude, slows
horizontally, then descends at 3 m/s. It shuts down only after low-altitude,
low-speed contact. Recovery does not return to the launch point.

Use clear, flat terrain at roughly the launch elevation. There is no terrain,
obstacle, chunk-loading or landing-site sensor. Recovery is an attempt, not a
guarantee after a real tumble or at unmeasured aerodynamic loads. If a control
or telemetry exception occurs in the air, it saves the error and leaves a hover
throttle command with centered gimbal; that is NOT active recovery and requires
manual takeover. Infinite lava does not remove these limits.

## Send back these files

- **rocketlimit.txt**: stop reason, speed at failure/ceiling, peak forward speed,
  fastest stable five-second mean and endpoint, time, throttle steps, mass,
  inertia scale and fuel status. Recovery speeds are excluded.
- **rocketlimit.log**: approximately 20 Hz telemetry including full position,
  velocity, cross track, altitude, nose/command vectors, signed angular rates,
  requested/actual gimbals, loop milliseconds and fuel amount.

Old log/summary files are renamed with a millisecond timestamp before launch.
Commands in each row are for the next batch; the attitude-error field compares
the sampled nose against the preceding command. No extra sequential peripheral
waits are added by fuel transfer or gimbal sampling.

This measures the usable speed of **this craft with this controller and these
tracking tolerances**. It cannot establish an intrinsic hardware limit or prove
that a different controller could not go faster. The real gimbal lag remains
unmeasured (SmallTest's 0.23 s was a fallback).

## Validation (2026-10-15)

```
luac5.1 -p RocketLimitTest.lua
python3 tests/limit_test.py
```

17/17 simulator experiments pass: LAG 0..4 with KAERO +/-0.03 (TQMAX=1.2),
nominal physics, low-drag 1,000 m/s run, stronger destabilizing torque, mass/
inertia 4/8 and 7/25, explicit 90-degree/200-block launch, and injected empty-fuel
telemetry. Each terminates with a recorded cause and completes powered recovery
and upright landing in this model; passing means the experiment behaves
correctly, not that every craft reaches top speed. Tests check 50 ms batches,
actual gimbal data, minimum throttle dwell, altitude band, failure recording,
1,000 m/s cutoff and no zero-throttle recovery commands.

Nominal model: speed run at about 144–156 blocks altitude; cross-track stop near
765 m/s. Positive-aero cases with delays 0–2 reach full-throttle plateaus near
769 m/s. Low drag reaches the ceiling at 1,000.08 m/s. High-delay cases stop
at stability/setup limits rather than silently claiming a stable maximum.
These are simulator observations only; the real rocket's limit is unknown.
