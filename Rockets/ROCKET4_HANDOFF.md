# Rocket4 Handoff (CC:Tweaked + Create Aeronautics / Sable, Minecraft)

Minecraft-only project: a 4-thruster cargo rocket flown by a CC:Tweaked computer. It delivers items (some spoil) from a pad to a rope net with a Create Aeronautics docking block that zeroes velocity on contact. Goal: fast, accurate, stable delivery.

Files delivered alongside this doc:
- `Rocket4.lua`: current flight program (about 1400 lines). Main deliverable.
- `VectorMap.lua`: thruster mapping test (already run; results below).
- `sim4.lua`: offline physics simulator used to test Rocket4 (see Simulator section).

## 1. Code style rules (user requirements)
- Lua for CC:Tweaked (CraftOS 1.9). No comments, no `print()`. Clean code only.
- Deliver scripts as downloadable files.
- All tunables in one `local C = { ... }` table. Keep top-level locals low: Lua has a limit of 200 locals and 60 upvalues per function. The current max is 28 upvalues; check with `luac5.1 -l`.
- Respond in English. Plain-text final outputs (captions/messages) without markdown.
- Do not build web artifacts unless asked.

## 2. Rocket hardware
- 2x2 footprint, about 18 blocks tall. Computer in one corner.
- 4 thrusters 2 blocks below the computer; nose tip 15 blocks above the computer.
- 13-block cargo silo, docking connector, extra sails. Mass changes with cargo.
- Peripherals on a wired modem: `liquid_vector_thruster_0..3`, plus fluid vessel `createpropulsion:platinum_fluid_vessel_block_entity_0`. Another vessel sits on the computer `front`. It is NOT on the modem network, so thrusters can't `pullFluid` from it; it would need its own modem.
- Fuel: lava. Each thruster has a 1000 mB internal buffer. `pullFluid(tankName)` pulls from a vessel on the same network.
- Thruster methods: getPower, getTargetVectorX/Y, getThrust, getVectorX/Y, pullFluid, pushFluid, setPower, setPowerNormalized, setThrust, setThrustNormalized, setVector(gx, gy), setVectorX/Y, tanks.
- Physical layout (user-stated, confirmed by measurement). Engine index E1..E4 is set by `C.ENGINE_ORDER` (thruster numbers in E1..E4 order; falls back to sorted-name order if the numbers don't match). Current wiring, `ENGINE_ORDER = { 2, 3, 1, 0 }`:
  - E1 `thruster_2` = front (body +Z, -X)
  - E2 `thruster_3` = front-right (+Z, +X)
  - E3 `thruster_1` = under the computer (-Z, -X)
  - E4 `thruster_0` = right (-Z, +X)
  - If the modems are reconnected and numbers change, update `ENGINE_ORDER` (the roll pattern depends on it). The simulator mirrors this wiring in `sim4.lua` (`wrap`).
  - Body frame: y = nose axis, +Z = front, +X = right. Engines are about 0.5 blocks from the CoM horizontally.
- Pad: CoM at y ≈ -53.3 when resting. Pad world pose was about (-331.65, -53.36, 44.86) in an early test.
- The rope net seems to be about Y -51. In a real NET flight the rocket came to rest with CoM at -46.3, i.e. 10 blocks above the -61 the user typed. Touchdown detection handles a wrong destination Y.

## 3. Sable / CC:Tweaked API knowledge
- `sublevel.getLogicalPose()` returns `{position, orientation}`. The quaternion arrives in varying shapes (w,x,y,z / a,v / a,b,c,d / array), so use the `quat()` helper in the code.
- `sublevel.getLinearVelocity()` is world frame. `sublevel.getAngularVelocity()` is already body frame.
- `sublevel.getMass()`: 64.06 empty. `sublevel.getInertiaTensor()` returns a 3x3 table with `rows`/`columns` fields: pitch/yaw ≈ 1144.4, roll (body Y) ≈ 61.5.
- `sublevel.isInPlotGrid()` tells whether the computer is on a sublevel.
- **Every peripheral or sublevel call costs one game tick (50 ms) when called sequentially.** Batch everything per loop in one `parallel.waitForAll`: 8 thruster calls (setThrustNormalized + setVector per engine) plus the 3 sublevel reads. Loop time ≈ 50 ms. This is `E.io()` in Rocket4.
- `peripheral.hasType(n, "liquid_vector_thruster")` finds engines. Thrusters ALSO have type `fluid_storage`, so exclude them when listing tanks.
- **Throttle is quantized.** `getThrust()` returned 1, 3, 4 at throttle 0.101, 0.202, 0.286, which suggests floor(throttle × 15) steps of about 0.067. Vertical acceleration was flat within bands (0.298–0.303, 0.333–0.342) with jumps. Anything below one step (0.0667) is zero thrust.
- `os.clock()` advances in ticks. Use `os.epoch("utc")` for ms timing.
- `textutils.serialize(t, {compact = true})` works.
- A locked docking connector holds the rocket down completely: full throttle, zero motion. The user once lost a test to this.

## 4. Measured physics (VectorMap test + flight logs)
- Thrust: about 31.6 m/s² per unit throttle, total, at mass 64 (THRUST_FORCE ≈ 2025). Hover throttle ≈ 0.335. Liftoff detection at about 0.40 because of ramp lag.
- Gimbal: first-order lag, τ ≈ 0.23 s (t63 0.23 s, t90 0.53 s). Same on every engine, both axes.
- Collective gimbal (all 4), per unit at hover throttle:
  - X+ gives ang_z +1.65 rad/s² and body lateral acc_x +6.9 m/s²
  - Y+ gives ang_x -1.64 rad/s² and acc_z +6.8 m/s²
  - The lateral force points OPPOSITE to where the resulting tilt will push the rocket: the rocket is first shoved the wrong way when it starts moving.
  - Thrust acts about 4.3 blocks below the CoM, which gives NOSE_OFFSET 12.7 and TAIL_OFFSET 4.8 (estimated).
- Roll pattern (per engine gimbal = collective + sx_i·r / sy_i·r, sx = {1,1,-1,-1}, sy = {1,-1,1,-1}): 4.57 rad/s² per unit r. Pure roll with negligible pitch coupling.
- Single-engine gimbal: about 0.4 rad/s² pitch plus about 0.55 rad/s² roll, so single-engine steering couples roll.
- Differential throttle: only about 0.25 rad/s² per unit throttle per engine, and throttle is quantized. Useless for steering; gimbals do everything.
- Sails (aero torque): tip the nose toward the direction of motion. About 0.03 rad/s² per m/s of body-lateral speed, measured consistent at 3 and 25 m/s. At high speed the evidence suggests aero forces saturate. Nose-first flight is weathervane-stable; broadside or tail-first flight is destabilizing.
- Lateral (broadside) drag is large at low speed. At 30° tilt the rocket is drag-limited to about 9–13 m/s horizontally. Nose-first drag is about 0.12/s linear, giving a top speed of about 245 m/s at full throttle, flat.
- Tail-first upright descent above about 12 m/s gets unstable (the sails want to flip it). Nose-down falling has very weak lateral control, so nose-down drops drift 7–95 blocks.

## 5. Rocket4.lua architecture
Modes: WATER (lands nose-first in a water pool, curved-approach steering), ARC, NET. **ARC and NET now fly the same path** (fastest and most accurate in testing); ARC only differs by `ARC_AB_MIN`.

Renamed in the code and docs (older real flight logs use the old names): SPLASH → WATER, BOOST → ASCENT, AFTERBURNER → EXPRESS (constants keep the `AB_` prefix), IMPACT → TOUCHDOWN. The saved destination file is `rocket.dest` (was `rocket.target`).

Phases for ARC/NET: IGNITION → ASCENT → EXPRESS (if distance > AB_MIN/ARC_AB_MIN) → brake (EXPRESS tmode BRAKE) → TRANSFER → SINK → RELEASE. Short trips: ASCENT → TRANSFER → SINK.

- **IGNITION:** throttle ramps from 0.75 × hover, estimated from mass (`tune()`), until vy > 0.5.
- **ASCENT:** vertical climb to start + CLIMB.
- **EXPRESS** (`levelDir`):
  - Carries the climb momentum up with gentle vertical deceleration (AB_VDECEL), then locks altitude `abY` when vy ≤ 0.5.
  - Altitude hold: desired vertical acceleration = KALT·e − KVY·vy. An integrator adjusts pitch angle from the *measured* vertical acceleration (filtered dv/dt), so it doesn't need a lift model.
  - Pitch is rate-limited to 15°/s and clamped between -10° and +55°. Lateral heading correction toward the destination.
  - Throttle ramps to AB_MAX_THR (1.0) up to AB_VMAX (500).
- **BRAKE:** velCmd tracks the speed profile sqrt(NET_VMAX² + 2·AB_BRAKE·(dh − handoff)) toward P (climb height), with tilt limited to NET_TILT (30°). If drag slows the rocket below 0.5× the profile while still more than 400 blocks out (AB_REARM), it returns to EXPRESS instead of crawling.
- **TRANSFER:** velCmd toward P at up to NET_VMAX (30). Horizontal gain is gentler near the destination (KVEL_MIN) because of gimbal lag. SINK starts at dh < 4 with speed < 4.
- **SINK:** vertical descent profile (max NET_VDESC 12, NET_SLOW 15 blocks braking zone, NET_SINK 2 m/s at the end, tilt ≤ 6). Release when the tail reaches destination + NET_CLEAR, or on touchdown detection (vy ≈ 0 for 1 s within NET_SLOW).
- **Steering (`steer`):**
  - PD + integral (KP 1.5, KD 1.6, ATT_KI 0.8, window 35°). Gains are softened at high throttle (ATT_REF_THR/thr) and scaled by the inertia ratio.
  - Sail feedforward: gimbal term = K·body-lateral velocity / authority. Only up to 25 m/s lateral, and scaled by how sideways the flow is (nose-first flow ⇒ no feedforward; tail-first ⇒ full).
  - A separate SINK_FIX PID is used in SINK.
- **Roll (`rollCmd`):** rate damping (KD 0.4) + heading hold (KP 0.3) when near upright. Gimbal priority goes to pitch/yaw.
- **Command smoothing:** turnToward at CMD_RATE 20°/s for TRANSFER, SINK, EXPRESS and recovery.
- **Steering-thrust floor:** if the nose lags its command by > 20° at > 15 m/s, throttle is raised to ≥ 0.45 (not while climbing upright).
- **Safety (`safety`):**
  - Upright phases (IGNITION/ASCENT/TRANSFER/SINK): tilt > 45° starts RECOVER.
  - Other phases: attitude error > 60° and not closing for 1 s, or spin > 2.5 rad/s, starts RECOVER.
  - Engine CUT only when tilt > 60° AND within 20 blocks above the lower of pad/destination (the ground-slide failure).
- **RECOVER (`recoverStep`):** velCmd to hold position and altitude with ≤ 15° tilt. Throttle ≥ 0.9·hover only when not climbing; capped at 0.8·hover when climbing (vy > 3). Exits after 1 s upright, or after 8 s if tilt < 30°.
- **Ceiling:** if the predicted apex exceeds max(pad, destination) + CEILING (700), throttle is capped at 0.1.
- **Keys:** Backspace cuts engines. Ctrl+T or any crash also cuts engines (top-level pcall).
- **Logs:**
  - `flight.log` columns: `t phase thr speed alt dist dh err_deg gx gy spin tilt roll wy loop_ms`. RECOVER/RESUME/CUT lines are written inline.
  - `rocket.log`: one line per flight with miss distance and time.
- **REMOTE:** optional table from startup.lua (Mission Control at celestial-cc.lovable.app): mission, report, plan, result, state, launchIn.
- **Planner (`predict`):** point-mass simulation with sail lift/drag (AB_LIFT 0.4), shown on screen and sent to Mission Control.

## 6. Simulator (`sim4.lua`)
Run: `MODE=ARC TX=7000 TZ=0 TMAX=300 lua5.1 sim4.lua Rocket4.lua`. It loads the program with a REMOTE mission, stubs CC APIs, and runs 50 ms physics per peripheral batch.

Outputs:
- `sim_flight.log` (the program's log)
- `sim_trace.txt` (t, pos, vel, nose every 0.25 s)
- a summary line: miss_xz, maxY, end time
- runtime errors printed as `TERMERR`

Model (all measured values above):
- per-engine gimbal lag τ 0.23 s
- quantized throttle (floor ×15)
- 31.6 m/s² total thrust
- pitch 0.41/engine and roll 0.55/0.52 per unit
- gimbal side force 1.75/1.8 per engine per unit × thrust ratio
- sail torque K·lateral velocity, capped TQMAX (1.2 rad/s²)
- lateral drag KLAT·v_lat capped LATMAX (12 m/s²); axial drag 0.12
- gravity 10.5
- net floor at destination (TY -58, ±8 blocks), pad floor -53.3

Env overrides: KAERO, KLAT, LATMAX, TQMAX, CAX, TX, TZ, MODE, TMAX.

Calibration check: the simulator reproduced the old EXPRESS wobble (tilt 30–87°), the SPIN crash of the lift-model version, the old NET hover oscillation, and the old TRANSFER crawl. Not exact at 240 m/s.

Test matrix (last results): 16 ARC flights (200–12,000 blocks, KAERO 0.015–0.045, KLAT 0.7–1.0, LATMAX 16–25, TQMAX 1.8) all land. 15 were within 1.4 blocks, the worst 2.8. Times: 400 blocks ≈ 75 s, 7,000 ≈ 111 s, 12,000 ≈ 130 s. NET: 0.2–0.4 blocks. WATER: 12–56 blocks (not tuned this session).

Always re-run the NET + ARC matrix after edits. The regression test caught a real bug: an empty `if g.phase == "SINK" then elseif ...` branch.

## 7. Real flight history (rocket.log)
- NET 1000: CUT TILT at 250 blocks. Sails tipped the rocket over at 40° tilt, and the P-only attitude loop allowed it. This led to the integral term, RECOVER, and the ground-only cut.
- NET -300: landed 1.9 blocks off after 140 s of hovering and oscillating (position loop too fast versus gimbal lag). The user slowed it with a creative staff.
- NET 500: 1.0 blocks, perfect, after gentler near-destination gains and touchdown detection.
- ARC 7000 (old ARC): 0.5 blocks but 239 s. Braked about 2,800 blocks out, then crawled 1,170 blocks at 8.8 m/s.
- AB_VMAX raised to 500 by the user. The rocket reached 245 m/s.
- Pop-up ARC attempt: RECOVER SPIN at 188 m/s, then recovery forced 0.93 throttle nose-up and climbed to Y 6109 (aborted). Two causes: the high-speed sail feedforward overcompensated, and recovery added climbing thrust. Both are fixed.
- EXPRESS wobble: tilt 33–110° at 216–245 m/s, the altitude loop limit-cycling against slow attitude. Rewritten.

## 8. Design lessons (don't repeat)
- Cutting engines at altitude = guaranteed crash. Only cut near the ground.
- Outer loops (position/velocity) must be 3–5× slower than attitude (about 2 rad/s with 0.23 s gimbal lag).
- Never throttle down to minimum while the nose must rotate against the sails: steering authority ∝ thrust.
- Pop-up/tower ARC: converting 200+ m/s into height means a 2 km+ climb. Rejected.
- Glide at height then descend: accurate but slower, because upright descent is limited to 12 m/s. Rejected.
- Nose-down final drop: drifts 7–95 blocks. Rejected.
- Feedforward outside the measured speed range is dangerous.
- `print`-free debugging: use flight.log and the simulator.

## 9. Open items / next steps
1. Real test of the new build: ARC 3,000–7,000 blocks, then check flight.log (RECOVER lines should end in RESUME).
2. EXPRESS at 240 m/s is the least certain part. If it wobbles, lower AB_KA or AB_PITCH_RATE, or cap AB_VMAX.
3. Verify the net Y and TAIL_OFFSET/NOSE_OFFSET with F3.
4. Cargo mass: `tune()` rescales thrust and gains from getMass/getInertiaTensor. Untested with a full silo.
5. The `front` vessel isn't feeding the engines (needs a modem).
6. WATER mode wasn't retuned for the new rocket.
7. Mission Control supports only one global program per /settings. ALTO (drop from a carrier ship) mode is planned, not started.
8. User preference: a rocket that looks like an arc and is fast. The user accepted that ARC now shares NET's route.

## 10. Key constants (current values)
KP 1.5, KD 1.6, ATT_KI 0.8, ATT_REF_THR 0.4, ROLL_KP 0.3, ROLL_KD 0.4, AERO_K 0.03, AERO_VMAX 25, THRUST_FORCE 2025, NOSE_OFFSET 12.7, TAIL_OFFSET 4.8, NET_VMAX 30, NET_TILT 30, NET_VDESC 12, NET_SLOW 15, NET_SINK 2, NET_SINK_TILT 6, KVEL_MIN 0.5, NET_POS_GAIN 0.2, AB_VMAX 500, AB_KALT 0.05, AB_KVY 0.4, AB_KA 1.5, AB_PITCH_RATE 15, AB_PITCH_UP 55, AB_PITCH_DOWN 10, AB_BRAKE 7, AB_REARM_D 400, AB_REARM_K 0.5, AB_MIN_DIST 1000, ARC_AB_MIN 1000, REC_TILT 45, CUT_TILT 60, CUT_LOW 20, REC_MAX_T 8, AUTH_ERR 20, AUTH_THR 0.45, CEILING 700, CMD_RATE 20.
