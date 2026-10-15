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
- Physical layout (user-stated, confirmed by measurement). Engine index = sorted-name order:
  - E1 `thruster_0` = front (body +Z, -X)
  - E2 `thruster_1` = front-right (+Z, +X)
  - E3 `thruster_2` = under the computer (-Z, -X)
  - E4 `thruster_3` = right (-Z, +X)
  - Body frame: y = nose axis, +Z = front, +X = right. Engines are about 0.5 blocks from the CoM horizontally.
- Pad: CoM at y ≈ -53.3 when resting. Pad world pose was about (-331.65, -53.36, 44.86) in an early test.
- The rope net seems to be about Y -51. In a real NET flight the rocket came to rest with CoM at -46.3, i.e. 10 blocks above the -61 the user typed. Touchdown detection handles a wrong target Y.

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
Modes: SPLASH (nose-first impact into water, PN guidance), ARC, NET. **ARC and NET now fly the same path** (fastest and most accurate in testing); ARC only differs by `ARC_AB_MIN`.

Phases for ARC/NET: IGNITION → BOOST → AFTERBURNER (if distance > AB_MIN/ARC_AB_MIN) → brake (AFTERBURNER tmode BRAKE) → TRANSFER → SINK → RELEASE. Short trips: BOOST → TRANSFER → SINK.

- **IGNITION:** throttle ramps from 0.75 × hover, estimated from mass (`tune()`), until vy > 0.5.
- **BOOST:** vertical climb to start + CLIMB.
- **AFTERBURNER** (`levelDir`):
  - Carries the climb momentum up with gentle vertical deceleration (AB_VDECEL), then locks altitude `abY` when vy ≤ 0.5.
  - Altitude hold: desired vertical acceleration = KALT·e − KVY·vy. An integrator adjusts angle of attack from the *measured* vertical acceleration (filtered dv/dt), so it doesn't need a lift model.
  - Pitch is rate-limited to 15°/s and clamped between -10° and +55°. Lateral heading correction toward the target.
  - Throttle ramps to AB_MAX_THR (1.0) up to AB_VMAX (500).
- **BRAKE:** velCmd tracks the speed profile sqrt(NET_VMAX² + 2·AB_BRAKE·(dh − handoff)) toward P (climb height), with tilt limited to NET_TILT (30°). If drag slows the rocket below 0.5× the profile while still more than 400 blocks out (AB_REARM), it returns to AFTERBURNER instead of crawling.
- **TRANSFER:** velCmd toward P at up to NET_VMAX (30). Horizontal gain is gentler near the target (KVEL_MIN) because of gimbal lag. SINK starts at dh < 4 with speed < 4.
- **SINK:** vertical descent profile (max NET_VDESC 12, NET_SLOW 15 blocks braking zone, NET_SINK 2 m/s at the end, tilt ≤ 6). Release when the tail reaches target + NET_CLEAR, or on touchdown detection (vy ≈ 0 for 1 s within NET_SLOW).
- **Steering (`steer`):**
  - PD + integral (KP 1.5, KD 1.6, ATT_KI 0.8, window 35°). Gains are softened at high throttle (ATT_REF_THR/thr) and scaled by the inertia ratio.
  - Sail feedforward: gimbal term = K·body-lateral velocity / authority. Only up to 25 m/s lateral, and scaled by how sideways the flow is (nose-first flow ⇒ no feedforward; tail-first ⇒ full).
  - A separate SINK_FIX PID is used in SINK.
- **Roll (`rollCmd`):** rate damping (KD 0.4) + heading hold (KP 0.3) when near upright. Gimbal priority goes to pitch/yaw.
- **Command smoothing:** turnToward at CMD_RATE 20°/s for TRANSFER, SINK, AFTERBURNER and recovery.
- **Steering-thrust floor:** if the nose lags its command by > 20° at > 15 m/s, throttle is raised to ≥ 0.45 (not while climbing upright).
- **Safety (`safety`):**
  - Upright phases (IGNITION/BOOST/TRANSFER/SINK): tilt > 45° starts RECOVER.
  - Other phases: attitude error > 60° and not closing for 1 s, or spin > 2.5 rad/s, starts RECOVER.
  - Engine CUT only when tilt > 60° AND within 20 blocks above the lower of pad/target (the ground-slide failure).
- **RECOVER (`recoverStep`):** velCmd to hold position and altitude with ≤ 15° tilt. Throttle ≥ 0.9·hover only when not climbing; capped at 0.8·hover when climbing (vy > 3). Exits after 1 s upright, or after 8 s if tilt < 30°.
- **Ceiling:** if the predicted apex exceeds max(pad, target) + CEILING (700), throttle is capped at 0.1.
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
- net floor at target (TY -58, ±8 blocks), pad floor -53.3

Env overrides: KAERO, KLAT, LATMAX, TQMAX, CAX, TX, TZ, MODE, TMAX. Airframe: MASS (default 64.06), IPITCH (1144.4), IROLL (61.5) rescale thrust, gimbal, sail and drag effects and feed getMass/getInertiaTensor. Drag is treated as a force (lighter = more drag deceleration); set DRAGACC=1 to keep drag as a fixed deceleration instead.

Calibration check: the simulator reproduced the old AFTERBURNER wobble (tilt 30–87°), the SPIN crash of the lift-model version, the old NET hover oscillation, and the old TRANSFER crawl. Not exact at 240 m/s.

Test matrix (last results): 16 ARC flights (200–12,000 blocks, KAERO 0.015–0.045, KLAT 0.7–1.0, LATMAX 16–25, TQMAX 1.8) all land. 15 were within 1.4 blocks, the worst 2.8. Times: 400 blocks ≈ 75 s, 7,000 ≈ 111 s, 12,000 ≈ 130 s. NET: 0.2–0.4 blocks. SPLASH: 12–56 blocks (not tuned this session).

Always re-run the NET + ARC matrix after edits. The regression test caught a real bug: an empty `if g.phase == "SINK" then elseif ...` branch.

## 7. Real flight history (rocket.log)
- NET 1000: CUT TILT at 250 blocks. Sails tipped the rocket over at 40° tilt, and the P-only attitude loop allowed it. This led to the integral term, RECOVER, and the ground-only cut.
- NET -300: landed 1.9 blocks off after 140 s of hovering and oscillating (position loop too fast versus gimbal lag). The user slowed it with a creative staff.
- NET 500: 1.0 blocks, perfect, after gentler near-target gains and touchdown detection.
- ARC 7000 (old ARC): 0.5 blocks but 239 s. Braked about 2,800 blocks out, then crawled 1,170 blocks at 8.8 m/s.
- AB_VMAX raised to 500 by the user. The rocket reached 245 m/s.
- Pop-up ARC attempt: RECOVER SPIN at 188 m/s, then recovery forced 0.93 throttle nose-up and climbed to Y 6109 (aborted). Two causes: the high-speed sail feedforward overcompensated, and recovery added climbing thrust. Both are fixed.
- AFTERBURNER wobble: tilt 33–110° at 216–245 m/s, the altitude loop limit-cycling against slow attitude. Rewritten.

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
2. AFTERBURNER at 240 m/s is the least certain part. If it wobbles, lower AB_KA or AB_PITCH_RATE, or cap AB_VMAX.
3. Verify the net Y and TAIL_OFFSET/NOSE_OFFSET with F3.
4. Cargo mass: `tune()` rescales thrust and gains from getMass/getInertiaTensor. Untested with a full silo.
5. The `front` vessel isn't feeding the engines (needs a modem).
6. SPLASH mode wasn't retuned for the new rocket.
7. Mission Control supports only one global program per /settings. ALTO (drop from a carrier ship) mode is planned, not started.
8. User preference: a rocket that looks like an arc and is fast. The user accepted that ARC now shares NET's route.

## 10. Key constants (current values)
KP 1.5, KD 1.6, ATT_KI 0.8, ATT_REF_THR 0.4, ROLL_KP 0.3, ROLL_KD 0.4, AERO_K 0.03, AERO_VMAX 25, THRUST_FORCE 2025, NOSE_OFFSET 12.7, TAIL_OFFSET 4.8, NET_VMAX 30, NET_TILT 30, NET_VDESC 12, NET_SLOW 15, NET_SINK 2, NET_SINK_TILT 6, KVEL_MIN 0.5, NET_POS_GAIN 0.2, AB_VMAX 500, AB_KALT 0.05, AB_KVY 0.4, AB_KA 1.5, AB_PITCH_RATE 15, AB_PITCH_UP 55, AB_PITCH_DOWN 10, AB_BRAKE 7, AB_REARM_D 400, AB_REARM_K 0.5, AB_MIN_DIST 1000, ARC_AB_MIN 1000, REC_TILT 45, CUT_TILT 60, CUT_LOW 20, REC_MAX_T 8, AUTH_ERR 20, AUTH_THR 0.45, CEILING 700, CMD_RATE 20.

## 11. Light Rocket4 (new airframe, VectorMap run 3, full 18 tests)
- Shorter and lighter. Mass 29.5. Inertia pitch/yaw 112.3, roll 31.2. Hover throttle ≈ 0.19 (between quantized steps 2/15 and 3/15, so it dithers). THRUST_FORCE ≈ 2000 (same engines).
- Gimbal lag unchanged (t63 0.17–0.22 s). Engine layout and sign pattern unchanged.
- Collective per unit at hover: ALL GX ang_z +4.13, acc_x +4.7; ALL GY ang_x -3.10, acc_z +6.2; ROLL 4.12. About 2–2.5× the old pitch/yaw authority. The right-hand engines (E2, E4) pitch about 40% harder on GY: the CoM is off-center.
- One throttle step on one engine tips it ≈ 20° within 1 s, so differential throttle is strong now (unused by Rocket4).
- 10° step test: rise ≈ 1 s, then a slow 2–4° wobble and drift; ends about 3° off. Slightly underdamped.
- Server lag: freezes of 0.54–0.63 s seen (looks like teleporting in game). The sim models them with LAGP (chance per loop of a 0.6 s freeze) and SEED.
- Thrust limiter: `ACC_CAP` (32) caps the thrust acceleration the program plans with; `E.tscale` scales the real throttle in `E.set`. The old rocket (31.6) is unaffected. The light rocket then flies like the old one in its linear dynamics.
- Sim stand-in: `MASS=29.5 IPITCH=300 IROLL=39` (effective values that reproduce the measured collective responses).
- Unknowns: sail strength (KAERO) and whether drag acts as a force (default) or as a fixed deceleration (DRAGACC=1). With strong sails plus low drag, long ARC flights still fail in the sim; short NET hops land in every case. Measure both from a real NET hop's flight.log before raising ACC_CAP or flying long ARC trips.
- First real light-rocket NET hop (300 blocks): landed 0.9 off but took 400 s. It stalled at dh 5.7 (just outside NET_SINK_RADIUS 4) for 290 s, tilted 2.5° toward the target with zero speed: a steady side force of about 0.5 m/s² (likely the off-center CoM) cancelled the P-only push (KVEL_MIN·0.2·dh ≈ 0.57). Fix: a horizontal velocity integral in `velCmd` (NET_HKI 0.4, NET_HIMAX 2.0), active only in TRANSFER (not recovering) when the commanded speed is < NET_HIV (3); zeroed otherwise. Applying it in SINK as well made the old rocket sway (misses up to 6 blocks), so don't.
- The same flight: at 30° tilt TRANSFER only managed 3.5–7 m/s, which supports force-based drag (light rocket gets ~2× the drag deceleration). With force drag the thrust cap of 32 m/s² also caps nose-first speed near 120 m/s, so raising ACC_CAP (at least in AFTERBURNER) is the way to go faster.
- Real hover throttle in program units was 0.42 vs the predicted 0.328: floor quantization makes the real thrust about half a step lower than commanded. The vertical bias integrator absorbs it.
- Sim: DISTX/DISTZ add a steady world-frame side push that scales with thrust (to mimic the CoM offset).

## 12. Flip brake and full-throttle afterburner (ported from the user's Transport.lua)
- Transport.lua is the user's later fork of Rocket4 (NET-only menu, passwords, launch lock, self-delete, ENGINE_ORDER {2,3,1,0}, NET flip brake, new ARC dive, SPLASH renamed WATER). The user wants Rocket4 to keep SPLASH, ARC and NET, so only its flight upgrades were ported. Security features and ENGINE_ORDER were left out: the light rocket's VectorMap shows the standard sorted engine order, so ENGINE_ORDER would scramble it.
- Ported: FLIP (rotate nose through vertical to 160° at TF_RATE) → RETRO (full throttle against velocity, pitch from `holdPitch`) → TRANSFER when horizontal speed < RETRO_END_V. Brake start distance `brk = (BRK_A·v + BRK_B·v²)·BRK_REF/THRUST_ACC·(1+BRK_PAD_K) + BRK_PAD`. AB thrust-hold pitch (`holdPitch`), AB_VMAX 350, AB_PITCH_RATE 30. Safety grace for FLIP/RETRO, no attitude-error recovery during FLIP, recovery resumes the afterburner when far, attitude integrator reset around FLIP/RETRO.
- Changes vs Transport, from sim testing:
  - Flip and thrust hold only switch on when full thrust ≥ FLIP_ACC (45 m/s²). Light rocket 68.6: on. Old rocket 31.6: keeps the proven tilt brake (thrust hold made it dive into the ground in the sim).
  - BRK_A 2.4 (Transport 4.2) scaled by BRK_REF/THRUST_ACC. 4.2 ended the retro 440 blocks short and the light rocket crawled the rest at 7 m/s.
  - Flip thrust is an acceleration (TF_ACC 19 m/s²) instead of throttle 0.6, which on the light rocket was 41 m/s² up and climbed it to ~470.
  - `holdPitch` targets a climb rate capped at AB_HOLD_VY (8) and its bias integrates climb-rate error (AB_BIAS_KI 0.3, always on). Transport's version only integrated within 40 blocks and drifted up to 400–580 with lift.
  - AB_VDECEL stays 5 (Transport 10): 10 plus pitch rate 30 caused 15 recoveries / 90° low tilt for the old rocket with strong sails.
  - Kept Transport's `if pos.y > g.P.y then g.P.y = pos.y end` after RETRO: removing it improved accuracy in heavy-drag sims but caused up to 5 recoveries (tail-first descent while moving).
- AB_FULL: AFTERBURNER/FLIP/RETRO run with real full thrust (`thrustMode` switches C.THRUST_ACC between E.accFull and E.accCap and rescales g.throttle so the real thrust is continuous). Recovery always uses the cap. The screen and flight.log `thr` now show the real engine throttle (g.throttle·E.tscale).
- Real 6.7 km NET flight with the capped version: 198 s, miss 1.0, AB cruise only 82 m/s at 46% real throttle. Calibrated sim: MASS=29.5 IPITCH=300 IROLL=39 KAERO=0.03 CAX=0.16 (matches 79 m/s, 205 s, peak 234).
- Sim results with the port, light rocket: 6,760 blocks ≈ 113–126 s, 12,000 ≈ 146 s, misses 0.3–2.4 in normal variants; 4–14 with a steady side push or KLAT 0.8 (sink from ~280). Old rocket: all 19 regression flights land, 7,000 ≈ 106 s (was 111).
- SPLASH on the light rocket is untuned (sim misses 15–67 blocks, flies low at ~58). Full thrust in CRUISE didn't fix it. Unchanged for the old rocket.
