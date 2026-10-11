local C = {
  LOG = "takeofftest.log",
  OUT = "takeofftest.txt",
  ENGINE_TYPE = "liquid_vector_thruster",
  GRAV = 10.5,
  FWD = { x = 0, y = 0, z = 1 },
  MAP_PITCH = -4.37,
  MAP_YAW = -3.88,
  KP = 4,
  KD = 3,
  KI = 3,
  IMAX = 1.0,
  ERR_MAX = 0.6,
  GMAX = 0.9,
  COUNTDOWN = 3,
  ROLL_STEPS = 4,
  ROLL_STEP_T = 1.0,
  ROLL_MAX_T = 30,
  ROLL_MAX_D = 400,
  ROLL_PITCH = 0,
  VR = 40,
  ROTATE_PITCH = 10,
  ROTATE_RATE = 5,
  LIFTOFF_H = 1.0,
  CLIMB_STEPS = 3,
  CLIMB_PITCH = 12,
  CRUISE_H = 30,
  V_LEVEL = 60,
  K_STEPS = 0.1,
  STEPS_MAX = 6,
  LEVEL_T = 5,
  SLOW_STEPS = 1,
  SLOW_T = 10,
  SLOW_PITCH_MAX = 20,
  SLOW_V_MIN = 30,
  TURN_RATE = 8,
  TURN_ANGLE = 180,
  TURN_MAX_T = 40,
  AFTER_TURN_T = 4,
  K_HV = 0.3,
  VY_MAX = 8,
  K_PVY = 0.012,
  K_IVY = 0.004,
  LEVEL_LEAD_T = 2.5,
  TRIM0 = 8,
  TRIM_MIN = -5,
  TRIM_MAX = 18,
  PITCH_MAX = 25,
  PITCH_RATE = 15,
  APP_SLOPE = 4,
  APP_STEPS = 1,
  FLARE_H = 4,
  FLARE_PITCH = 6,
  TOUCH_H = 1.0,
  TOUCH_VY = 1.0,
  STOP_V = 0.5,
  STOP_T = 1.5,
  CUT_SPIN = 4,
  MAX_T = 180,
}

local atan2 = math.atan2 or math.atan
local function v3(a) return { x = a.x or a[1], y = a.y or a[2], z = a.z or a[3] } end
local function add(a, b) return { x = a.x + b.x, y = a.y + b.y, z = a.z + b.z } end
local function sub(a, b) return { x = a.x - b.x, y = a.y - b.y, z = a.z - b.z } end
local function mul(a, s) return { x = a.x * s, y = a.y * s, z = a.z * s } end
local function dot(a, b) return a.x * b.x + a.y * b.y + a.z * b.z end
local function cross(a, b) return { x = a.y * b.z - a.z * b.y, y = a.z * b.x - a.x * b.z, z = a.x * b.y - a.y * b.x } end
local function len(a) return math.sqrt(dot(a, a)) end
local function clamp(x, lo, hi) return math.max(lo, math.min(hi, x)) end
local function quat(q)
  if q.w then return q.w, { x = q.x, y = q.y, z = q.z } end
  if q.a and q.v then return q.a, v3(q.v) end
  if q.a then return q.a, { x = q.b, y = q.c, z = q.d } end
  return q[4], { x = q[1], y = q[2], z = q[3] }
end
local function rotate(w, u, v)
  local t = mul(cross(u, v), 2)
  return add(add(v, mul(t, w)), cross(u, t))
end
local function toBody(w, u, v) return rotate(w, mul(u, -1), v) end
local function f3(v) return string.format("%+.3f %+.3f %+.3f", v.x, v.y, v.z) end

local out = {}
local function note(s)
  out[#out + 1] = s
  print(s)
end
local function save()
  local f = fs.open(C.OUT, "w")
  f.write(table.concat(out, "\n") .. "\n")
  f.close()
end

local function solve(A, b, n)
  local m = {}
  for i = 1, n do
    m[i] = {}
    for j = 1, n do m[i][j] = A[i][j] end
    m[i][n + 1] = b[i]
  end
  for c = 1, n do
    local p = c
    for r = c + 1, n do if math.abs(m[r][c]) > math.abs(m[p][c]) then p = r end end
    m[c], m[p] = m[p], m[c]
    if math.abs(m[c][c]) < 1e-12 then return nil end
    for r = 1, n do
      if r ~= c then
        local f = m[r][c] / m[c][c]
        for k = c, n + 1 do m[r][k] = m[r][k] - f * m[c][k] end
      end
    end
  end
  local x = {}
  for i = 1, n do x[i] = m[i][n + 1] / m[i][i] end
  return x
end

local name
for _, n in ipairs(peripheral.getNames()) do
  if peripheral.hasType(n, C.ENGINE_TYPE) then name = name or n end
end
term.clear()
term.setCursorPos(1, 1)
note("TAKEOFF TEST " .. tostring(os.date and os.date() or ""))
if not name then
  note("No thruster found.")
  save()
  return
end
if not sublevel then
  note("No sublevel API: not on a Sable sub-level.")
  save()
  return
end
local eng = peripheral.wrap(name)
note("THRUSTER " .. name)
pcall(function() note("INERTIA " .. textutils.serialize(sublevel.getInertiaTensor(), { compact = true })) end)

local cmd = { 0, 0, 0 }
local function tick(send, withMass)
  local p, v, o, m
  local fns = {
    function() p = sublevel.getLogicalPose() end,
    function() v = sublevel.getLinearVelocity() end,
    function() o = sublevel.getAngularVelocity() end,
  }
  if send then
    fns[#fns + 1] = function() eng.setThrustNormalized(cmd[1]) end
    fns[#fns + 1] = function() eng.setVector(cmd[2], cmd[3]) end
  end
  if withMass then fns[#fns + 1] = function() m = sublevel.getMass() end end
  parallel.waitForAll(table.unpack(fns))
  local w, u = quat(p.orientation)
  return v3(p.position), w, u, v3(v), v3(o), m
end
local function cut()
  pcall(eng.setThrustNormalized, 0)
  pcall(eng.setVector, 0, 0)
end

local R = { reason = "?" }
local flog

local I = { x = 0, y = 0 }

local function hold(w, u, om, head, pitch, steps, dt)
  local d = { x = head.x * math.cos(pitch), y = math.sin(pitch), z = head.z * math.cos(pitch) }
  local b = toBody(w, u, d)
  local s = math.sqrt(b.x * b.x + b.y * b.y)
  local ex, ey = 0, 0
  if s > 1e-9 then
    local k = math.min(atan2(s, b.z), C.ERR_MAX) / s
    ex, ey = -b.y * k, b.x * k
  elseif b.z < 0 then
    ex = C.ERR_MAX
  end
  I.x = clamp(I.x + ex * dt, -C.IMAX, C.IMAX)
  I.y = clamp(I.y + ey * dt, -C.IMAX, C.IMAX)
  local st = math.max(steps, 1)
  local ax = C.KP * ex + C.KI * I.x - C.KD * om.x
  local ay = C.KP * ey + C.KI * I.y - C.KD * om.y
  local gy = clamp(ax / (C.MAP_PITCH * st), -C.GMAX, C.GMAX)
  local gx = clamp(ay / (C.MAP_YAW * st), -C.GMAX, C.GMAX)
  return gx, gy
end

local function rotY(h, a)
  local c, s = math.cos(a), math.sin(a)
  return { x = h.x * c + h.z * s, y = 0, z = -h.x * s + h.z * c }
end

local function waitStart()
  print("")
  print("TAKEOFF TEST. Point the plane down the runway.")
  print("ENTER or redstone on top starts, BACKSPACE cancels.")
  local why
  local function rs()
    while true do
      if redstone.getInput("top") then why = "REDSTONE" return end
      sleep(0.1)
    end
  end
  local function keysIn()
    while true do
      local _, k = os.pullEvent("key")
      if k == keys.enter or k == keys.numPadEnter then why = "ENTER" return end
      if k == keys.backspace then why = false return end
    end
  end
  parallel.waitForAny(rs, keysIn)
  if not why then return false end
  for i = C.COUNTDOWN, 1, -1 do
    print("Engine on in " .. i)
    sleep(1)
  end
  return why
end

local function main()
  cut()
  local why = waitStart()
  if not why then R.reason = "cancelled" return end
  local p0, w0, u0 = tick(false)
  local f0 = rotate(w0, u0, C.FWD)
  local hl = math.sqrt(f0.x * f0.x + f0.z * f0.z)
  if hl < 0.5 then R.reason = "plane is not level (nose points up/down)" return end
  local head0 = { x = f0.x / hl, y = 0, z = f0.z / hl }
  local y0 = p0.y
  local okm, mass = pcall(sublevel.getMass)
  note(string.format("START %s at %.1f %.1f %.1f heading %.0f deg  MASS %s", why, p0.x, p0.y, p0.z, math.deg(atan2(head0.x, head0.z)), tostring(okm and mass or "?")))
  flog = fs.open(C.LOG, "w")
  flog.writeLine("t phase steps gx gy px py pz vx vy vz speed hgt pitch pitch_cmd bank heading head_cmd wx wy wz")
  local t0 = os.clock()
  local last = t0
  local phase, pt = "ROLL", 0
  local steps, stepsI = 1, C.CLIMB_STEPS
  local pitchCmd, trim = math.rad(C.ROLL_PITCH), math.rad(C.TRIM0)
  local head = head0
  local turned = 0
  local rollAcc = {}
  local prevV
  local maxSpin = 0
  local stillT = 0
  local slowMin = math.huge
  local land = {}
  local cmdThr, cmdX, cmdY = 0, 0, 0
  while true do
    cmd[1], cmd[2], cmd[3] = cmdThr, cmdX, cmdY
    local pos, w, u, vel, om = tick(true)
    local now = os.clock()
    local dt = math.max(now - last, 0.01)
    last = now
    pt = pt + dt
    local T = now - t0
    local nose = rotate(w, u, C.FWD)
    local pitch = math.asin(clamp(nose.y, -1, 1))
    local upb = rotate(w, u, { x = 0, y = 1, z = 0 })
    local right = { x = nose.z, y = 0, z = -nose.x }
    local rn = math.sqrt(right.x * right.x + right.z * right.z)
    local bank = rn > 1e-6 and math.deg(math.asin(clamp(-(upb.x * right.x + upb.z * right.z) / rn, -1, 1))) or 0
    local sp = len(vel)
    local hs = math.sqrt(vel.x * vel.x + vel.z * vel.z)
    local hgt = pos.y - y0
    local heading = math.deg(atan2(nose.x, nose.z))
    local spin = len(om)
    maxSpin = math.max(maxSpin, spin)
    if spin > C.CUT_SPIN then R.reason = string.format("spin %.1f rad/s at %.1f s (%s)", spin, T, phase) break end
    if T > C.MAX_T then R.reason = "time limit" break end
    local function altHold(target, vyff)
      local vyc = clamp(C.K_HV * (target - pos.y), -C.VY_MAX, C.VY_MAX) + (vyff or 0)
      trim = clamp(trim + C.K_IVY * (vyc - vel.y) * dt, math.rad(C.TRIM_MIN), math.rad(C.TRIM_MAX))
      return trim + C.K_PVY * (vyc - vel.y)
    end
    local function speedSteps(vt, smin)
      stepsI = clamp(stepsI + 0.05 * (vt - sp) * dt, smin or 1, C.STEPS_MAX)
      return math.floor(clamp(stepsI + C.K_STEPS * (vt - sp), smin or 1, C.STEPS_MAX) + 0.5)
    end
    local want = pitchCmd
    if phase == "ROLL" then
      steps = math.min(C.ROLL_STEPS, 1 + math.floor(pt / C.ROLL_STEP_T))
      if prevV then
        local a = (hs - prevV) / dt
        local A = rollAcc[steps] or { n = 0, a = 0, v0 = hs }
        A.n, A.a, A.v1 = A.n + 1, A.a + a, hs
        rollAcc[steps] = A
      end
      want = math.rad(C.ROLL_PITCH)
      if sp >= C.VR then phase, pt = "ROTATE", 0 note(string.format("ROTATE at %.1f m/s after %.1f s, rolled %.0f blocks", sp, T, len(sub(pos, p0)))) end
      if T > C.ROLL_MAX_T or len(sub(pos, p0)) > C.ROLL_MAX_D then
        R.reason = string.format("no takeoff: %.1f m/s after %.0f blocks / %.1f s", sp, len(sub(pos, p0)), T)
        break
      end
    elseif phase == "ROTATE" then
      steps = C.ROLL_STEPS
      want = math.rad(C.ROTATE_PITCH)
      if hgt > C.LIFTOFF_H and vel.y > 0.3 then
        phase, pt = "CLIMB", 0
        R.liftV, R.liftD = sp, len(sub(pos, p0))
        note(string.format("LIFTOFF at %.1f m/s, %.0f blocks from start, %.1f s", sp, R.liftD, T))
      elseif pt > 10 then
        R.reason = "rotated but no liftoff in 10 s"
        break
      end
    elseif phase == "CLIMB" then
      steps = C.CLIMB_STEPS
      want = math.rad(C.CLIMB_PITCH)
      if hgt + math.max(vel.y, 0) * C.LEVEL_LEAD_T >= C.CRUISE_H then trim = pitch phase, pt = "LEVEL", 0 note(string.format("CRUISE HEIGHT +%.0f at %.1f s, %.1f m/s", hgt, T, sp)) end
    elseif phase == "LEVEL" then
      steps = speedSteps(C.V_LEVEL, 1)
      want = altHold(y0 + C.CRUISE_H)
      if pt > C.LEVEL_T then
        phase, pt = "SLOW", 0
        note(string.format("LEVEL: %.1f m/s at %d steps, pitch %+.1f, height %+.1f", sp, steps, math.deg(pitch), hgt))
      end
    elseif phase == "SLOW" then
      steps = C.SLOW_STEPS
      want = altHold(y0 + C.CRUISE_H)
      if math.abs(hgt - C.CRUISE_H) < 4 then slowMin = math.min(slowMin, sp) end
      if pt > C.SLOW_T or sp < C.SLOW_V_MIN or math.deg(pitch) > C.SLOW_PITCH_MAX then
        note(string.format("SLOW: %d step(s) for %.1f s, %.1f m/s, pitch %+.1f, height %+.1f, slowest at height %.1f m/s", steps, pt, sp, math.deg(pitch), hgt, slowMin))
        phase, pt = "TURN", 0
      end
    elseif phase == "TURN" then
      steps = speedSteps(C.V_LEVEL, 1)
      want = altHold(y0 + C.CRUISE_H)
      local step = math.rad(C.TURN_RATE) * dt
      if turned < math.rad(C.TURN_ANGLE) then
        head = rotY(head, step)
        turned = turned + step
      end
      local hd = math.deg(atan2(head.x, head.z))
      local herr = (hd - heading + 540) % 360 - 180
      if turned >= math.rad(C.TURN_ANGLE) and math.abs(herr) < 10 then
        note(string.format("TURN: %d deg in %.1f s (command %d deg/s), max bank %.0f", C.TURN_ANGLE, pt, C.TURN_RATE, R.maxBank or 0))
        phase, pt = "STRAIGHT", 0
      elseif pt > C.TURN_MAX_T then
        note(string.format("TURN: only reached heading error %.0f deg after %.0f s", herr, pt))
        phase, pt = "STRAIGHT", 0
      end
      R.maxBank = math.max(R.maxBank or 0, math.abs(bank))
    elseif phase == "STRAIGHT" then
      steps = speedSteps(C.V_LEVEL, 1)
      want = altHold(y0 + C.CRUISE_H)
      if pt > C.AFTER_TURN_T then phase, pt = "APPROACH", 0 note(string.format("APPROACH from height %+.1f at %.1f m/s", hgt, sp)) end
    elseif phase == "APPROACH" then
      steps = C.APP_STEPS
      want = altHold(pos.y, -hs * math.tan(math.rad(C.APP_SLOPE)))
      if hgt <= C.FLARE_H then phase, pt = "FLARE", 0 land.flareV, land.flareVy = sp, vel.y end
    elseif phase == "FLARE" then
      steps = C.APP_STEPS
      want = math.rad(C.FLARE_PITCH)
      if hgt <= C.TOUCH_H and math.abs(vel.y) < C.TOUCH_VY + 3 then
        phase, pt = "ROLLOUT", 0
        land.v, land.vy, land.pos = sp, vel.y, pos
        note(string.format("TOUCHDOWN at %.1f m/s, sink %.1f m/s (flare started at %.1f m/s, sink %.1f)", sp, vel.y, land.flareV or 0, land.flareVy or 0))
      elseif pt > 15 then
        note("FLARE: no touchdown in 15 s")
        phase, pt = "ROLLOUT", 0
      end
    elseif phase == "ROLLOUT" then
      steps = 0
      want = math.rad(0)
      if sp < C.STOP_V then stillT = stillT + dt else stillT = 0 end
      if stillT > C.STOP_T then
        if land.pos then note(string.format("STOPPED %.0f blocks after touchdown", len(sub(pos, land.pos)))) end
        R.reason = "landed and stopped"
        break
      end
    end
    local pr = math.rad(C.PITCH_RATE) * dt
    pitchCmd = clamp(clamp(want, -math.rad(C.PITCH_MAX), math.rad(C.PITCH_MAX)), pitchCmd - pr, pitchCmd + pr)
    if phase == "ROLLOUT" then
      cmdThr, cmdX, cmdY = 0, 0, 0
    else
      cmdThr = steps / 15 + 0.003
      cmdX, cmdY = hold(w, u, om, head, pitchCmd, steps, dt)
    end
    prevV = hs
    flog.writeLine(string.format("%.2f %s %d %+.2f %+.2f %.1f %.1f %.1f %+.2f %+.2f %+.2f %.1f %+.1f %+.1f %+.1f %+.1f %.0f %.0f %+.3f %+.3f %+.3f",
      T, phase, steps, cmdX, cmdY, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, sp, hgt, math.deg(pitch), math.deg(pitchCmd), bank, heading, math.deg(atan2(head.x, head.z)), om.x, om.y, om.z))
  end
  if R.reason == "?" then R.reason = "ended" end
  local ks = {}
  for k in pairs(rollAcc) do ks[#ks + 1] = k end
  table.sort(ks)
  for _, k in ipairs(ks) do
    local A = rollAcc[k]
    note(string.format("GROUND ROLL %d step(s): acceleration %.2f m/s2 (thrust alone ~%.1f), speed %.1f -> %.1f", k, A.a / A.n, 3.3 * k, A.v0 or 0, A.v1 or 0))
  end
  note(string.format("MAX SPIN %.2f rad/s", maxSpin))
end

local ok, err = pcall(main)
cut()
if flog then flog.close() end
if not ok then R.reason = "error: " .. tostring(err) end
note("END " .. R.reason)
save()
print("Saved " .. C.OUT .. " and " .. C.LOG)
