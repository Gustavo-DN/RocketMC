local C = {
  ENGINE_TYPE = "liquid_vector_thruster",
  TANK_TYPE = "fluid_storage",
  ENGINE_ORDER = { 2, 3, 1, 0 },
  THRUST_FORCE = 2025,
  GRAV = 10.5,
  KP = 1.5,
  KD = 1.6,
  KI = 0.8,
  IMAX = 1.0,
  IWIN = 35,
  REF_THR = 0.4,
  KD_MULT = 1.3,
  VMAX = 1.0,
  SIGN_X = 1,
  SIGN_Y = 1,
  SWAP = false,
  ROLL_KD = 0.4,
  ROLL_MAX = 0.4,
  CLIMB = 200,
  FLOOR = 120,
  PITCH_RATE = 15,
  PULL_RATE = 10,
  UP_RATE = 20,
  KALT = 0.05,
  KVY = 0.4,
  AMAX = 3,
  BIAS_KI = 0.01,
  BIAS_MAX = 12,
  PITCH_CAP = 60,
  PULL_PITCH = 85,
  PULL_GAMMA = 45,
  PULL_MAX_T = 20,
  COAST_END_VY = 5,
  COAST_THR = 0.14,
  COAST_END_V = 45,
  DESC_V = 11,
  DESC_V_LOW = 3,
  DESC_LOW = 40,
  LOG = "arcmaneuvertest.log",
  SUMMARY = "arcmaneuvertest.txt",
  TRIALS = { { v = 0, los = 30, lead = 4 }, { v = 200, los = 45, lead = 4 }, { v = 200, los = 60, lead = 4 } },
  CRUISE_H = 1700,
  DIVE_H = 450,
  SPEED_KT = 0.03,
  SPEED_MIN_THR = 0.5,
  SPEED_OK = 8,
  SPEED_HOLD_T = 3,
  ACCEL_MAX_T = 90,
  CLIMB_OK = 20,
  CLIMB_MAX_T = 120,
  DIVE_THR = 0.5,
  DIVE_NAV = 3,
  NOSE_OFFSET = 12.7,
  DRAG = 0.13,
  DIVE_FLOOR = 250,
  PULL_UP_RATE = 90,
  FUEL_MIN = 12000,
  LOG_DT = 0.25,
}

local function v3(a) return { x = a.x or a[1], y = a.y or a[2], z = a.z or a[3] } end
local function add(a, b) return { x = a.x + b.x, y = a.y + b.y, z = a.z + b.z } end
local function mul(a, s) return { x = a.x * s, y = a.y * s, z = a.z * s } end
local function dot(a, b) return a.x * b.x + a.y * b.y + a.z * b.z end
local function cross(a, b) return { x = a.y * b.z - a.z * b.y, y = a.z * b.x - a.x * b.z, z = a.x * b.y - a.y * b.x } end
local function len(a) return math.sqrt(dot(a, a)) end
local function unit(a)
  local l = len(a)
  if l < 1e-9 then return { x = 0, y = 0, z = 0 } end
  return mul(a, 1 / l)
end
local function clamp(x, lo, hi) return math.max(lo, math.min(hi, x)) end
local atan2 = math.atan2 or math.atan
local UP = { x = 0, y = 1, z = 0 }

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
local function turnToward(a, b, maxStep)
  local c = clamp(dot(a, b), -1, 1)
  local ang = math.acos(c)
  if ang <= maxStep or ang < 1e-6 then return b end
  local t = maxStep / ang
  local sn = math.sin(ang)
  return unit(add(mul(a, math.sin((1 - t) * ang) / sn), mul(b, math.sin(t * ang) / sn)))
end

local E = { names = {}, eng = {}, cmd = {}, tanks = {} }
for _, n in ipairs(peripheral.getNames()) do
  if peripheral.hasType(n, C.ENGINE_TYPE) then E.names[#E.names + 1] = n end
end
table.sort(E.names)
if #E.names ~= 4 then
  print("FlightTest needs exactly 4 thrusters, found " .. #E.names)
  return
end
do
  local ord, full = {}, true
  for _, n in ipairs(E.names) do
    local k = tonumber(n:match("_(%d+)$"))
    for i, want in ipairs(C.ENGINE_ORDER) do if k == want then ord[i] = n end end
  end
  for i = 1, 4 do if not ord[i] then full = false end end
  if full then E.names = ord end
end
for i, n in ipairs(E.names) do
  E.eng[i] = peripheral.wrap(n)
  E.cmd[i] = { 0, 0, 0 }
end
E.sx, E.sy = { 1, 1, -1, -1 }, { 1, -1, 1, -1 }
for _, n in ipairs(peripheral.getNames()) do
  if peripheral.hasType(n, C.TANK_TYPE) and not peripheral.hasType(n, C.ENGINE_TYPE) then E.tanks[#E.tanks + 1] = n end
end

local okM, mass = pcall(sublevel.getMass)
if not okM or type(mass) ~= "number" or mass <= 0 then mass = 64 end
local TACC = C.THRUST_FORCE / mass
local HOVER = C.GRAV / TACC

local function io(send)
  local fns, p, v, o = {}
  if send then
    for i = 1, 4 do
      local e, c = E.eng[i], E.cmd[i]
      local thr, gx, gy = c[1], c[2], c[3]
      fns[#fns + 1] = function() e.setThrustNormalized(thr) end
      fns[#fns + 1] = function() e.setVector(gx, gy) end
    end
  end
  fns[#fns + 1] = function() p = sublevel.getLogicalPose() end
  fns[#fns + 1] = function() v = sublevel.getLinearVelocity() end
  fns[#fns + 1] = function() o = sublevel.getAngularVelocity() end
  parallel.waitForAll(table.unpack(fns))
  local w, u = quat(p.orientation)
  return v3(p.position), w, u, v3(v), v3(o)
end

local function set(thr, gx, gy, r)
  thr = clamp(thr, 0, 1)
  for i = 1, 4 do
    local c = E.cmd[i]
    c[1] = thr
    c[2] = clamp(gx + E.sx[i] * r, -1, 1)
    c[3] = clamp(gy + E.sy[i] * r, -1, 1)
  end
end

local function cut()
  set(0, 0, 0, 0)
  pcall(io, true)
end

local attI = { x = 0, z = 0 }
local function steer(dirW, w, u, om, dt, thr)
  local d = unit(toBody(w, u, dirW))
  local axis = { x = d.z, y = 0, z = -d.x }
  local s = len(axis)
  local e = { x = 0, y = 0, z = 0 }
  if s > 1e-9 then e = mul(axis, atan2(s, d.y) / s) end
  if len(e) < math.rad(C.IWIN) then
    attI.x = clamp(attI.x + e.x * dt, -C.IMAX, C.IMAX)
    attI.z = clamp(attI.z + e.z * dt, -C.IMAX, C.IMAX)
  end
  local k = thr > C.REF_THR and C.REF_THR / thr or 1
  local kd = k < 1 and C.KD * C.KD_MULT or C.KD
  local gx = clamp(k * (C.KP * e.z + C.KI * attI.z - kd * om.z), -C.VMAX, C.VMAX) * C.SIGN_X
  local gy = clamp(-k * (C.KP * e.x + C.KI * attI.x - kd * om.x), -C.VMAX, C.VMAX) * C.SIGN_Y
  if C.SWAP then gx, gy = gy, gx end
  return gx, gy
end

local function rollCmd(om, thr, gx, gy, rate)
  local k = thr > C.REF_THR and C.REF_THR / thr or 1
  local lim = math.min(C.ROLL_MAX, math.max(1 - math.max(math.abs(gx), math.abs(gy)), 0))
  return clamp(-C.ROLL_KD * (om.y - rate) * k, -lim, lim)
end

local function fromPitch(f, deg)
  local r = math.rad(deg)
  return { x = f.x * math.cos(r), y = math.sin(r), z = f.z * math.cos(r) }
end

local summary, stats = {}, nil
local function stageStart(name, t, pos, v)
  stats = { name = name, t0 = t, x0 = pos.x, z0 = pos.z, alt0 = pos.y, v0 = v, vmax = v, n = 0, p1 = 0, p2 = 0, aoa = 0, rollMax = 0, spinMax = 0, g2 = 0, sat = 0, vyMin = 0, vyMax = 0 }
end

local function stageEnd(t, pos, v)
  if not stats then return end
  local s, n = stats, math.max(stats.n, 1)
  local dur = t - s.t0
  local pm = s.p1 / n
  summary[#summary + 1] = string.format("%-9s %5.1fs  speed %5.1f -> %5.1f (max %5.1f, %+5.1f m/s per s)  dist %5.0f  alt %4.0f -> %4.0f  vy %+5.1f..%+5.1f",
    s.name, dur, s.v0, v, s.vmax, dur > 0 and (v - s.v0) / dur or 0, math.sqrt((pos.x - s.x0) ^ 2 + (pos.z - s.z0) ^ 2), s.alt0, pos.y, s.vyMin, s.vyMax)
  summary[#summary + 1] = string.format("         pitch avg %5.1f wobble %4.1f  nose vs path %4.1f  roll rate max %4.2f  spin max %4.2f  gimbal RMS %.2f at limit %2.0f%%",
    pm, math.sqrt(math.max(s.p2 / n - pm * pm, 0)), s.aoa / n, s.rollMax, s.spinMax, math.sqrt(s.g2 / n), 100 * s.sat / n)
  stats = nil
end

local function stageSample(v, vel, pitch, aoa, om, gx, gy)
  local s = stats
  if not s then return end
  s.n, s.vmax = s.n + 1, math.max(s.vmax, v)
  s.p1, s.p2, s.aoa = s.p1 + pitch, s.p2 + pitch * pitch, s.aoa + aoa
  s.rollMax, s.spinMax = math.max(s.rollMax, math.abs(om.y)), math.max(s.spinMax, len(om))
  s.g2 = s.g2 + gx * gx + gy * gy
  if math.max(math.abs(gx), math.abs(gy)) >= 0.99 then s.sat = s.sat + 1 end
  s.vyMin, s.vyMax = math.min(s.vyMin, vel.y), math.max(s.vyMax, vel.y)
end

local aborted = false

local dives = {}

local function fluidTotal()
  local sum, any = 0, false
  local list = {}
  for _, n in ipairs(E.tanks) do list[#list + 1] = n end
  for _, n in ipairs(E.names) do list[#list + 1] = n end
  for _, n in ipairs(list) do
    local ok, t = pcall(peripheral.call, n, "tanks")
    if ok and type(t) == "table" then
      for _, fl in pairs(t) do sum, any = sum + (fl.amount or 0), true end
    end
  end
  return any and sum or nil
end

local function writeTable(extra)
  local f = fs.open(C.SUMMARY, "w")
  if not f then return end
  f.writeLine(string.format("ARC MANEUVER TEST  MASS %.1f  TACC %.1f  HOVER %.3f", mass, TACC, HOVER))
  f.writeLine("")
  f.writeLine(string.format("DIVE TABLE (Rocket4 ARC steering, throttle %.2f, destination %d blocks below)", C.DIVE_THR, C.DIVE_H))
  f.writeLine("start dist = drop / tan(angle) + entry v * lead (Rocket4 ARC rule, lead 4 s)")
  f.writeLine("entry v | angle | start dist | dive time | miss  | arrive v | path angle | nose tilt | pull-out drop | max spin | note")
  for _, r in ipairs(dives) do
    f.writeLine(string.format("%6.1f  | %4d  | %9.0f  | %8.1fs | %5.1f | %7.1f  | %9.1f  | %8.1f  | %12.0f  | %7.2f  | %s",
      r.v0, r.los, r.d0, r.t or 0, r.miss or -1, r.v1 or 0, r.gam or 0, r.tilt or 0, r.pull or -1, r.spin, r.note or ""))
  end
  f.writeLine("")
  for _, l in ipairs(summary) do f.writeLine(l) end
  if extra then f.writeLine(extra) end
  f.close()
end

local function approachDir(tgt, pos, vel, nose, thr)
  local r = { x = tgt.x - pos.x - nose.x * C.NOSE_OFFSET, y = tgt.y - pos.y - nose.y * C.NOSE_OFFSET, z = tgt.z - pos.z - nose.z * C.NOSE_OFFSET }
  local dist = len(r)
  if dist < 1e-6 then return nose end
  local rh = mul(r, 1 / dist)
  local tArr = clamp(dist / math.max(dot(vel, rh), 1), 0.3, 20)
  local acc = mul(vel, -C.DRAG)
  acc.y = acc.y - C.GRAV
  local p = add(mul(vel, tArr), mul(acc, 0.5 * tArr * tArr))
  local drift = { x = r.x - p.x, y = r.y - p.y, z = r.z - p.z }
  local dr = dot(drift, rh)
  local zp = { x = drift.x - rh.x * dr, y = drift.y - rh.y * dr, z = drift.z - rh.z * dr }
  local T = math.max(thr, 0.02) * TACC
  local aLat = mul(zp, C.DIVE_NAV / (tArr * tArr))
  local al = len(aLat)
  if al > 3 * T then aLat = mul(aLat, 3 * T / al) end
  return unit(add(mul(rh, T), aLat))
end

local function run()
  local pos, w, u, vel, om = io(false)
  local y0 = pos.y
  local fb = rotate(w, u, { x = 0, y = 0, z = 1 })
  local f = unit({ x = fb.x, y = 0, z = fb.z })
  if len(f) < 0.5 then f = { x = 1, y = 0, z = 0 } end
  local f0 = f
  local log = fs.open(C.LOG, "w")
  if log then
    log.writeLine(string.format("ARC MANEUVER TEST y0 %.1f heading %.2f %.2f MASS %.1f TACC %.1f", y0, f.x, f.z, mass, TACC))
    log.writeLine("t stage trial thr speed alt vy pitch nose_vs_path spin gx gy yaw vh dh_tgt")
  end
  local stage, tS = "LIFTOFF", 0
  local thr = HOVER * 0.75
  local cmd = UP
  local cmdPitch = 90
  local holdY, bias = y0 + C.CLIMB, 0
  local calmT, okT = 0, 0
  local locked = true
  local ti, rec, tgt = 0, nil, nil
  local t0 = os.clock()
  local last, nextLog = t0, t0
  stageStart(stage, 0, pos, 0)
  local function go(name, t)
    stageEnd(t, pos, len(vel))
    stage, tS = name, 0
    stageStart(name, t, pos, len(vel))
  end
  local function hold(amax, dt)
    local e = holdY - pos.y
    if not locked then
      holdY, e = pos.y, 0
      if vel.y < 1 then locked = true end
    end
    if math.abs(e) < 40 and math.abs(vel.y) < 10 then bias = clamp(bias + C.BIAS_KI * e * dt, -C.BIAS_MAX, C.BIAS_MAX) end
    local ad = clamp(C.KALT * e - C.KVY * vel.y, -amax, amax)
    local T = math.max(thr, 0.05) * TACC
    return math.min(math.deg(math.asin(clamp((C.GRAV + ad + bias) / T, -1, 1))), C.PITCH_CAP)
  end
  local function hdist(a, b) return math.sqrt((a.x - b.x) ^ 2 + (a.z - b.z) ^ 2) end
  local function nextTrial(t)
    ti = ti + 1
    local fuelNow = fluidTotal()
    if not C.TRIALS[ti] then
      go("UPRIGHT", t)
    elseif fuelNow and fuelNow < C.FUEL_MIN then
      summary[#summary + 1] = string.format("LOW FUEL %.0f mB before trial %d, landing", fuelNow, ti)
      go("UPRIGHT", t)
    else
      holdY, locked = y0 + C.CRUISE_H, true
      go("CLIMBUP", t)
    end
  end
  while not aborted do
    local now = os.clock()
    local t = now - t0
    local dt = clamp(now - last, 0.01, 0.5)
    last = now
    tS = tS + dt
    local v = len(vel)
    local nose = rotate(w, u, UP)
    local vh = math.sqrt(vel.x * vel.x + vel.z * vel.z)
    local fast = stage == "PITCHOVER" or stage == "CLIMBUP" or stage == "ACCEL"
    if fast and pos.y < y0 + C.FLOOR then go("PULLUP", t) end
    if stage == "LIFTOFF" then
      thr = math.min(thr + 0.3 * dt, 0.9)
      cmd = UP
      if vel.y > 0.5 then go("ASCENT", t) end
    elseif stage == "ASCENT" then
      local vyT = clamp(math.sqrt(2 * 2 * math.max(holdY - pos.y, 0)), 0, 25)
      thr = clamp((C.GRAV + (vyT - vel.y)) / TACC, 0.1, 0.9)
      cmd = UP
      if pos.y >= holdY - 3 then
        locked = false
        go("PITCHOVER", t)
      end
    elseif stage == "PITCHOVER" or stage == "CLIMBUP" or stage == "ACCEL" then
      local vT = stage == "ACCEL" and C.TRIALS[ti].v or 0
      if vT > 0 then
        thr = clamp(thr + (clamp(C.SPEED_KT * (vT - v), -0.5, 0.5) - 0) * dt * 2, C.SPEED_MIN_THR, 1)
      else
        thr = math.min(thr + 0.5 * dt, 1)
      end
      local want = hold(C.AMAX, dt)
      cmdPitch = cmdPitch + clamp(want - cmdPitch, -C.PITCH_RATE * dt, C.PITCH_RATE * dt)
      cmd = fromPitch(f, cmdPitch)
      if stage == "PITCHOVER" then
        if locked and math.abs(vel.y) < 3 then nextTrial(t) end
      elseif stage == "CLIMBUP" then
        if (math.abs(holdY - pos.y) < C.CLIMB_OK and math.abs(vel.y) < 5) or tS > C.CLIMB_MAX_T then go("ACCEL", t) end
      else
        local tr = C.TRIALS[ti]
        if tr.v > 0 and math.abs(v - tr.v) < C.SPEED_OK and math.abs(vel.y) < 5 then okT = okT + dt else okT = 0 end
        if (tr.v > 0 and okT > C.SPEED_HOLD_T) or (tr.v == 0 and tS > 1) or tS > C.ACCEL_MAX_T then
          okT = 0
          local fwd = vh > 1 and { x = vel.x / vh, y = 0, z = vel.z / vh } or f
          local d0 = C.DIVE_H / math.tan(math.rad(tr.los)) + v * tr.lead
          tgt = { x = pos.x + fwd.x * d0, y = pos.y - C.DIVE_H, z = pos.z + fwd.z * d0 }
          rec = { v0 = v, los = tr.los, d0 = d0, t0 = t, p0 = pos, spin = 0 }
          dives[#dives + 1] = rec
          go("DIVE", t)
        end
      end
    elseif stage == "DIVE" then
      thr = C.DIVE_THR
      cmd = approachDir(tgt, pos, vel, nose, thr)
      local tip = add(pos, mul(nose, C.NOSE_OFFSET))
      if tip.y <= tgt.y or pos.y < y0 + C.DIVE_FLOOR then
        rec.t, rec.miss, rec.v1 = t - rec.t0, hdist(tip, tgt), v
        rec.gam = math.deg(math.asin(clamp(-vel.y / math.max(v, 1), -1, 1)))
        rec.tilt = math.deg(math.acos(clamp(nose.y, -1, 1)))
        rec.cross, rec.minY = pos.y, pos.y
        if tip.y > tgt.y then rec.note = "floor abort" end
        writeTable()
        go("PULLOUT", t)
      end
    elseif stage == "PULLOUT" then
      thr = 1
      cmd = turnToward(unit(cmd), UP, math.rad(C.PULL_UP_RATE) * dt)
      rec.minY = math.min(rec.minY, pos.y)
      if vel.y >= 0 then
        rec.pull = rec.cross - rec.minY
        writeTable()
        if vh > 1 then f = { x = vel.x / vh, y = 0, z = vel.z / vh } end
        cmdPitch = math.deg(math.asin(clamp(nose.y, -1, 1)))
        nextTrial(t)
      end
    elseif stage == "PULLUP" then
      thr = 1
      cmdPitch = cmdPitch + clamp(C.PULL_PITCH - cmdPitch, -C.PULL_RATE * dt, C.PULL_RATE * dt)
      cmd = fromPitch(f, cmdPitch)
      local gam = math.deg(math.asin(clamp(vel.y / math.max(v, 1), -1, 1)))
      if gam >= C.PULL_GAMMA or tS > C.PULL_MAX_T or v < C.COAST_END_V then go("COAST", t) end
    elseif stage == "COAST" then
      thr = C.COAST_THR
      cmd = v > 1 and unit(vel) or UP
      if v < C.COAST_END_V or vel.y < C.COAST_END_VY then go("UPRIGHT", t) end
    elseif stage == "UPRIGHT" then
      thr = HOVER
      cmd = turnToward(unit(cmd), UP, math.rad(C.UP_RATE) * dt)
      if nose.y > 0.95 then go("DESCENT", t) end
    elseif stage == "DESCENT" then
      local vyT = pos.y - y0 > C.DESC_LOW and -C.DESC_V or -C.DESC_V_LOW
      local ax, az = clamp(-0.3 * vel.x, -3, 3), clamp(-0.3 * vel.z, -3, 3)
      cmd = unit({ x = ax, y = C.GRAV, z = az })
      thr = clamp((C.GRAV + (vyT - vel.y)) / (TACC * math.max(cmd.y, 0.5)), 0.1, 0.9)
      if math.abs(vel.y) < 0.5 and vh < 1 and tS > 5 then calmT = calmT + dt else calmT = 0 end
      if calmT > 1 then
        go("LANDED", t)
        break
      end
    end
    if rec and (stage == "DIVE" or stage == "PULLOUT") then rec.spin = math.max(rec.spin, len(om)) end
    local gx, gy = steer(cmd, w, u, om, dt, thr)
    local r = rollCmd(om, thr, gx, gy, 0)
    set(thr, gx, gy, r)
    local pitch = math.deg(math.asin(clamp(nose.y, -1, 1)))
    local aoa = v > 1 and math.deg(math.acos(clamp(dot(nose, vel) / v, -1, 1))) or 0
    stageSample(v, vel, pitch, aoa, om, gx, gy)
    if log and now >= nextLog then
      local yaw = math.deg(atan2(f0.x * nose.z - f0.z * nose.x, f0.x * nose.x + f0.z * nose.z))
      log.writeLine(string.format("%.2f %s %d %.2f %.1f %.1f %+.1f %+.1f %.1f %.2f %+.2f %+.2f %+.0f %.1f %.0f", t, stage, ti, thr, v, pos.y, vel.y, pitch, aoa, len(om), gx, gy, yaw, vh, tgt and hdist(pos, tgt) or -1))
      log.flush()
      nextLog = now + C.LOG_DT
    end
    pos, w, u, vel, om = io(true)
  end
  stageEnd(os.clock() - t0, pos, len(vel))
  writeTable()
  if log then
    log.writeLine(aborted and "ABORT" or "END")
    log.close()
  end
  cut()
end

local function fuel()
  while true do
    local fns = {}
    for i = 1, 4 do
      local e = E.eng[i]
      for _, tk in ipairs(E.tanks) do
        fns[#fns + 1] = function() pcall(e.pullFluid, tk) end
      end
    end
    if #fns > 0 then parallel.waitForAll(table.unpack(fns)) end
    sleep(0.5)
  end
end

local function keys_()
  while true do
    local e, k = os.pullEvent("key")
    if k == keys.backspace then
      aborted = true
      return
    end
  end
end

term.clear()
term.setCursorPos(1, 1)
print("ARC MANEUVER TEST: climb to +1700, then nose-down")
print("dives at an imaginary destination 350 blocks lower,")
print("pull-outs, and an upright descent at the end.")
print("Backspace cuts the engines at any time.")
print("Launch in 5 s.")
for _ = 1, 5 do
  local timer = os.startTimer(1)
  while true do
    local e, a = os.pullEvent()
    if e == "key" and a == keys.backspace then
      print("Cancelled.")
      return
    end
    if e == "timer" and a == timer then break end
  end
end
print("Flying. Data: " .. C.LOG .. " and " .. C.SUMMARY)
local ok, err = pcall(parallel.waitForAny, run, fuel, keys_)
cut()
if not ok then
  writeTable("ERROR " .. tostring(err))
  print("Error: " .. tostring(err))
end
if aborted then writeTable("ABORTED by Backspace") end
print("")
for _, r in ipairs(dives) do
  print(string.format("%3.0f m/s, %2d deg: miss %s, arrive %s m/s, pull-out %s", r.v0, r.los, r.miss and string.format("%.1f", r.miss) or "--", r.v1 and string.format("%.0f", r.v1) or "--", r.pull and string.format("%.0f", r.pull) or "--"))
end
print("Saved " .. C.SUMMARY .. " and " .. C.LOG)
