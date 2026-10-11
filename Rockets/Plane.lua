local C = {}
C.REMOTE = ...
if type(C.REMOTE) ~= "table" then C.REMOTE = nil end
C.SIDE_ENGINE = "back"
C.SIDE_TANK = "top"
C.CLIMB = 50
C.DELAY = 5
C.VMAX = 1.0
C.KP = 1.0
C.KD = 0.6
C.ATT_KI = 1.0
C.ATT_IWIN = 20
C.ATT_IMAX = 1.0
C.ATT_REF_THR = 0.5
C.MAX_THRUST = 0.40
C.CRUISE_THR = 0.07
C.PLANE_THR = 0.07
C.START_THRUST = C.PLANE_THR
C.THR_RAMP = 0.05
C.THR_MARGIN = 0.03
C.LIFTOFF_VY = 0.5
C.ASCENT_TWR = 2
C.ASCENT_STALL_H = 5
C.TURN_RATE = 8
C.KV = 1.0
C.ERR_MAX = 0.6
local FWD = { x = 0, y = 0, z = 1 }
local RGT = { x = 1, y = 0, z = 0 }
local axisInfo = {}
C.MAP_PITCH = -4.37
C.MAP_YAW = -3.88
C.KP_A = 4
C.KI_A = 3
C.KD_A = 3
C.I_MAX = 1.0
C.I_WIN = 10
C.GMAX = 0.9
C.V_CRUISE = 100
C.STEPS_START = 3
C.STEPS_MAX = 8
C.DIVE_STEPS = 3
C.STEPS_MIN = 3
C.STEP_PER_V = 60
C.K_V = 0.05
C.KI_V = 0.03
C.CRUISE_MIN_H = 60
C.K_H = 0.001
C.GAMMA_MAX = 6
C.AOA_START = 8
C.K_TRIM = 0.2
C.TRIM_MIN = -5
C.TRIM_MAX = 15
C.PITCH_MAX = 25
C.PITCH_RATE = 15
C.YAW_MAX = 15
C.GLIDE_ANG = 10
C.FINAL_T = 1
C.DIVE_MIN_D = 40
C.DIVE_K = 1.0
C.DIVE_GRAV_T = 0.3
C.FF_TAU = 0.2
C.FF_MAX = 1.5
C.FF_JUMP = 0.94
C.SIGN_X = 1
C.SIGN_Y = 1
C.SWAP = false
C.HIT_RADIUS = 3
C.CUT_TIME = 0.1
C.STUCK_V = 2
C.STUCK_T = 1
C.NOSE_OFFSET = 1
C.CUT_ERR = 100
C.CUT_ERR_T = 2
C.CUT_LOW = 15
C.CUT_STILL_V = 1
C.CUT_STILL_T = 3
C.GRAV = 10.5
C.DRAG = 0.13
C.THRUST_FORCE = 225
C.THRUST_ACC = 77.6
C.REF_I = 3.15
C.GAIN_MIN = 0.25
C.GAIN_MAX = 4
local inertiaK = 1
C.FAST_THR = C.CRUISE_THR
C.SLOW_DIST = 300
C.WATER_RAMP = 0.3
C.SHUTTLE_RAMP = 1.0
C.NAV = 3
C.HOLD_TGO = 0.4
C.SEP_DELAY = 0.5
C.REL_SIDE = "top"
C.FLARE_SIDE = "front"
C.FLARE_PULSE = 1.0
C.VT_PITCH = 60
C.VT_FAIL_PITCH = 30
C.VT_FAIL_T = 1
C.VT_STUCK_T = 4
C.VT_STUCK_H = 2
C.VT_STEPS = 8
C.VT_STEP_T = 0.25
C.VT_H = 20
C.PO_RATE = 25
C.PO_STEPS = 6
C.PO_END_PITCH = 20
C.PO_V = 45
C.CLIMB_PITCH = 12
C.K_HV = 0.3
C.VY_MAX = 8
C.K_PVY = 0.012
C.K_IVY = 0.004
C.TRIM0 = 8
C.TRIM_LO = -5
C.TRIM_HI = 18
C.PITCH_LIM = 25
C.PITCH_RATE_C = 15
C.TURN_RATE = 8
C.V_FLY = 80
C.V_APP = 40
C.SLOW_D = 300
C.ALIGN_ERR = 45
C.GA_ERR = 60
C.GA_HIGH = 35
C.APP_HIGH = 30
C.K_STEPS = 0.1
C.STEPS_TOP = 8
C.APP_SLOPE = 4
C.APP_STEPS = 1
C.APP_LEAD = 150
C.STEEP_SLOPE = 10
C.APP_H2 = 40
C.K_APP_STEPS = 0.3
C.APP_STEPS_MAX = 6
C.ALIGN_D = 150
C.FLARE_H = 4
C.FLARE_PITCH = 6
C.TOUCH_H = 1
C.TOUCH_VY = 4
C.FLARE_MAX_T = 15
C.CRUISE_ABOVE = 30
C.REL_ON_MASS = false
C.REL_MASS = 0.7
C.REL_FREEFALL = false
C.REL_FALL_ACC = 6
C.REL_FALL_T = 0.4
C.REL_MIN_T = 1
C.SIM_TIME = 240
C.SAVE = "rocket.target"
C.LOG = "rocket.log"
C.FLIGHT_LOG = "flight.log"
C.MODES = { "WATER", "SHUTTLE" }
local climb = C.CLIMB
local mode = "WATER"

local engine = peripheral.wrap(C.SIDE_ENGINE)
if not engine then error("no vector engine on " .. C.SIDE_ENGINE, 0) end

local W, H = term.getSize()
local atan2 = math.atan2 or math.atan

local function v3(a) return { x = a.x or a[1], y = a.y or a[2], z = a.z or a[3] } end
local function add(a, b) return { x = a.x + b.x, y = a.y + b.y, z = a.z + b.z } end
local function sub(a, b) return { x = a.x - b.x, y = a.y - b.y, z = a.z - b.z } end
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

local function tick(thr, gx, gy, withMass)
  local p, v, o, m
  local fns = {
    function() p = sublevel.getLogicalPose() end,
    function() v = sublevel.getLinearVelocity() end,
    function() o = sublevel.getAngularVelocity() end,
  }
  if thr then
    fns[#fns + 1] = function() engine.setThrustNormalized(thr) end
    fns[#fns + 1] = function() engine.setVector(gx, gy) end
  end
  if withMass then fns[#fns + 1] = function() m = sublevel.getMass() end end
  parallel.waitForAll(table.unpack(fns))
  local w, u = quat(p.orientation)
  return v3(p.position), w, u, v3(v), v3(o), m
end

local function cutEngine()
  pcall(engine.setThrustNormalized, 0)
  pcall(engine.setVector, 0, 0)
end

local function turnToward(a, b, maxStep)
  local c = clamp(dot(a, b), -1, 1)
  local ang = math.acos(c)
  if ang <= maxStep or ang < 1e-6 then return b end
  local t = maxStep / ang
  local sn = math.sin(ang)
  if sn < 1e-4 then
    local p = math.abs(a.x) < 0.9 and { x = 1, y = 0, z = 0 } or { x = 0, y = 0, z = 1 }
    p = unit(sub(p, mul(a, dot(p, a))))
    return unit(add(mul(a, math.cos(maxStep)), mul(p, math.sin(maxStep))))
  end
  return unit(add(mul(a, math.sin((1 - t) * ang) / sn), mul(b, math.sin(t * ang) / sn)))
end

local function aim(dir, vel)
  local sp = len(vel)
  if sp < 3 then return dir end
  return unit(add(dir, mul(sub(dir, mul(vel, 1 / sp)), C.KV)))
end

local attI = { x = 0, z = 0 }
local ff = { w = { x = 0, y = 0, z = 0 } }

local function steer(dirW, w, u, om, dt, thr)
  local du = unit(dirW)
  if ff.prev and dt and dt > 0 and dot(ff.prev, du) > C.FF_JUMP then
    local a = math.min(1, dt / C.FF_TAU)
    ff.w = add(ff.w, mul(sub(mul(cross(ff.prev, du), 1 / dt), ff.w), a))
    local m = len(ff.w)
    if m > C.FF_MAX then ff.w = mul(ff.w, C.FF_MAX / m) end
  end
  ff.prev = du
  local steps = math.floor(clamp(thr or 0, 0, 1) * 15 + 1e-9)
  if steps < 1 then return 0, 0 end
  local wbb = toBody(w, u, ff.w)
  local db = unit(toBody(w, u, dirW))
  local wb = { x = dot(wbb, RGT), y = wbb.y }
  local d = { x = dot(db, RGT), y = db.y, z = dot(db, FWD) }
  om = { x = dot(om, RGT), y = om.y }
  local s = math.sqrt(d.x * d.x + d.y * d.y)
  local ex, ey = 0, 0
  if s > 1e-9 then
    local k = math.min(atan2(s, d.z), C.ERR_MAX) / s
    ex, ey = -d.y * k, d.x * k
  elseif d.z < 0 then
    ex = C.ERR_MAX
  end
  if dt and math.sqrt(ex * ex + ey * ey) < C.I_WIN then
    attI.x = clamp(attI.x + ex * dt, -C.I_MAX, C.I_MAX)
    attI.z = clamp(attI.z + ey * dt, -C.I_MAX, C.I_MAX)
  end
  local ax = C.KP_A * ex + C.KI_A * attI.x - C.KD_A * (om.x - wb.x)
  local ay = C.KP_A * ey + C.KI_A * attI.z - C.KD_A * (om.y - wb.y)
  local gy = clamp(ax / (C.MAP_PITCH * steps / inertiaK), -C.GMAX, C.GMAX)
  local gx = clamp(ay / (C.MAP_YAW * steps / inertiaK), -C.GMAX, C.GMAX)
  return gx, gy
end

local function newGuide(start, tgt, md, cl)
  local hc = md == "SHUTTLE" and start.y or start.y + cl
  return {
    start = start, tgt = tgt, mode = md, climb = cl,
    phase = md == "SHUTTLE" and "CRUISE" or "VTAKE", tmode = "", pt = 0,
    throttle = 0, steps = 1, stepsI = 3,
    hCmd = math.max(hc, tgt.y + C.CRUISE_ABOVE), trim = math.rad(C.TRIM0), stuck = 0,
  }
end

local function contactPoint(pos, nose)
  return add(pos, mul(nose, C.NOSE_OFFSET))
end

local function diveDir(g, pos, vel, nose)
  local r = sub(g.tgt, add(pos, mul(nose, C.NOSE_OFFSET)))
  local dist = len(r)
  if dist < 1e-6 then return nose end
  local rh = mul(r, 1 / dist)
  local tgo = clamp(dist / math.max(dot(vel, rh), 1), 0.3, 20)
  if g.held and tgo < C.HOLD_TGO then return g.held end
  local acc = mul(vel, -C.DRAG)
  acc.y = acc.y - C.GRAV
  local zem = sub(r, add(mul(vel, tgo), mul(acc, 0.5 * tgo * tgo)))
  local zp = sub(zem, mul(rh, dot(zem, rh)))
  local T = math.max(g.throttle, 0.02) * C.THRUST_ACC
  local aLat = mul(zp, C.NAV / (tgo * tgo))
  local al = len(aLat)
  if al > 3 * T then aLat = mul(aLat, 3 * T / al) end
  g.held = unit(add(mul(rh, T), aLat))
  return g.held
end

local function guideStep(g, pos, vel, nose, dt)
  local tgt = g.tgt
  local hx, hz = tgt.x - pos.x, tgt.z - pos.z
  local dh = math.sqrt(hx * hx + hz * hz)
  local sp = len(vel)
  local hs = math.sqrt(vel.x * vel.x + vel.z * vel.z)
  local pitch = math.asin(clamp(nose.y, -1, 1))
  local hgt = pos.y - tgt.y
  g.pt = g.pt + dt
  if not g.head then
    local h
    if math.deg(pitch) > C.VT_PITCH and g.upb then
      h = { x = -g.upb.x, y = 0, z = -g.upb.z }
    elseif hs > 5 then
      h = { x = vel.x, y = 0, z = vel.z }
    else
      h = { x = nose.x, y = 0, z = nose.z }
    end
    if len(h) < 1e-3 then h = { x = hx, y = 0, z = hz } end
    g.head = unit(h)
    g.pitchCmd = g.phase == "VTAKE" and math.rad(90) or pitch
  end
  local function altHold(target, vyff)
    local vyc = clamp(C.K_HV * (target - pos.y), -C.VY_MAX, C.VY_MAX) + (vyff or 0)
    g.trim = clamp(g.trim + C.K_IVY * (vyc - vel.y) * dt, math.rad(C.TRIM_LO), math.rad(C.TRIM_HI))
    return g.trim + C.K_PVY * (vyc - vel.y)
  end
  local function speedSteps(vt)
    g.stepsI = clamp(g.stepsI + 0.05 * (vt - sp) * dt, 1, C.STEPS_TOP)
    return math.floor(clamp(g.stepsI + C.K_STEPS * (vt - sp), 1, C.STEPS_TOP) + 0.5)
  end
  if dh > 1 and (g.phase == "CRUISE" or (g.phase == "APPROACH" and dh > C.ALIGN_D)) then
    g.head = turnToward(g.head, { x = hx / dh, y = 0, z = hz / dh }, math.rad(C.TURN_RATE) * dt)
  end
  local want, steps = g.pitchCmd, g.steps
  local d2 = C.APP_H2 / math.tan(math.rad(C.APP_SLOPE))
  local function slopeH(d)
    if d > d2 then return tgt.y + C.APP_H2 + (d - d2) * math.tan(math.rad(C.STEEP_SLOPE)) end
    return tgt.y + d * math.tan(math.rad(C.APP_SLOPE))
  end
  local herr = 0
  if dh > 1 and hs > 5 then
    local c = (vel.x * hx + vel.z * hz) / (hs * dh)
    herr = math.deg(math.acos(clamp(c, -1, 1)))
  end
  if g.phase == "VTAKE" then
    steps = math.min(C.VT_STEPS, 1 + math.floor(g.pt / C.VT_STEP_T))
    want = math.rad(90)
    if (g.pt > C.VT_FAIL_T and math.deg(pitch) < C.VT_FAIL_PITCH) or (g.pt > C.VT_STUCK_T and pos.y - g.start.y < C.VT_STUCK_H) then
      g.phase, g.throttle, g.tmode = "CUT", 0, "TAKEOFF FAIL"
      return nil, true
    end
    if pos.y - g.start.y >= C.VT_H then g.phase, g.pt = "PITCHOVER", 0 end
  elseif g.phase == "PITCHOVER" then
    steps = C.PO_STEPS
    want = math.rad(C.CLIMB_PITCH)
    if (math.deg(pitch) <= C.PO_END_PITCH and hs >= C.PO_V) or g.pt > 15 then
      g.phase, g.pt, g.trim = "CRUISE", 0, math.rad(C.TRIM0)
    end
  elseif g.phase == "CRUISE" then
    local dApp = math.min(math.max(hgt, 0), C.APP_H2) / math.tan(math.rad(C.APP_SLOPE)) + math.max(hgt - C.APP_H2, 0) / math.tan(math.rad(C.STEEP_SLOPE))
    local near = dh <= dApp + C.APP_LEAD + C.SLOW_D
    steps = speedSteps(near and C.V_APP or C.V_FLY)
    local hRef = math.max(tgt.y + C.APP_H2, math.min(g.hCmd, slopeH(dh)))
    want = altHold(hRef)
    if dh <= dApp + C.APP_LEAD and herr < C.ALIGN_ERR and pos.y - slopeH(dh) < C.APP_HIGH then g.phase, g.pt = "APPROACH", 0 end
  elseif g.phase == "APPROACH" then
    local slope = dh > d2 and C.STEEP_SLOPE or C.APP_SLOPE
    local href = math.min(slopeH(dh), g.hCmd)
    if dh > C.ALIGN_D and (herr > C.GA_ERR or pos.y - href > C.GA_HIGH) then
      g.phase, g.pt = "CRUISE", 0
      g.goArounds = (g.goArounds or 0) + 1
    end
    g.stepsI = clamp(g.stepsI + 0.05 * (C.V_APP - sp) * dt, C.APP_STEPS, C.APP_STEPS_MAX)
    steps = math.floor(clamp(g.stepsI + C.K_STEPS * (C.V_APP - sp) + C.K_APP_STEPS * math.max(href - pos.y, 0), C.APP_STEPS, C.APP_STEPS_MAX) + 0.5)
    want = altHold(href, -hs * math.tan(math.rad(slope)))
    if hgt <= C.FLARE_H then g.phase, g.pt = "FLARE", 0 end
  elseif g.phase == "FLARE" then
    steps = C.APP_STEPS
    want = math.rad(C.FLARE_PITCH)
    if (hgt <= C.TOUCH_H and vel.y > -C.TOUCH_VY) or g.pt > C.FLARE_MAX_T then
      g.phase, g.throttle, g.tmode = "TOUCHDOWN", 0, "OFF"
      return nil, true
    end
  end
  local tall = g.phase == "VTAKE" or g.phase == "PITCHOVER"
  local lim = math.rad(tall and 90 or C.PITCH_LIM)
  local pr = math.rad(g.phase == "PITCHOVER" and C.PO_RATE or C.PITCH_RATE_C) * dt
  g.pitchCmd = clamp(clamp(want, -lim, lim), g.pitchCmd - pr, g.pitchCmd + pr)
  g.steps, g.tmode = steps, g.phase
  g.throttle = steps / 15 + 0.003
  local p = g.pitchCmd
  return { x = g.head.x * math.cos(p), y = math.sin(p), z = g.head.z * math.cos(p) }
end

local function newTracker() return { bestD = math.huge } end

local function track(tr, p, tgt)
  local d = len(sub(tgt, p))
  if d < tr.bestD then tr.bestD, tr.best = d, p end
  if tr.prev and not tr.hit and tr.prev.y > tgt.y and p.y <= tgt.y then
    local f = (tr.prev.y - tgt.y) / (tr.prev.y - p.y)
    tr.hit = add(tr.prev, mul(sub(p, tr.prev), f))
  end
  tr.prev = p
end

local function missOf(tr, start, tgt)
  local p = tr.hit or tr.best
  if not p then return nil end
  local mx, mz = p.x - tgt.x, p.z - tgt.z
  local fx, fz = tgt.x - start.x, tgt.z - start.z
  local fd = math.sqrt(fx * fx + fz * fz)
  if fd < 1e-6 then fx, fz, fd = 0, -1, 1 end
  fx, fz = fx / fd, fz / fd
  return { total = math.sqrt(mx * mx + mz * mz), long = mx * fx + mz * fz, right = mz * fx - mx * fz, crossed = tr.hit ~= nil }
end

local function predict(start, tgt, md, cl, v0)
  local g = newGuide(start, tgt, md, cl)
  local pos = { x = start.x, y = start.y, z = start.z }
  local vel = v0 and { x = v0.x, y = v0.y, z = v0.z } or { x = 0, y = 0, z = 0 }
  local nose = len(vel) > 1 and unit(vel) or unit({ x = tgt.x - start.x, y = 0, z = tgt.z - start.z })
  local thr, t, nextRec = 0, 0, 0
  local pts = { pos }
  local tr = newTracker()
  local done, tDone = false, nil
  while t < C.SIM_TIME do
    local near = len(sub(tgt, pos)) < 150
    local dt = (g.phase ~= "CRUISE" or near) and 0.1 or 0.25
    local dir
    if not done then
      local fin
      dir, fin = guideStep(g, pos, vel, nose, dt)
      if fin then done, tDone = true, t end
    end
    thr = thr + ((done and 0 or g.throttle) - thr) * math.min(1, dt / 0.2)
    if dir then
      local dd = unit(dir)
      local ang = math.acos(clamp(dot(nose, dd), -1, 1))
      local rate = clamp(6 * thr + 0.02 * len(vel), 0.3, 3)
      nose = turnToward(nose, dd, math.min(rate * dt, ang * math.min(1, dt / 0.3)))
    end
    local acc = add(mul(nose, thr * C.THRUST_ACC), mul(vel, -C.DRAG))
    acc.y = acc.y - C.GRAV
    vel = add(vel, mul(acc, dt))
    pos = add(pos, mul(vel, dt))
    if g.phase == "IGNITION" and pos.y < start.y then
      pos.y = start.y
      vel = { x = 0, y = 0, z = 0 }
    end
    t = t + dt
    if g.phase == "CRUISE" or done then track(tr, contactPoint(pos, nose), tgt) end
    if t >= nextRec then
      pts[#pts + 1] = pos
      nextRec = t + 0.25
    end
    if done and (tr.hit or t - tDone > 4) then break end
  end
  pts[#pts + 1] = pos
  return { pts = pts, eta = tDone, miss = done and missOf(tr, start, tgt) or nil }
end

local function fluidAmount(side)
  local ok, t = pcall(peripheral.call, side, "tanks")
  if not ok or type(t) ~= "table" then return nil end
  local s = 0
  for _, f in pairs(t) do s = s + (f.amount or 0) end
  return s
end

local tel = { sub = false }

local function telemetry()
  while true do
    local ok, r = pcall(function() return sublevel.isInPlotGrid() end)
    tel.sub = ok and r or false
    tel.pos = nil
    if tel.sub then
      local ok2, p = pcall(function() return sublevel.getLogicalPose() end)
      if ok2 and p then tel.pos = v3(p.position) end
    end
    tel.eng = fluidAmount(C.SIDE_ENGINE)
    tel.tank = fluidAmount(C.SIDE_TANK)
    os.queueEvent("telemetry")
    sleep(0.5)
  end
end

local function fuel()
  while true do
    pcall(engine.pullFluid, C.SIDE_TANK)
    sleep(0.5)
  end
end

local function fmt(n)
  if not n then return "--" end
  return tostring(math.floor(n + 0.5))
end

local function fuelStr(n) return n and (fmt(n) .. " mB") or "--" end

local function fill(x, y, w, bg)
  term.setBackgroundColor(bg)
  term.setCursorPos(x, y)
  term.write(string.rep(" ", w))
end

local function text(x, y, s, fg, bg)
  term.setCursorPos(x, y)
  term.setTextColor(fg)
  term.setBackgroundColor(bg or colors.black)
  term.write(s)
end

local function clear()
  term.setCursorBlink(false)
  term.setBackgroundColor(colors.black)
  term.clear()
end

local function header(status, color)
  fill(1, 1, W, colors.gray)
  text(2, 1, "PLANE CONTROL", colors.white, colors.gray)
  text(W - #status, 1, status, color, colors.gray)
end

local PX, PY, PH = 28, 5, 9
C.PW = W - PX - 1
C.BITS = { 1, 2, 4, 8, 16 }
C.HEX = {}
for i = 0, 15 do C.HEX[2 ^ i] = string.format("%x", i) end
local function toBlit(c) return C.HEX[c] or "0" end

local function plot(start, tgt, cur, pred, trail)
  text(PX - 1, 3, "FLIGHT PROFILE", colors.yellow)
  fill(PX - 1, PY - 1, C.PW + 2, colors.gray)
  fill(PX - 1, PY + PH, C.PW + 2, colors.gray)
  for r = PY, PY + PH - 1 do
    fill(PX - 1, r, 1, colors.gray)
    fill(PX, r, C.PW, colors.black)
    fill(PX + C.PW, r, 1, colors.gray)
  end
  if not start or not tgt then
    local m = start and "NO TARGET" or "NO POSITION"
    text(PX + math.floor((C.PW - #m) / 2), PY + math.floor(PH / 2), m, colors.gray)
    return
  end
  local dx, dz = tgt.x - start.x, tgt.z - start.z
  local D = math.sqrt(dx * dx + dz * dz)
  local fx, fz = 0, 0
  if D > 1e-6 then fx, fz = dx / D, dz / D end
  local function downrange(p)
    local px, pz = p.x - start.x, p.z - start.z
    if D > 1e-6 then return px * fx + pz * fz end
    return math.sqrt(px * px + pz * pz)
  end
  local dlo, dhi = 0, math.max(D, 1)
  local lo = math.min(start.y, tgt.y)
  local hi = math.max(start.y + climb, tgt.y)
  local function grow(p)
    local d = downrange(p)
    dlo, dhi = math.min(dlo, d), math.max(dhi, d)
    lo, hi = math.min(lo, p.y), math.max(hi, p.y)
  end
  if pred then for _, p in ipairs(pred.pts) do grow(p) end end
  if trail then for _, p in ipairs(trail) do grow(p) end end
  if cur then grow(cur) end
  local pad = (hi - lo) * 0.05 + 0.5
  lo, hi = lo - pad, hi + pad
  local dpad = (dhi - dlo) * 0.03
  dlo, dhi = dlo - dpad, dhi + dpad
  local GW, GH = C.PW * 2, PH * 3
  local pc, pp = {}, {}
  local function map(p)
    local x = math.floor((downrange(p) - dlo) / (dhi - dlo) * (GW - 1) + 0.5)
    local y = math.floor((hi - p.y) / (hi - lo) * (GH - 1) + 0.5)
    return clamp(x, 0, GW - 1), clamp(y, 0, GH - 1)
  end
  local function dot2(x, y, c, pr)
    local i = y * GW + x
    if not pp[i] or pr >= pp[i] then pc[i], pp[i] = c, pr end
  end
  local function path(pts, c, pr)
    local ax, ay
    for _, p in ipairs(pts) do
      local bx, by = map(p)
      if ax then
        local n = math.max(math.abs(bx - ax), math.abs(by - ay), 1)
        for k = 0, n do
          dot2(math.floor(ax + (bx - ax) * k / n + 0.5), math.floor(ay + (by - ay) * k / n + 0.5), c, pr)
        end
      else
        dot2(bx, by, c, pr)
      end
      ax, ay = bx, by
    end
  end
  if pred then path(pred.pts, colors.cyan, 1) end
  if trail and #trail > 0 then
    local t2 = {}
    for i, p in ipairs(trail) do t2[i] = p end
    if cur then t2[#t2 + 1] = cur end
    path(t2, colors.orange, 2)
  end
  for cy = 0, PH - 1 do
    local ch, fg, bg = {}, {}, {}
    for cx = 0, C.PW - 1 do
      local col, pr, on = nil, 0, {}
      for k = 0, 5 do
        local i = (cy * 3 + math.floor(k / 2)) * GW + cx * 2 + k % 2
        on[k] = pc[i] ~= nil
        if on[k] and pp[i] >= pr then col, pr = pc[i], pp[i] end
      end
      if not col then
        ch[#ch + 1], fg[#fg + 1], bg[#bg + 1] = " ", "0", "f"
      else
        local n = 0
        if on[5] then
          for k = 0, 4 do if not on[k] then n = n + C.BITS[k + 1] end end
          ch[#ch + 1], fg[#fg + 1], bg[#bg + 1] = string.char(128 + n), "f", toBlit(col)
        else
          for k = 0, 4 do if on[k] then n = n + C.BITS[k + 1] end end
          ch[#ch + 1], fg[#fg + 1], bg[#bg + 1] = string.char(128 + n), toBlit(col), "f"
        end
      end
    end
    term.setCursorPos(PX, PY + cy)
    term.blit(table.concat(ch), table.concat(fg), table.concat(bg))
  end
  local function cell(p)
    local x, y = map(p)
    return PX + math.floor(x / 2), PY + math.floor(y / 3)
  end
  local sx, sy = cell(start)
  text(sx, sy, "^", colors.lime)
  local tx, ty = cell(tgt)
  text(tx, ty, "X", colors.red)
  if cur then
    local mx, my = cell(cur)
    text(mx, my, "o", colors.yellow)
  end
  if trail then
    text(PX, PY - 1, "PLAN", colors.cyan, colors.gray)
    text(PX + 5, PY - 1, "FLOWN", colors.orange, colors.gray)
  end
  local brg = (math.deg(atan2(dx, -dz)) + 360) % 360
  text(PX - 1, 15, "RANGE   " .. fmt(D) .. " m", colors.white)
  text(PX - 1, 16, "BEARING " .. string.format("%03d", math.floor(brg + 0.5) % 360), colors.white)
  if pred then
    local s = pred.miss and ("PLAN " .. fmt(pred.eta) .. "s MISS " .. string.format("%.1f", pred.miss.total)) or "PLAN NO ARRIVAL"
    text(PX - 1, 17, s, pred.miss and colors.lightGray or colors.red)
  end
end

local fields = { { label = "X", value = "" }, { label = "Y", value = "" }, { label = "Z", value = "" }, { label = "CLIMB", value = tostring(C.CLIMB) }, { label = "MODE", value = "WATER", choice = true } }
local active = 1
local msg = ""
local FX, FW = 9, 14

if fs.exists(C.SAVE) then
  local f = fs.open(C.SAVE, "r")
  local d = textutils.unserialize(f.readAll())
  f.close()
  if type(d) == "table" and d.x and d.y and d.z then
    fields[1].value, fields[2].value, fields[3].value = tostring(d.x), tostring(d.y), tostring(d.z)
  end
  if type(d) == "table" and d.climb then fields[4].value = tostring(d.climb) end
  if type(d) == "table" and d.mode then
    for _, m in ipairs(C.MODES) do if m == d.mode then fields[5].value = m end end
    if d.mode == "SPLASH" then fields[5].value = "WATER" end
  end
end

local function fieldRow(i) return 3 + i end

local function target()
  local x, y, z = tonumber(fields[1].value), tonumber(fields[2].value), tonumber(fields[3].value)
  if x and y and z then return { x = x, y = y, z = z } end
end

local function cycleMode(dir)
  local i = 1
  for k, m in ipairs(C.MODES) do if m == fields[5].value then i = k end end
  fields[5].value = C.MODES[(i - 1 + dir) % #C.MODES + 1]
end

local function climbValue()
  local n = tonumber(fields[4].value)
  if n and n >= 1 then return n end
end

local predKey, predVal, pendKey, pendAt

local function planKey(start, tgt, md, cl)
  if not start or not tgt then return nil end
  return table.concat({ fmt(start.x), fmt(start.y), fmt(start.z), tgt.x, tgt.y, tgt.z, md, cl }, ",")
end

local function currentPlan(start, tgt, md, cl, now)
  if md then return nil end
  local key = planKey(start, tgt, md, cl)
  if not key then return nil end
  if key == predKey then return predVal end
  if now or (pendKey == key and os.clock() - pendAt > 0.4) then
    predKey, predVal = key, predict(start, tgt, md, cl)
    return predVal
  end
  if pendKey ~= key then pendKey, pendAt = key, os.clock() end
  return nil
end

local function drawInput()
  clear()
  header(tel.sub and "SUB-LEVEL OK " or "NO SUB-LEVEL ", tel.sub and colors.lime or colors.red)
  text(2, 3, "TARGET", colors.yellow)
  for i, f in ipairs(fields) do
    local r = fieldRow(i)
    local on = i == active
    local bg = on and colors.lightGray or colors.gray
    text(2, r, f.label, colors.white)
    fill(FX, r, FW, bg)
    text(FX + 1, r, f.value, on and colors.black or colors.white, bg)
  end
  local p = tel.pos
  text(2, 10, "POSITION", colors.yellow)
  text(2, 11, "X " .. fmt(p and p.x), colors.white)
  text(2, 12, "Y " .. fmt(p and p.y), colors.white)
  text(2, 13, "Z " .. fmt(p and p.z), colors.white)
  text(2, 15, "ENGINE " .. fuelStr(tel.eng), colors.white)
  text(2, 16, "TANK   " .. fuelStr(tel.tank), colors.white)
  climb = climbValue() or C.CLIMB
  local tgt = target()
  local pl = currentPlan(p, tgt, fields[5].value, climb)
  plot(p, tgt, nil, pl, nil)
  if p and tgt then
    text(PX - 1, 17, fields[5].value == "SHUTTLE" and "WAITS FOR RELEASE" or "STAND IT ON ITS TAIL", colors.gray)
  elseif p and tgt and not pl then
    text(PX - 1, 17, "PLANNING...", colors.gray)
  end
  if msg ~= "" then text(2, 17, msg, colors.red) end
  fill(2, 18, 12, colors.red)
  text(5, 18, "LAUNCH", colors.white, colors.red)
  fill(15, 18, 9, colors.gray)
  text(17, 18, "CLEAR", colors.white, colors.gray)
  text(2, 19, "TAB next  ENTER ok  BKSP del  SPACE mode", colors.gray)
  local r = fieldRow(active)
  term.setCursorPos(FX + 1 + #fields[active].value, r)
  term.setTextColor(colors.black)
  term.setBackgroundColor(colors.lightGray)
  term.setCursorBlink(not fields[active].choice)
end

local function tryLaunch()
  local t = target()
  if not t then msg = "ENTER ALL THREE COORDS" return nil end
  local c = climbValue()
  if not c then msg = "CLIMB MUST BE 1 OR MORE" return nil end
  if not tel.sub then msg = "NOT ON A SUB-LEVEL" return nil end
  climb = c
  mode = fields[5].value
  local f = fs.open(C.SAVE, "w")
  f.write(textutils.serialize({ x = t.x, y = t.y, z = t.z, climb = c, mode = mode }))
  f.close()
  msg = ""
  return t
end

local function inputScreen()
  while true do
    drawInput()
    local e, a, b, c = os.pullEvent()
    if e == "char" and fields[active].choice then
      if a == " " then cycleMode(1) end
    elseif e == "char" then
      local v = fields[active].value
      if a:match("%d") and #v < 12 then
        fields[active].value = v .. a
      elseif a == "-" and #v == 0 and active < 4 then
        fields[active].value = "-"
      end
      msg = ""
    elseif e == "key" then
      if a == keys.backspace and not fields[active].choice then
        fields[active].value = fields[active].value:sub(1, -2)
      elseif (a == keys.left or a == keys.right) and fields[active].choice then
        cycleMode(a == keys.right and 1 or -1)
      elseif a == keys.tab or a == keys.down then
        active = active % #fields + 1
      elseif a == keys.up then
        active = (active - 2) % #fields + 1
      elseif a == keys.enter or a == keys.numPadEnter then
        if active < #fields then
          active = active + 1
        else
          local t = tryLaunch()
          if t then return t end
        end
      end
    elseif e == "mouse_click" then
      for i = 1, #fields do
        if c == fieldRow(i) and b >= 2 and b < FX + FW then
          if active == i and fields[i].choice then cycleMode(1) end
          active = i
        end
      end
      if c == 18 and b >= 2 and b <= 13 then
        local t = tryLaunch()
        if t then return t end
      elseif c == 18 and b >= 15 and b <= 23 then
        for i = 1, 3 do fields[i].value = "" end
        fields[4].value = tostring(C.CLIMB)
        active, msg = 1, ""
      end
    end
  end
end

C.FONT = {
  ["0"] = { "###", "# #", "# #", "# #", "###" },
  ["1"] = { " # ", "## ", " # ", " # ", "###" },
  ["2"] = { "###", "  #", "###", "#  ", "###" },
  ["3"] = { "###", "  #", " ##", "  #", "###" },
  ["4"] = { "# #", "# #", "###", "  #", "  #" },
  ["5"] = { "###", "#  ", "###", "  #", "###" },
  ["6"] = { "###", "#  ", "###", "# #", "###" },
  ["7"] = { "###", "  #", "  #", "  #", "  #" },
  ["8"] = { "###", "# #", "###", "# #", "###" },
  ["9"] = { "###", "# #", "###", "  #", "###" },
}

local function bigDigit(ch, x, y, color)
  local g = C.FONT[ch]
  for r = 1, 5 do
    for i = 1, 3 do
      if g[r]:sub(i, i) == "#" then fill(x + (i - 1) * 2, y + r - 1, 2, color) end
    end
  end
end

local function drawCountdown(left, tgt)
  clear()
  header("ARMED ", colors.red)
  local m = "LAUNCH IN"
  text(math.floor((W - #m) / 2) + 1, 4, m, colors.yellow)
  bigDigit(tostring(math.min(9, math.ceil(left))), math.floor((W - 6) / 2) + 1, 6, colors.red)
  local bw = W - 10
  local done = math.floor(bw * clamp((C.DELAY - left) / C.DELAY, 0, 1) + 0.5)
  fill(6, 12, bw, colors.gray)
  if done > 0 then fill(6, 12, done, colors.orange) end
  local t = "TGT " .. fmt(tgt.x) .. " " .. fmt(tgt.y) .. " " .. fmt(tgt.z)
  text(math.floor((W - #t) / 2) + 1, 14, t, colors.white)
  local cl = "CLIMB " .. fmt(climb) .. "  " .. mode
  text(math.floor((W - #cl) / 2) + 1, 15, cl, colors.white)
  local ax = math.floor((W - 11) / 2) + 1
  fill(ax, 17, 11, colors.gray)
  text(ax + 3, 17, "ABORT", colors.white, colors.gray)
  local h = "BKSP to abort"
  text(math.floor((W - #h) / 2) + 1, 19, h, colors.gray)
end

local function countdown(tgt)
  local t0 = os.clock()
  local timer = os.startTimer(0.1)
  local ax = math.floor((W - 11) / 2) + 1
  while true do
    local left = C.DELAY - (os.clock() - t0)
    if left <= 0 then return true end
    drawCountdown(left, tgt)
    local e, a, b, c = os.pullEvent()
    if e == "timer" and a == timer then
      timer = os.startTimer(0.1)
    elseif e == "key" and a == keys.backspace then
      return false
    elseif e == "mouse_click" and c == 17 and b >= ax and b < ax + 11 then
      return false
    end
  end
end

local function drawArmed(tgt)
  clear()
  header("ARMED ", colors.orange)
  local m = "WAITING FOR LAUNCH"
  text(math.floor((W - #m) / 2) + 1, 6, m, colors.yellow)
  local t = "TGT " .. fmt(tgt.x) .. " " .. fmt(tgt.y) .. " " .. fmt(tgt.z)
  text(math.floor((W - #t) / 2) + 1, 9, t, colors.white)
  local cl = "CLIMB " .. fmt(climb) .. "  " .. mode
  text(math.floor((W - #cl) / 2) + 1, 10, cl, colors.white)
  local h = "BKSP to disarm here"
  text(math.floor((W - #h) / 2) + 1, 19, h, colors.gray)
end

local function remoteWait(tgt)
  local ax = math.floor((W - 11) / 2) + 1
  while true do
    if C.REMOTE.state.aborted then return false end
    local left = C.REMOTE.launchIn()
    if left and left <= 0 then return true end
    if left then drawCountdown(math.min(left, C.DELAY), tgt) else drawArmed(tgt) end
    local timer = os.startTimer(0.1)
    repeat
      local e, a, b, c = os.pullEvent()
      if e == "key" and a == keys.backspace then
        C.REMOTE.state.localAbort = true
        return false
      elseif e == "mouse_click" and left and c == 17 and b >= ax and b < ax + 11 then
        C.REMOTE.state.localAbort = true
        return false
      end
    until e == "timer" and a == timer
  end
end

local function drawRelease(tgt)
  clear()
  header("ARMED ", colors.orange)
  local m = "WAITING FOR RELEASE"
  text(math.floor((W - #m) / 2) + 1, 5, m, colors.yellow)
  local t = "TGT " .. fmt(tgt.x) .. " " .. fmt(tgt.y) .. " " .. fmt(tgt.z)
  text(math.floor((W - #t) / 2) + 1, 8, t, colors.white)
  local p = tel.pos
  local a = "ALT " .. fmt(p and p.y)
  text(math.floor((W - #a) / 2) + 1, 9, a, colors.white)
  local l = {
    "Starts when the shuttle is let go:",
    C.REL_ON_MASS and "- the craft it rides on is left behind" or "",
    C.REL_FREEFALL and "- it starts falling freely" or "",
    C.REL_SIDE and ("- a redstone signal on " .. C.REL_SIDE:upper()) or "- a redstone signal on any side",
    "- ENTER to start now",
  }
  for k, v in ipairs(l) do text(3, 11 + k, v, colors.lightGray) end
  local h = "BKSP to disarm"
  text(math.floor((W - #h) / 2) + 1, 19, h, colors.gray)
end

local function waitRelease(tgt)
  local result
  local function watch()
    local _, _, _, v0, _, m0 = tick(nil, nil, nil, true)
    local t0, last, lastVy, fall = os.clock(), os.clock(), v0.y, 0
    local nextDraw = 0
    while true do
      local _, _, _, vel, _, m = tick(nil, nil, nil, true)
      local now = os.clock()
      local dt = now - last
      last = now
      local why
      if C.REL_ON_MASS and m0 and m and m < m0 * C.REL_MASS then why = "SEPARATED (MASS " .. fmt(m0) .. " -> " .. fmt(m) .. ")" end
      for _, side in ipairs(C.REL_SIDE and { C.REL_SIDE } or redstone.getSides()) do
        if not why and redstone.getInput(side) then why = "REDSTONE " .. side:upper() end
      end
      if dt > 0 then
        if (vel.y - lastVy) / dt < -C.REL_FALL_ACC then fall = fall + dt else fall = 0 end
        lastVy = vel.y
      end
      if not why and C.REL_FREEFALL and now - t0 > C.REL_MIN_T and fall >= C.REL_FALL_T then why = "FREE FALL" end
      if why then
        result = why
        return
      end
      if now >= nextDraw then
        drawRelease(tgt)
        nextDraw = now + 0.5
      end
    end
  end
  local function keysIn()
    while true do
      local _, k = os.pullEvent("key")
      if k == keys.enter or k == keys.numPadEnter then
        result = "MANUAL"
        return
      elseif k == keys.backspace then
        result = false
        if C.REMOTE then C.REMOTE.state.localAbort = true end
        return
      end
    end
  end
  parallel.waitForAny(watch, keysIn)
  return result
end

local fl = { phase = "IGNITION", dx = 0, dy = 0, dz = 0, trail = {} }

local function drawFlight(start, tgt)
  clear()
  local colorsFor = { IGNITION = colors.yellow, ASCENT = colors.orange, SEPARATE = colors.lightBlue, CRUISE = colors.cyan, VTAKE = colors.orange, PITCHOVER = colors.yellow, APPROACH = colors.magenta, FLARE = colors.lime, TOUCHDOWN = colors.red, ABORT = colors.red, CUT = colors.red }
  header(fl.phase .. " ", colorsFor[fl.phase] or colors.white)
  local p = fl.pos
  text(2, 3, "FLIGHT " .. mode, colors.yellow)
  text(2, 5, "ALT    " .. fmt(p and p.y), colors.white)
  text(2, 6, "SPEED  " .. fmt(fl.speed) .. " m/s", colors.white)
  text(2, 7, "DIST   " .. fmt(fl.dist) .. " m", colors.white)
  local eta = (fl.speed and fl.speed > 0.5 and fl.dist) and (fmt(fl.dist / fl.speed) .. " s") or "--"
  text(2, 8, "ETA    " .. eta, colors.white)
  text(2, 9, "THRUST " .. (fl.thr and (fmt(fl.thr * 100) .. "%" .. " " .. (fl.tmode or "RAMP")) or "--"), colors.white)
  text(2, 10, "GIMBAL", colors.yellow)
  text(2, 11, "X " .. (fl.gx and string.format("%+.2f", fl.gx) or "--"), colors.white)
  text(2, 12, "Y " .. (fl.gy and string.format("%+.2f", fl.gy) or "--"), colors.white)
  text(2, 13, "BODY dx" .. string.format("%+.2f", fl.dx) .. " dy" .. string.format("%+.2f", fl.dy) .. " dz" .. string.format("%+.2f", fl.dz), colors.gray)
  text(2, 14, "POS " .. fmt(p and p.x) .. " " .. fmt(p and p.y) .. " " .. fmt(p and p.z), colors.gray)
  text(2, 15, "ENGINE " .. fuelStr(tel.eng), colors.white)
  text(2, 16, "TANK   " .. fuelStr(tel.tank), colors.white)
  plot(start, tgt, p, fl.pred, fl.trail)
  local r = fl.result
  if r then
    text(2, 17, string.format("MISS %.1f  L%+.1f R%+.1f", r.total, r.long, r.right), r.total <= 2 and colors.lime or colors.orange)
  end
  text(2, 18, "TGT " .. fmt(tgt.x) .. " " .. fmt(tgt.y) .. " " .. fmt(tgt.z), colors.gray)
end

local function writeLog(tgt, r, speed, t)
  local ok, d = pcall(os.date, "%Y-%m-%d %H:%M")
  local f = fs.open(C.LOG, "a")
  if not f then return end
  f.writeLine(string.format("%s %s TGT %d %d %d MISS %.1f LONG %+.1f RIGHT %+.1f SPEED %.0f TIME %.1f%s",
    ok and d or "?", mode, math.floor(tgt.x), math.floor(tgt.y), math.floor(tgt.z),
    r.total, r.long, r.right, speed or 0, t or 0, r.crossed and "" or " CLOSEST"))
  f.close()
end

local function tune()
  local ok, m = pcall(sublevel.getMass)
  if ok and type(m) == "number" and m > 0 then C.THRUST_ACC = C.THRUST_FORCE / m end
  local ok2, it = pcall(sublevel.getInertiaTensor)
  axisInfo = { fwd = "z" }
  if ok2 and type(it) == "table" then
    local function el(i, k) local r = it[i] return type(r) == "table" and (r[k] or r[({ "x", "y", "z" })[k]]) or nil end
    local ix, iy, iz = el(1, 1), el(2, 2), el(3, 3)
    if type(ix) == "number" and type(iy) == "number" and type(iz) == "number" then
      axisInfo.ix, axisInfo.iy, axisInfo.iz = ix, iy, iz
      axisInfo.tilted = iy < math.max(ix, iz)
      axisInfo.fwd = ix < iz and "x" or "z"
      inertiaK = clamp(math.max(ix, iz) / C.REF_I, C.GAIN_MIN, C.GAIN_MAX)
    end
  end
  return ok and m or nil
end

local function flight(tgt, pred, why)
  local mass = tune()
  local start, w0, u0, v0 = tick()
  local ax = axisInfo.fwd == "x" and { x = 1, y = 0, z = 0 } or { x = 0, y = 0, z = 1 }
  local aw = rotate(w0, u0, ax)
  local sign = 1
  local refuse
  if axisInfo.tilted then
    refuse = "ASSEMBLED TILTED: RE-ASSEMBLE LEVEL"
  elseif mode == "SHUTTLE" then
    if len(v0) > 2 and dot(aw, v0) < 0 then sign = -1 end
  else
    if aw.y < 0 then sign = -1 end
    if math.abs(aw.y) < math.sin(math.rad(C.VT_PITCH)) then refuse = "STAND IT ON ITS TAIL" end
  end
  FWD.x, FWD.y, FWD.z = ax.x * sign, 0, ax.z * sign
  RGT.x, RGT.y, RGT.z = FWD.z, 0, -FWD.x
  if refuse then
    fl.phase = refuse
    drawFlight(start, tgt)
    sleep(5)
    return
  end
  local g = newGuide(start, tgt, mode, climb)
  local tr = newTracker()
  fl.trail, fl.pred, fl.result = {}, pred, nil
  local function guide()
    local t0 = os.clock()
    local last, nextTrail, nextLog = t0, t0, t0
    local flog = fs.open(C.FLIGHT_LOG, "w")
    if flog then
      flog.writeLine(string.format("%s TGT %d %d %d CLIMB %d%s", mode, math.floor(tgt.x), math.floor(tgt.y), math.floor(tgt.z), climb, why and (" START " .. why) or ""))
      flog.writeLine(string.format("FROM %.1f %.1f %.1f VEL %.1f %.1f %.1f MASS %s TACC %.1f GAIN %.2f NOSE %s%s I %s %s %s", start.x, start.y, start.z, v0.x, v0.y, v0.z, tostring(mass), C.THRUST_ACC, inertiaK,
        (FWD.x + FWD.z) > 0 and "+" or "-", axisInfo.fwd, tostring(axisInfo.ix), tostring(axisInfo.iy), tostring(axisInfo.iz)))
      flog.writeLine("t phase thr speed alt dist dh err_deg gx gy spin wx wy wz steps pitch pitch_cmd vy")
    end
    local cT, cX, cY = g.throttle, 0, 0
    local badT, stillT = 0, 0
    while true do
      if fl.abort then
        if flog then flog.close() flog = nil end
        cutEngine()
        fl.thr, fl.gx, fl.gy, fl.phase = 0, 0, 0, "ABORT"
        return
      end
      local pos, w, u, vel, om = tick(cT, cX, cY)
      local now = os.clock()
      local dt = clamp(now - last, 0.01, 0.5)
      last = now
      local nose = rotate(w, u, FWD)
      g.upb = rotate(w, u, UP)
      local dir, done = guideStep(g, pos, vel, nose, dt)
      fl.phase, fl.tmode, fl.thr = g.phase, g.tmode, g.throttle
      fl.pos, fl.speed, fl.dist = pos, len(vel), len(sub(tgt, pos))
      if g.phase == "APPROACH" or g.phase == "FLARE" then track(tr, pos, tgt) end
      if now >= nextTrail then
        fl.trail[#fl.trail + 1] = pos
        nextTrail = now + 0.25
      end
      if dir then
        local d = unit(toBody(w, u, dir))
        fl.dx, fl.dy, fl.dz = d.x, d.y, d.z
        fl.gx, fl.gy = steer(dir, w, u, om, dt, g.throttle)
      else
        fl.gx, fl.gy = 0, 0
      end
      local errNow = dir and math.deg(math.acos(clamp(dot(nose, unit(dir)), -1, 1))) or 0
      local low = pos.y < math.min(start.y, tgt.y) + C.CUT_LOW
      if low and errNow > C.CUT_ERR then badT = badT + dt else badT = 0 end
      if g.phase ~= "VTAKE" and len(vel) < C.CUT_STILL_V then stillT = stillT + dt else stillT = 0 end
      if not done and (badT > C.CUT_ERR_T or stillT > C.CUT_STILL_T) then
        g.phase, g.throttle, g.tmode = "CUT", 0, badT > C.CUT_ERR_T and "TILT" or "STILL"
        fl.phase, fl.tmode = g.phase, g.tmode
        if flog then flog.writeLine(string.format("CUT %s %.2f", g.tmode, now - t0)) end
        done = true
      end
      if flog and now >= nextLog then
        local err = errNow
        local dhx, dhz = tgt.x - pos.x, tgt.z - pos.z
        flog.writeLine(string.format("%.2f %s %.2f %.1f %.1f %.1f %.1f %.1f %+.2f %+.2f %.2f %+.2f %+.2f %+.2f %d %+.1f %+.1f %+.1f",
          now - t0, g.phase, g.throttle, len(vel), pos.y, len(sub(tgt, pos)), math.sqrt(dhx * dhx + dhz * dhz), err, fl.gx, fl.gy, len(om), om.x, om.y, om.z,
          g.steps or 0, math.deg(math.asin(clamp(nose.y, -1, 1))), math.deg(g.pitchCmd or 0), vel.y))
        flog.flush()
        nextLog = now + 0.1
      end
      if done then
        if flog then flog.close() flog = nil end
        cutEngine()
        fl.thr, fl.gx, fl.gy = 0, 0, 0
        track(tr, contactPoint(pos, nose), tgt)
        local flareOff
        if g.phase == "TOUCHDOWN" and C.FLARE_SIDE then
          pcall(redstone.setOutput, C.FLARE_SIDE, true)
          flareOff = os.clock() + C.FLARE_PULSE
        end
        local cutSpeed, cutTime = len(vel), now - t0
        local stop, still = os.clock() + 4, 0
        while os.clock() < stop and not tr.hit do
          if flareOff and os.clock() >= flareOff then
            pcall(redstone.setOutput, C.FLARE_SIDE, false)
            flareOff = nil
          end
          local ok, p2, w2, u2, v2 = pcall(tick)
          if not ok then break end
          track(tr, p2, tgt)
          fl.pos, fl.speed, fl.dist = p2, len(v2), len(sub(tgt, p2))
          fl.trail[#fl.trail + 1] = p2
          still = len(v2) < 0.5 and still + 1 or 0
          if still > 10 then break end
        end
        if flareOff then
          sleep(math.max(0, flareOff - os.clock()))
          pcall(redstone.setOutput, C.FLARE_SIDE, false)
        end
        fl.result = missOf(tr, start, tgt)
        if fl.result then pcall(writeLog, tgt, fl.result, cutSpeed, cutTime) end
        if C.REMOTE and fl.result then pcall(C.REMOTE.result, fl.result, cutSpeed, cutTime) end
        return
      end
      cT, cX, cY = g.throttle, fl.gx, fl.gy
    end
  end
  local function display()
    while true do
      drawFlight(start, tgt)
      sleep(0.2)
    end
  end
  local function uplink()
    while true do
      sleep(1)
      local p = fl.pos or start
      local sp = fl.speed or 0
      local ok, aborted = pcall(C.REMOTE.report, {
        phase = fl.phase, alt = p.y, speed = sp, dist = fl.dist or 0,
        eta = (sp > 0.5 and fl.dist) and fl.dist / sp or 0, thrust = (fl.thr or 0) * 100,
        gx = fl.gx or 0, gy = fl.gy or 0, engine_mb = tel.eng or 0, tank_mb = tel.tank or 0,
        x = p.x, y = p.y, z = p.z
      })
      if ok and aborted then fl.abort = true end
    end
  end
  fl.abort = false
  if C.REMOTE then
    parallel.waitForAny(guide, display, uplink)
  else
    parallel.waitForAny(guide, display)
  end
  cutEngine()
  drawFlight(start, tgt)
end

local function main()
  cutEngine()
  if C.REMOTE then
    local m = C.REMOTE.mission
    mode, climb = m.mode == "SHUTTLE" and "SHUTTLE" or "WATER", m.climb or C.CLIMB
    local rt = { x = m.x, y = m.y, z = m.z }
    local w0 = os.clock()
    while not tel.pos and os.clock() - w0 < 5 do sleep(0.1) end
    local pred = tel.pos and currentPlan(tel.pos, rt, mode, climb, true) or nil
    if pred and pred.eta then pcall(C.REMOTE.plan, pred.eta) end
    if remoteWait(rt) then
      local why
      if mode == "SHUTTLE" then
        why = waitRelease(rt)
        if not why then return end
      end
      C.REMOTE.state.flown = true
      flight(rt, pred, why)
    end
    return
  end
  while true do
    local tgt = inputScreen()
    if mode == "SHUTTLE" then
      local why = waitRelease(tgt)
      if why then
        flight(tgt, nil, why)
        return
      end
    elseif countdown(tgt) then
      flight(tgt, currentPlan(tel.pos, tgt, mode, climb, true))
      return
    end
  end
end

local ok, err = pcall(parallel.waitForAny, main, telemetry, fuel)
cutEngine()
if C.FLARE_SIDE then pcall(redstone.setOutput, C.FLARE_SIDE, false) end
term.setCursorBlink(false)
term.setBackgroundColor(colors.black)
term.setTextColor(colors.white)
term.setCursorPos(1, H)
if not ok then print(err) end
