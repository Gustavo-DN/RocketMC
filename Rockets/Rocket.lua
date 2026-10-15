local REMOTE = ...
if type(REMOTE) ~= "table" then REMOTE = nil end
local C = {
  SIDE_ENGINE = "bottom",
  SIDE_TANK = "top",
  CLIMB = 50,
  DELAY = 5,
  VMAX = 1.0,
  KP = 5,
  KD = 3,
  RATE_LEAD = 0.2,
  RATE_TAU = 0.15,
  RATE_ACC_MAX = 12,
  ATT_KI = 3,
  ATT_IWIN = 20,
  ATT_IMAX = 1.0,
  MAP_X = 3.37,
  MAP_Y = -1.95,
  MAP_CROSS = -0.94,
  MAP_THR = 2 / 15,
  MAX_THRUST = 0.40,
  CRUISE_THR = 0.70,
  START_THRUST = 0.15,
  THR_RAMP = 0.05,
  THR_MARGIN = 0.03,
  LIFTOFF_VY = 0.5,
  ASCENT_TWR = 2,
  ASCENT_STALL_H = 5,
  TURN_RATE = 8,
  ERR_MAX = 1.2,
  FF_TAU = 0.2,
  FF_MAX = 1.5,
  FF_JUMP = 0.94,
  HIT_RADIUS = 3,
  CUT_TIME = 0.1,
  STUCK_V = 2,
  STUCK_T = 1,
  NOSE_OFFSET = 3,
  CUT_ERR = 100,
  CUT_ERR_T = 2,
  CUT_LOW = 15,
  CUT_STILL_V = 1,
  CUT_STILL_T = 3,
  GRAV = 10.5,
  DRAG = 0.13,
  THRUST_FORCE = 427,
  THRUST_ACC = 77.6,
  REF_I = 15.57,
  GAIN_MIN = 0.25,
  GAIN_MAX = 4,
  FAST_THR = 1.0,
  SLOW_DIST = 300,
  WATER_RAMP = 0.3,
  SHUTTLE_RAMP = 1.0,
  NAV = 3,
  HOLD_TGO = 0.4,
  SEP_DELAY = 0.5,
  REL_MASS = 0.7,
  REL_FREEFALL = true,
  REL_FALL_ACC = 6,
  REL_FALL_T = 0.4,
  REL_MIN_T = 1,
  SIM_TIME = 240,
  SAVE = "rocket.target",
  LOG = "rocket.log",
  FLIGHT_LOG = "flight.log",
  MODES = { "WATER", "SHUTTLE" },
}
local inertiaK = 1
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
  local p, v, o, m, actualX, actualY
  local fns = {
    function() p = sublevel.getLogicalPose() end,
    function() v = sublevel.getLinearVelocity() end,
    function() o = sublevel.getAngularVelocity() end,
  }
  if thr then
    fns[#fns + 1] = function() engine.setThrustNormalized(thr) end
    fns[#fns + 1] = function() engine.setVector(gx, gy) end
    if engine.getVectorX then fns[#fns + 1] = function() local ok, x = pcall(engine.getVectorX) if ok then actualX = x end end end
    if engine.getVectorY then fns[#fns + 1] = function() local ok, y = pcall(engine.getVectorY) if ok then actualY = y end end end
  end
  if withMass then fns[#fns + 1] = function() m = sublevel.getMass() end end
  parallel.waitForAll(table.unpack(fns))
  local w, u = quat(p.orientation)
  return v3(p.position), w, u, v3(v), v3(o), m, actualX, actualY
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

local attI = { x = 0, z = 0 }
local ff = { w = { x = 0, y = 0, z = 0 } }
local rates = { acc = { x = 0, y = 0, z = 0 } }

local function steer(dirW, w, u, om, dt, thr)
  local du = unit(dirW)
  if ff.prev and dt and dt > 0 and dot(ff.prev, du) > C.FF_JUMP then
    local a = math.min(1, dt / C.FF_TAU)
    ff.w = add(ff.w, mul(sub(mul(cross(ff.prev, du), 1 / dt), ff.w), a))
    local m = len(ff.w)
    if m > C.FF_MAX then ff.w = mul(ff.w, C.FF_MAX / m) end
  else
    ff.w = { x = 0, y = 0, z = 0 }
  end
  ff.prev = du
  local wb = toBody(w, u, ff.w)
  local d = unit(toBody(w, u, dirW))
  local axis = { x = d.z, y = 0, z = -d.x }
  local s = len(axis)
  local e = { x = 0, y = 0, z = 0 }
  if s > 1e-9 then
    e = mul(axis, math.min(atan2(s, d.y), C.ERR_MAX) / s)
  elseif d.y < 0 then
    e = { x = C.ERR_MAX, y = 0, z = 0 }
  end
  if C.ATT_KI > 0 and dt and len(e) < math.rad(C.ATT_IWIN) then
    attI.x = clamp(attI.x + e.x * dt, -C.ATT_IMAX, C.ATT_IMAX)
    attI.z = clamp(attI.z + e.z * dt, -C.ATT_IMAX, C.ATT_IMAX)
  end
  local tq = math.floor(clamp(thr or 0, 0, 1) * 15 + 1e-9) / 15
  if tq <= 0 then return 0, 0 end
  local scale = tq / C.MAP_THR / inertiaK
  if rates.prev then
    local a = math.min(1, dt / C.RATE_TAU)
    rates.acc = add(rates.acc, mul(sub(mul(sub(om, rates.prev), 1 / dt), rates.acc), a))
  end
  rates.prev = om
  local lead = { x = om.x + C.RATE_LEAD * clamp(rates.acc.x, -C.RATE_ACC_MAX, C.RATE_ACC_MAX), y = om.y, z = om.z + C.RATE_LEAD * clamp(rates.acc.z, -C.RATE_ACC_MAX, C.RATE_ACC_MAX) }
  local ax = C.KP * e.x + C.ATT_KI * attI.x - C.KD * (lead.x - wb.x)
  local az = C.KP * e.z + C.ATT_KI * attI.z - C.KD * (lead.z - wb.z)
  local gy = ax / (C.MAP_Y * scale)
  local gx = (az / scale - C.MAP_CROSS * gy) / C.MAP_X
  local limit = math.max(1, math.abs(gx) / C.VMAX, math.abs(gy) / C.VMAX)
  gx, gy = gx / limit, gy / limit
  return gx, gy
end

local function newGuide(start, tgt, md, cl)
  return {
    start = start, tgt = tgt, mode = md, climb = cl, climbY = start.y + cl,
    phase = md == "SHUTTLE" and "SEPARATE" or "IGNITION", tmode = md == "SHUTTLE" and "COAST" or "RAMP",
    throttle = md == "SHUTTLE" and 0 or C.START_THRUST, sepT = 0, stuck = 0,
    cmd = { x = 0, y = 1, z = 0 },
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
  if g.phase == "IGNITION" then
    if vel.y > C.LIFTOFF_VY then
      g.throttle = math.min(math.max(g.throttle + C.THR_MARGIN, C.ASCENT_TWR * C.GRAV / C.THRUST_ACC), math.max(C.MAX_THRUST, C.ASCENT_TWR * C.GRAV / C.THRUST_ACC))
      g.phase, g.tmode = "ASCENT", "HOLD"
    else
      g.throttle = math.min(g.throttle + C.THR_RAMP * dt, C.MAX_THRUST)
      return nil
    end
  elseif g.phase == "SEPARATE" then
    g.sepT = g.sepT + dt
    if g.sepT < C.SEP_DELAY then return nil end
    g.phase, g.tmode, g.throttle = "CRUISE", "RAMP", C.START_THRUST
  end
  local r = sub(tgt, add(pos, mul(nose, C.NOSE_OFFSET)))
  local dn = len(r)
  local vc = dn > 1e-6 and dot(vel, r) / dn or 0
  local closing = g.lastDn and dn < g.lastDn
  g.lastDn = dn
  local final = g.phase == "CRUISE"
  if final and pos.y < math.min(g.start.y, tgt.y) + C.CUT_LOW and len(vel) < C.STUCK_V then g.stuck = g.stuck + dt else g.stuck = 0 end
  if dn < C.HIT_RADIUS or g.stuck > C.STUCK_T or (final and ((vc > 1 and dn / vc < C.CUT_TIME) or (closing == false and dn < 20))) then
    g.phase, g.throttle, g.tmode = "TOUCHDOWN", 0, "OFF"
    return nil, true
  end
  local los = sub(tgt, pos)
  if g.phase == "ASCENT" and (pos.y >= g.climbY or (pos.y > g.start.y + C.ASCENT_STALL_H and vel.y < C.LIFTOFF_VY)) then g.phase = "CRUISE" end
  if g.phase == "ASCENT" then
    local f = clamp((pos.y - g.start.y) / g.climb, 0, 1)
    f = f * f
    local want = unit(add(mul(UP, 1 - f), mul(unit(los), f)))
    g.cmd = turnToward(g.cmd, want, math.rad(C.TURN_RATE) * dt)
    return g.cmd
  end
  local want = len(los) > C.SLOW_DIST and C.FAST_THR or C.CRUISE_THR
  if g.throttle < want then
    g.throttle = math.min(g.throttle + (g.mode == "SHUTTLE" and C.SHUTTLE_RAMP or C.WATER_RAMP) * dt, want)
  else
    g.throttle = want
  end
  g.tmode = "CRUISE"
  return diveDir(g, pos, vel, nose)
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
  local nose = { x = 0, y = 1, z = 0 }
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
  text(2, 1, "ROCKET CONTROL", colors.white, colors.gray)
  text(W - #status, 1, status, color, colors.gray)
end

local PX, PY, PH = 28, 5, 9
local PW = W - PX - 1
local BITS = { 1, 2, 4, 8, 16 }
local HEX = {}
for i = 0, 15 do HEX[2 ^ i] = string.format("%x", i) end
local function toBlit(c) return HEX[c] or "0" end

local function plot(start, tgt, cur, pred, trail)
  text(PX - 1, 3, "FLIGHT PROFILE", colors.yellow)
  fill(PX - 1, PY - 1, PW + 2, colors.gray)
  fill(PX - 1, PY + PH, PW + 2, colors.gray)
  for r = PY, PY + PH - 1 do
    fill(PX - 1, r, 1, colors.gray)
    fill(PX, r, PW, colors.black)
    fill(PX + PW, r, 1, colors.gray)
  end
  if not start or not tgt then
    local m = start and "NO TARGET" or "NO POSITION"
    text(PX + math.floor((PW - #m) / 2), PY + math.floor(PH / 2), m, colors.gray)
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
  local GW, GH = PW * 2, PH * 3
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
    for cx = 0, PW - 1 do
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
          for k = 0, 4 do if not on[k] then n = n + BITS[k + 1] end end
          ch[#ch + 1], fg[#fg + 1], bg[#bg + 1] = string.char(128 + n), "f", toBlit(col)
        else
          for k = 0, 4 do if on[k] then n = n + BITS[k + 1] end end
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
  if md == "SHUTTLE" then return nil end
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
  if p and tgt and fields[5].value == "SHUTTLE" then
    text(PX - 1, 17, "PLAN AFTER RELEASE", colors.gray)
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

local FONT = {
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
  local g = FONT[ch]
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
    if REMOTE.state.aborted then return false end
    local left = REMOTE.launchIn()
    if left and left <= 0 then return true end
    if left then drawCountdown(math.min(left, C.DELAY), tgt) else drawArmed(tgt) end
    local timer = os.startTimer(0.1)
    repeat
      local e, a, b, c = os.pullEvent()
      if e == "key" and a == keys.backspace then
        REMOTE.state.localAbort = true
        return false
      elseif e == "mouse_click" and left and c == 17 and b >= ax and b < ax + 11 then
        REMOTE.state.localAbort = true
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
    "- the craft it rides on is left behind",
    C.REL_FREEFALL and "- it starts falling freely" or "",
    "- a redstone signal on any side",
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
      if m0 and m and m < m0 * C.REL_MASS then why = "SEPARATED (MASS " .. fmt(m0) .. " -> " .. fmt(m) .. ")" end
      for _, side in ipairs(redstone.getSides()) do
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
        if REMOTE then REMOTE.state.localAbort = true end
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
  local colorsFor = { IGNITION = colors.yellow, ASCENT = colors.orange, SEPARATE = colors.lightBlue, CRUISE = colors.cyan, TOUCHDOWN = colors.red, ABORT = colors.red, CUT = colors.red }
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
  if ok2 and type(it) == "table" then
    local a, c = it[1], it[3]
    local ia = type(a) == "table" and (a[1] or a.x)
    local ic = type(c) == "table" and (c[3] or c.z)
    if type(ia) == "number" and type(ic) == "number" then inertiaK = clamp(math.max(ia, ic) / C.REF_I, C.GAIN_MIN, C.GAIN_MAX) end
  end
  return ok and m or nil
end

local function flight(tgt, pred, why)
  attI = { x = 0, z = 0 }
  ff = { w = { x = 0, y = 0, z = 0 } }
  rates = { acc = { x = 0, y = 0, z = 0 } }
  local mass = tune()
  local start, _, _, v0 = tick()
  if mode == "SHUTTLE" then pred = predict(start, tgt, mode, climb, v0) end
  local g = newGuide(start, tgt, mode, climb)
  local tr = newTracker()
  fl.trail, fl.pred, fl.result = {}, pred, nil
  local function guide()
    local t0 = os.clock()
    local last, nextTrail, nextLog = t0, t0, t0
    local flog = fs.open(C.FLIGHT_LOG, "w")
    if flog then
      flog.writeLine(string.format("%s TGT %d %d %d CLIMB %d%s", mode, math.floor(tgt.x), math.floor(tgt.y), math.floor(tgt.z), climb, why and (" START " .. why) or ""))
      flog.writeLine(string.format("FROM %.1f %.1f %.1f VEL %.1f %.1f %.1f MASS %s TACC %.1f GAIN %.2f", start.x, start.y, start.z, v0.x, v0.y, v0.z, tostring(mass), C.THRUST_ACC, inertiaK))
      flog.writeLine("t phase thr speed alt dist dh err_deg gx gy spin wx wy wz nx ny nz vx vy vz cmdx cmdy cmdz actual_gx actual_gy loop_ms")
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
      local pos, w, u, vel, om, _, actualX, actualY = tick(cT, cX, cY)
      local now = os.clock()
      local elapsed = now - last
      local dt = clamp(elapsed, 0.01, 0.5)
      last = now
      local nose = rotate(w, u, UP)
      local dir, done = guideStep(g, pos, vel, nose, dt)
      fl.phase, fl.tmode, fl.thr = g.phase, g.tmode, g.throttle
      fl.pos, fl.speed, fl.dist = pos, len(vel), len(sub(tgt, pos))
      if g.phase == "CRUISE" then track(tr, contactPoint(pos, nose), tgt) end
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
      if low and g.phase ~= "IGNITION" and g.phase ~= "SEPARATE" and len(vel) < C.CUT_STILL_V then stillT = stillT + dt else stillT = 0 end
      if not done and (badT > C.CUT_ERR_T or stillT > C.CUT_STILL_T) then
        g.phase, g.throttle, g.tmode = "CUT", 0, badT > C.CUT_ERR_T and "TILT" or "STILL"
        fl.phase, fl.tmode = g.phase, g.tmode
        if flog then flog.writeLine(string.format("CUT %s %.2f", g.tmode, now - t0)) end
        done = true
      end
      if flog and now >= nextLog then
        local err = errNow
        local dhx, dhz = tgt.x - pos.x, tgt.z - pos.z
        flog.writeLine(string.format("%.2f %s %.2f %.1f %.1f %.1f %.1f %.1f %+.2f %+.2f %.2f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.2f %+.2f %+.2f %+.3f %+.3f %+.3f %s %s %.0f",
          now - t0, g.phase, g.throttle, len(vel), pos.y, len(sub(tgt, pos)), math.sqrt(dhx * dhx + dhz * dhz), err, fl.gx, fl.gy, len(om), om.x, om.y, om.z, nose.x, nose.y, nose.z, vel.x, vel.y, vel.z,
          dir and dir.x or 0, dir and dir.y or 0, dir and dir.z or 0,
          type(actualX) == "number" and string.format("%+.3f", actualX) or "NA",
          type(actualY) == "number" and string.format("%+.3f", actualY) or "NA", elapsed * 1000))
        flog.flush()
        nextLog = now + 0.25
      end
      if done then
        if flog then flog.close() flog = nil end
        cutEngine()
        fl.thr, fl.gx, fl.gy = 0, 0, 0
        track(tr, contactPoint(pos, nose), tgt)
        local cutSpeed, cutTime = len(vel), now - t0
        local stop, still = os.clock() + 4, 0
        while os.clock() < stop and not tr.hit do
          local ok, p2, w2, u2, v2 = pcall(tick)
          if not ok then break end
          track(tr, contactPoint(p2, rotate(w2, u2, UP)), tgt)
          fl.pos, fl.speed, fl.dist = p2, len(v2), len(sub(tgt, p2))
          fl.trail[#fl.trail + 1] = p2
          still = len(v2) < 0.5 and still + 1 or 0
          if still > 10 then break end
        end
        fl.result = missOf(tr, start, tgt)
        if fl.result then pcall(writeLog, tgt, fl.result, cutSpeed, cutTime) end
        if REMOTE and fl.result then pcall(REMOTE.result, fl.result, cutSpeed, cutTime) end
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
      local ok, aborted = pcall(REMOTE.report, {
        phase = fl.phase, alt = p.y, speed = sp, dist = fl.dist or 0,
        eta = (sp > 0.5 and fl.dist) and fl.dist / sp or 0, thrust = (fl.thr or 0) * 100,
        gx = fl.gx or 0, gy = fl.gy or 0, engine_mb = tel.eng or 0, tank_mb = tel.tank or 0,
        x = p.x, y = p.y, z = p.z
      })
      if ok and aborted then fl.abort = true end
    end
  end
  fl.abort = false
  if REMOTE then
    parallel.waitForAny(guide, display, uplink)
  else
    parallel.waitForAny(guide, display)
  end
  cutEngine()
  drawFlight(start, tgt)
end

local function main()
  cutEngine()
  if REMOTE then
    local m = REMOTE.mission
    mode, climb = m.mode == "SHUTTLE" and "SHUTTLE" or "WATER", m.climb or C.CLIMB
    local rt = { x = m.x, y = m.y, z = m.z }
    local w0 = os.clock()
    while not tel.pos and os.clock() - w0 < 5 do sleep(0.1) end
    local pred = tel.pos and currentPlan(tel.pos, rt, mode, climb, true) or nil
    if pred and pred.eta then pcall(REMOTE.plan, pred.eta) end
    if remoteWait(rt) then
      local why
      if mode == "SHUTTLE" then
        why = waitRelease(rt)
        if not why then return end
      end
      REMOTE.state.flown = true
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
term.setCursorBlink(false)
term.setBackgroundColor(colors.black)
term.setTextColor(colors.white)
term.setCursorPos(1, H)
if not ok then print(err) end
