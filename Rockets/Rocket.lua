local REMOTE = ...
if type(REMOTE) ~= "table" then REMOTE = nil end
local SIDE_ENGINE = "bottom"
local SIDE_TANK = "top"
local CLIMB = 50
local DELAY = 5
local VMAX = 1.0
local KP = 1.0
local KD = 0.5
local ATT_KI = 0
local ATT_IMAX = 1.0
local ATT_REF_THR = 0.5
local MAX_THRUST = 0.40
local CRUISE_THR = 0.70
local START_THRUST = 0.15
local THR_RAMP = 0.05
local THR_MARGIN = 0.03
local LIFTOFF_VY = 0.5
local TURN_RATE = 8
local KV = 1.0
local SIGN_X = 1
local SIGN_Y = 1
local SWAP = false
local HIT_RADIUS = 3
local CUT_TIME = 0.1
local NOSE_OFFSET = 3
local TAIL_OFFSET = 2
local GRAV = 10.5
local DRAG = 0.13
local THRUST_ACC = 80
local NET_HOVER = 30
local NET_VMAX = 40
local NET_ABRAKE = 5
local NET_KVEL = 1.0
local NET_KI = 0.5
local NET_TILT = 60
local NET_SINK_TILT = 20
local NET_POS_GAIN = 0.6
local NET_SINK_RADIUS = 4
local NET_SINK_SPEED = 4
local NET_SINK = 3
local NET_VDESC = 12
local NET_CUT = 4
local NET_SLOW = 10
local NET_MIN_THR = 0.08
local NET_MAX_THR = 0.5
local NET_HOLD_ALT = false
local NET_ATT_BOOST = false
local NET_ATT_REF = 0.25
local NET_ATT_MAXK = 3
local SINK_FIX = true
local SINK_KI = 0.4
local SINK_KD = 0.9
local SINK_IWIN = 35
local SINK_REF = 0.2
local SINK_MAXK = 2.5
local FAST_VMAX = 100
local FAST_ABRAKE = 7
local FAST_MARGIN = 120
local FAST_MAX_THR = 1.0
local SPLASH_FAST_THR = 1.0
local SPLASH_SLOW_DIST = 300
local SPLASH_RAMP = 0.3
local ARC_DROP_RADIUS = 6
local ARC_FLIP_THR = 0.06
local ARC_DIVE_THR = 0.15
local ARC_NAV = 3
local ARC_RETRO_MIN = 2
local ARC_RETRO_TIME = 1.0
local ARC_RETRO_LIFT = 0.7
local ARC_HEIGHT = 30
local SIM_TIME = 240
local SAVE = "rocket.target"
local LOG = "rocket.log"
local FLIGHT_LOG = "flight.log"
local MODES = { "SPLASH", "ARC", "NET" }
local climb = CLIMB
local mode = "SPLASH"

local engine = peripheral.wrap(SIDE_ENGINE)
if not engine then error("no vector engine on " .. SIDE_ENGINE, 0) end

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

local function limitTilt(d, maxDeg)
  local h = math.sqrt(d.x * d.x + d.z * d.z)
  if h < 1e-6 then return { x = 0, y = 1, z = 0 } end
  local m = math.rad(maxDeg)
  if atan2(h, d.y) <= m then return d end
  local s = math.sin(m) / h
  return { x = d.x * s, y = math.cos(m), z = d.z * s }
end

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

local function state()
  local p = sublevel.getLogicalPose()
  local w, u = quat(p.orientation)
  return v3(p.position), w, u, v3(sublevel.getLinearVelocity()), v3(sublevel.getAngularVelocity())
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
  return unit(add(dir, mul(sub(dir, mul(vel, 1 / sp)), KV)))
end

local attI = { x = 0, z = 0 }

local sinkI = { x = 0, z = 0 }
local function steer(dirW, w, u, om, dt, thr, boost, sinkFix)
  local d = unit(toBody(w, u, dirW))
  local axis = { x = d.z, y = 0, z = -d.x }
  local s = len(axis)
  local e = { x = 0, y = 0, z = 0 }
  if s > 1e-9 then e = mul(axis, atan2(s, d.y) / s) end
  if ATT_KI > 0 and dt and len(e) < math.rad(10) then
    attI.x = clamp(attI.x + e.x * dt, -ATT_IMAX, ATT_IMAX)
    attI.z = clamp(attI.z + e.z * dt, -ATT_IMAX, ATT_IMAX)
  end
  local k = (mode ~= "NET" and thr and thr > ATT_REF_THR) and ATT_REF_THR / thr or 1
  if boost and thr and thr > 0 then k = clamp(NET_ATT_REF / thr, 1, NET_ATT_MAXK) end
  local ix, iz, kd = 0, 0, KD
  if sinkFix then
    kd = SINK_KD
    if dt and len(e) < math.rad(SINK_IWIN) then
      sinkI.x = clamp(sinkI.x + e.x * dt, -1, 1)
      sinkI.z = clamp(sinkI.z + e.z * dt, -1, 1)
    end
    ix, iz = SINK_KI * sinkI.x, SINK_KI * sinkI.z
    if thr and thr > 0 then k = clamp(SINK_REF / thr, 1, SINK_MAXK) end
  end
  local gx = clamp(k * (KP * e.z + ATT_KI * attI.z + iz - kd * om.z), -VMAX, VMAX) * SIGN_X
  local gy = clamp(-k * (KP * e.x + ATT_KI * attI.x + ix - kd * om.x), -VMAX, VMAX) * SIGN_Y
  if SWAP then gx, gy = gy, gx end
  engine.setVector(gx, gy)
  return gx, gy
end

local function newGuide(start, tgt, md, cl)
  local climbY = start.y + cl
  return {
    start = start, tgt = tgt, mode = md, climb = cl, climbY = climbY,
    P = { x = tgt.x, y = math.max(climbY, tgt.y + (md == "ARC" and ARC_HEIGHT or NET_HOVER)), z = tgt.z },
    phase = "IGNITION", tmode = "RAMP", throttle = START_THRUST,
    cmd = { x = 0, y = 1, z = 0 }, bias = 0, flipped = false,
  }
end

local function contactPoint(g, pos, nose)
  if g.mode == "NET" then return sub(pos, mul(nose, TAIL_OFFSET)) end
  return add(pos, mul(nose, NOSE_OFFSET))
end

local function travelSpeed(d, slow, margin)
  local handoff = NET_VMAX * NET_VMAX / (2 * NET_ABRAKE) + margin
  local fast = math.sqrt(2 * FAST_ABRAKE * math.max(d - handoff, 0))
  return math.min(FAST_VMAX, math.max(slow, fast)), d > handoff
end

local function velCmd(g, vDes, vel, nose, dt, maxTilt, maxThr, capLift)
  if math.abs(vDes.y - vel.y) < 3 then g.bias = clamp(g.bias + NET_KI * (vDes.y - vel.y) * dt, -5, 5) end
  local acc = add(mul(sub(vDes, vel), NET_KVEL), mul(vel, DRAG))
  acc.y = acc.y + GRAV + g.bias
  local push = dot(nose, acc)
  if capLift and nose.y > 0.3 then push = math.min(push, math.max(acc.y, 0) / nose.y) end
  g.throttle = clamp(push / THRUST_ACC, NET_MIN_THR, maxThr or NET_MAX_THR)
  return limitTilt(unit(acc), maxTilt)
end

local function diveDir(g, pos, vel, nose)
  local r = sub(g.tgt, add(pos, mul(nose, NOSE_OFFSET)))
  local dist = len(r)
  if dist < 1e-6 then return nose end
  local rh = mul(r, 1 / dist)
  local tgo = clamp(dist / math.max(dot(vel, rh), 1), 0.3, 20)
  local acc = mul(vel, -DRAG)
  acc.y = acc.y - GRAV
  local zem = sub(r, add(mul(vel, tgo), mul(acc, 0.5 * tgo * tgo)))
  local zp = sub(zem, mul(rh, dot(zem, rh)))
  local T = math.max(g.throttle, 0.02) * THRUST_ACC
  local aLat = mul(zp, ARC_NAV / (tgo * tgo))
  local al = len(aLat)
  if al > 3 * T then aLat = mul(aLat, 3 * T / al) end
  return unit(add(mul(rh, T), aLat))
end

local function guideStep(g, pos, vel, nose, dt)
  local tgt = g.tgt
  if g.phase == "IGNITION" then
    if vel.y > LIFTOFF_VY then
      g.throttle = math.min(g.throttle + THR_MARGIN, MAX_THRUST)
      g.phase, g.tmode = "BOOST", "HOLD"
    else
      g.throttle = math.min(g.throttle + THR_RAMP * dt, MAX_THRUST)
      return nil
    end
  end
  if g.mode ~= "NET" then
    local r = sub(tgt, add(pos, mul(nose, NOSE_OFFSET)))
    local dn = len(r)
    local vc = dn > 1e-6 and dot(vel, r) / dn or 0
    local closing = g.lastDn and dn < g.lastDn
    g.lastDn = dn
    local terminal = g.phase == "CRUISE" or (g.phase == "DIVE" and g.flipped)
    local passed = g.phase == "DIVE" and ((closing == false and dn < 20) or pos.y < tgt.y - 10)
    if dn < HIT_RADIUS or passed or (terminal and ((vc > 1 and dn / vc < CUT_TIME) or (closing == false and dn < 20))) then
      g.phase, g.throttle, g.tmode = "IMPACT", 0, "OFF"
      return nil, true
    end
  end
  local los = sub(tgt, pos)
  if g.phase == "BOOST" and pos.y >= g.climbY then
    if g.mode == "SPLASH" then
      g.phase = "CRUISE"
    else
      g.phase, g.tmode, g.bias = "TRANSFER", "AUTO", 0
    end
  end
  if g.phase == "BOOST" then
    local want = UP
    if g.mode == "SPLASH" then
      local f = clamp((pos.y - g.start.y) / g.climb, 0, 1)
      f = f * f
      want = unit(add(mul(UP, 1 - f), mul(unit(los), f)))
    end
    g.cmd = turnToward(g.cmd, want, math.rad(TURN_RATE) * dt)
    return aim(g.cmd, vel)
  elseif g.phase == "CRUISE" then
    local want = len(los) > SPLASH_SLOW_DIST and SPLASH_FAST_THR or CRUISE_THR
    if g.throttle < want then
      g.throttle = math.min(g.throttle + SPLASH_RAMP * dt, want)
    else
      g.throttle = want
    end
    return diveDir(g, pos, vel, nose)
  elseif g.phase == "TRANSFER" then
    local to = sub(g.P, pos)
    local dP = len(to)
    local hx, hz = tgt.x - pos.x, tgt.z - pos.z
    local dh = math.sqrt(hx * hx + hz * hz)
    if g.mode == "ARC" and dh < ARC_DROP_RADIUS then
      g.phase, g.flipped, g.retroDone, g.flipSide = "DIVE", false, false, nil
    elseif g.mode == "NET" and dh < NET_SINK_RADIUS and len(vel) < NET_SINK_SPEED then
      g.phase = "SINK"
    elseif g.mode == "ARC" then
      local sp, far = travelSpeed(dh, math.min(NET_VMAX, math.sqrt(2 * NET_ABRAKE * dh)), FAST_MARGIN)
      return velCmd(g, { x = hx / dh * sp, y = clamp((g.P.y - pos.y) * 0.5, -10, 10), z = hz / dh * sp }, vel, nose, dt, NET_TILT, far and FAST_MAX_THR or NET_MAX_THR, far)
    else
      if NET_HOLD_ALT then
        local sp = math.min(NET_VMAX, math.sqrt(2 * NET_ABRAKE * dh), NET_POS_GAIN * dh)
        local vx, vz = 0, 0
        if dh > 1e-6 then vx, vz = hx / dh * sp, hz / dh * sp end
        return velCmd(g, { x = vx, y = clamp((g.P.y - pos.y) * 0.5, -10, 10), z = vz }, vel, nose, dt, NET_TILT)
      end
      return velCmd(g, mul(unit(to), math.min(NET_VMAX, math.sqrt(2 * NET_ABRAKE * dP), NET_POS_GAIN * dP)), vel, nose, dt, NET_TILT)
    end
  end
  if g.phase == "DIVE" then
    if not g.flipped and nose.y < -0.3 then g.flipped = true end
    if g.flipped then
      g.throttle, g.tmode = ARC_DIVE_THR, "DIVE"
      return diveDir(g, pos, vel, nose)
    end
    local vh = { x = vel.x, y = 0, z = vel.z }
    local sh = len(vh)
    if sh <= ARC_RETRO_MIN or nose.y <= 0.2 then g.retroDone = true end
    if not g.retroDone then
      local back = mul(vh, -1 / sh)
      local hf = math.max(dot(nose, back), 0.2)
      local thr = math.max(ARC_FLIP_THR, sh / (ARC_RETRO_TIME * THRUST_ACC * hf))
      g.throttle = math.min(thr, NET_MAX_THR, ARC_RETRO_LIFT * GRAV / (THRUST_ACC * nose.y))
      g.tmode = "RETRO"
      return back
    end
    g.throttle, g.tmode = ARC_FLIP_THR, "FLIP"
    local dirT = unit(sub(tgt, add(pos, mul(nose, NOSE_OFFSET))))
    if dot(nose, dirT) < -0.8 then
      if not g.flipSide then
        local nh = { x = nose.x, y = 0, z = nose.z }
        if sh > 0.5 then
          g.flipSide = mul(vh, -1 / sh)
        elseif len(nh) > 1e-3 then
          g.flipSide = unit(nh)
        else
          g.flipSide = { x = 1, y = 0, z = 0 }
        end
      end
      return g.flipSide
    end
    local ymax = nose.y - 0.3
    if dirT.y > ymax then
      local hd = { x = dirT.x, y = 0, z = dirT.z }
      local hl = len(hd)
      if hl < 1e-3 then hd, hl = g.flipSide or { x = 1, y = 0, z = 0 }, 1 end
      local y = math.max(ymax, -1)
      local k = math.sqrt(math.max(1 - y * y, 0)) / hl
      return { x = hd.x * k, y = y, z = hd.z * k }
    end
    return dirT
  elseif g.phase == "SINK" then
    local h = pos.y - (tgt.y + NET_CUT)
    if h <= 0 then
      g.phase, g.throttle, g.tmode = "RELEASE", 0, "OFF"
      return nil, true
    end
    local vy = -math.max(NET_SINK, math.min(NET_VDESC, math.sqrt(2 * NET_ABRAKE * math.max(h - NET_SLOW, 0))))
    return velCmd(g, { x = clamp((tgt.x - pos.x) * 0.5, -2, 2), y = vy, z = clamp((tgt.z - pos.z) * 0.5, -2, 2) }, vel, nose, dt, NET_SINK_TILT)
  end
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

local function predict(start, tgt, md, cl)
  local g = newGuide(start, tgt, md, cl)
  local pos = { x = start.x, y = start.y, z = start.z }
  local vel = { x = 0, y = 0, z = 0 }
  local nose = { x = 0, y = 1, z = 0 }
  local thr, t, nextRec = 0, 0, 0
  local pts = { pos }
  local tr = newTracker()
  local done, tDone = false, nil
  while t < SIM_TIME do
    local near = len(sub(tgt, pos)) < 150
    local dt = (g.phase == "IGNITION" or g.phase == "BOOST" or near) and 0.1 or 0.25
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
    local acc = add(mul(nose, thr * THRUST_ACC), mul(vel, -DRAG))
    acc.y = acc.y - GRAV
    vel = add(vel, mul(acc, dt))
    pos = add(pos, mul(vel, dt))
    if g.phase == "IGNITION" and pos.y < start.y then
      pos.y = start.y
      vel = { x = 0, y = 0, z = 0 }
    end
    t = t + dt
    if g.phase ~= "IGNITION" and g.phase ~= "BOOST" then track(tr, contactPoint(g, pos, nose), tgt) end
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
    tel.eng = fluidAmount(SIDE_ENGINE)
    tel.tank = fluidAmount(SIDE_TANK)
    os.queueEvent("telemetry")
    sleep(0.5)
  end
end

local function fuel()
  while true do
    pcall(engine.pullFluid, SIDE_TANK)
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

local fields = { { label = "X", value = "" }, { label = "Y", value = "" }, { label = "Z", value = "" }, { label = "CLIMB", value = tostring(CLIMB) }, { label = "MODE", value = "SPLASH", choice = true } }
local active = 1
local msg = ""
local FX, FW = 9, 14

if fs.exists(SAVE) then
  local f = fs.open(SAVE, "r")
  local d = textutils.unserialize(f.readAll())
  f.close()
  if type(d) == "table" and d.x and d.y and d.z then
    fields[1].value, fields[2].value, fields[3].value = tostring(d.x), tostring(d.y), tostring(d.z)
  end
  if type(d) == "table" and d.climb then fields[4].value = tostring(d.climb) end
  if type(d) == "table" and d.mode then
    for _, m in ipairs(MODES) do if m == d.mode then fields[5].value = m end end
  end
end

local function fieldRow(i) return 3 + i end

local function target()
  local x, y, z = tonumber(fields[1].value), tonumber(fields[2].value), tonumber(fields[3].value)
  if x and y and z then return { x = x, y = y, z = z } end
end

local function cycleMode(dir)
  local i = 1
  for k, m in ipairs(MODES) do if m == fields[5].value then i = k end end
  fields[5].value = MODES[(i - 1 + dir) % #MODES + 1]
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
  climb = climbValue() or CLIMB
  local tgt = target()
  local pl = currentPlan(p, tgt, fields[5].value, climb)
  plot(p, tgt, nil, pl, nil)
  if p and tgt and not pl then text(PX - 1, 17, "PLANNING...", colors.gray) end
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
  local f = fs.open(SAVE, "w")
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
        fields[4].value = tostring(CLIMB)
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
  local done = math.floor(bw * clamp((DELAY - left) / DELAY, 0, 1) + 0.5)
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
    local left = DELAY - (os.clock() - t0)
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
    if left then drawCountdown(math.min(left, DELAY), tgt) else drawArmed(tgt) end
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

local fl = { phase = "IGNITION", dx = 0, dy = 0, dz = 0, trail = {} }

local function drawFlight(start, tgt)
  clear()
  local colorsFor = { IGNITION = colors.yellow, BOOST = colors.orange, CRUISE = colors.cyan, TRANSFER = colors.cyan, DIVE = colors.magenta, SINK = colors.lightBlue, RELEASE = colors.lime, IMPACT = colors.red }
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
  local f = fs.open(LOG, "a")
  if not f then return end
  f.writeLine(string.format("%s %s TGT %d %d %d MISS %.1f LONG %+.1f RIGHT %+.1f SPEED %.0f TIME %.1f%s",
    ok and d or "?", mode, math.floor(tgt.x), math.floor(tgt.y), math.floor(tgt.z),
    r.total, r.long, r.right, speed or 0, t or 0, r.crossed and "" or " CLOSEST"))
  f.close()
end

local function flight(tgt, pred)
  local start = state()
  local g = newGuide(start, tgt, mode, climb)
  local tr = newTracker()
  fl.trail, fl.pred, fl.result = {}, pred, nil
  local function setDir(dirW, w, u, om, dt)
    local d = unit(toBody(w, u, dirW))
    fl.dx, fl.dy, fl.dz = d.x, d.y, d.z
    local sinking = SINK_FIX and mode == "NET" and g.phase == "SINK"
    if sinking and not g.sinkStarted then
      g.sinkStarted = true
      sinkI.x, sinkI.z = 0, 0
    end
    fl.gx, fl.gy = steer(dirW, w, u, om, dt, g.throttle, NET_ATT_BOOST and mode == "NET" and (g.phase == "TRANSFER" or g.phase == "SINK"), sinking)
  end
  local function guide()
    local t0 = os.clock()
    local last, nextTrail, nextLog = t0, t0, t0
    local flog = fs.open(FLIGHT_LOG, "w")
    if flog then
      flog.writeLine(string.format("%s TGT %d %d %d CLIMB %d", mode, math.floor(tgt.x), math.floor(tgt.y), math.floor(tgt.z), climb))
      flog.writeLine("t phase thr speed alt dist dh err_deg gx gy spin")
    end
    engine.setVector(0, 0)
    engine.setThrustNormalized(g.throttle)
    while true do
      if fl.abort then
        if flog then flog.close() flog = nil end
        engine.setThrustNormalized(0)
        engine.setVector(0, 0)
        fl.thr, fl.gx, fl.gy, fl.phase = 0, 0, 0, "ABORT"
        return
      end
      local now = os.clock()
      local dt = clamp(now - last, 0.01, 0.5)
      last = now
      local pos, w, u, vel, om = state()
      local nose = rotate(w, u, UP)
      local dir, done = guideStep(g, pos, vel, nose, dt)
      fl.phase, fl.tmode, fl.thr = g.phase, g.tmode, g.throttle
      fl.pos, fl.speed, fl.dist = pos, len(vel), len(sub(tgt, pos))
      if g.phase ~= "IGNITION" and g.phase ~= "BOOST" then track(tr, contactPoint(g, pos, nose), tgt) end
      if now >= nextTrail then
        fl.trail[#fl.trail + 1] = pos
        nextTrail = now + 0.25
      end
      if flog and now >= nextLog then
        local err = dir and math.deg(math.acos(clamp(dot(nose, unit(dir)), -1, 1))) or 0
        local dhx, dhz = tgt.x - pos.x, tgt.z - pos.z
        flog.writeLine(string.format("%.2f %s %.2f %.1f %.1f %.1f %.1f %.1f %+.2f %+.2f %.2f",
          now - t0, g.phase, g.throttle, len(vel), pos.y, len(sub(tgt, pos)), math.sqrt(dhx * dhx + dhz * dhz), err, fl.gx or 0, fl.gy or 0, len(om)))
        flog.flush()
        nextLog = now + 0.25
      end
      if done then
        if flog then flog.close() flog = nil end
        engine.setThrustNormalized(0)
        engine.setVector(0, 0)
        fl.thr, fl.gx, fl.gy = 0, 0, 0
        local cutSpeed, cutTime = len(vel), now - t0
        local stop, still = os.clock() + 4, 0
        while os.clock() < stop and not tr.hit do
          sleep(0.05)
          local ok, p2, w2, u2, v2 = pcall(state)
          if not ok then break end
          track(tr, contactPoint(g, p2, rotate(w2, u2, UP)), tgt)
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
      if dir then
        setDir(dir, w, u, om, dt)
      else
        engine.setVector(0, 0)
        fl.gx, fl.gy = 0, 0
      end
      engine.setThrustNormalized(g.throttle)
      sleep(0.05)
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
  drawFlight(start, tgt)
end

local function main()
  engine.setThrustNormalized(0)
  engine.setVector(0, 0)
  if REMOTE then
    local m = REMOTE.mission
    mode, climb = m.mode, m.climb
    local rt = { x = m.x, y = m.y, z = m.z }
    local w0 = os.clock()
    while not tel.pos and os.clock() - w0 < 5 do sleep(0.1) end
    local pred = tel.pos and currentPlan(tel.pos, rt, mode, climb, true) or nil
    if pred and pred.eta then pcall(REMOTE.plan, pred.eta) end
    if remoteWait(rt) then
      REMOTE.state.flown = true
      flight(rt, pred)
    end
    return
  end
  local tgt
  repeat
    tgt = inputScreen()
  until countdown(tgt)
  flight(tgt, currentPlan(tel.pos, tgt, mode, climb, true))
end

parallel.waitForAny(main, telemetry, fuel)
term.setCursorBlink(false)
term.setBackgroundColor(colors.black)
term.setTextColor(colors.white)
term.setCursorPos(1, H)
