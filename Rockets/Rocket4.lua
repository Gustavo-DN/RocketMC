local REMOTE = ...
if type(REMOTE) ~= "table" then REMOTE = nil end
local C = {
  ENGINE_TYPE = "liquid_vector_thruster",
  ENGINE_ORDER = { 2, 3, 1, 0 },
  TANK_TYPE = "fluid_storage",
  FLARE_SIDE = "top",
  FLARE_PULSE = 1.0,
  CLIMB = 50,
  DELAY = 5,
  VMAX = 1.0,
  KP = 1.5,
  KD = 1.6,
  ATT_KI = 0.8,
  ATT_IMAX = 1.0,
  ATT_IWIN = 35,
  ATT_REF_THR = 0.4,
  CRUISE_KD_MULT = 1.3,
  ROLL = true,
  ROLL_KP = 0.3,
  ROLL_KD = 0.4,
  ROLL_MAX = 0.3,
  ROLL_HOLD = true,
  ROLL_HOLD_MINY = 0.8,
  REF_I_PITCH = 1144.4,
  REF_I_ROLL = 61.5,
  GAIN_MAX = 2,
  THRUST_FORCE = 2025,
  AERO_FF = true,
  AERO_K = 0.03,
  AERO_A0 = 1.65,
  AERO_THR0 = 0.335,
  AERO_VMAX = 25,
  AERO_MAX = 0.6,
  THRUST_ACC = 31,
  HOVER_START = 0.75,
  IGN_MAX = 0.85,
  MAX_THRUST = 0.85,
  CRUISE_THR = 1.0,
  START_THRUST = 0.25,
  THR_RAMP = 0.05,
  THR_MARGIN = 0.12,
  LIFTOFF_VY = 0.5,
  TURN_RATE = 8,
  KV = 1.0,
  ALIGN_MIN_V = 8,
  SIGN_X = 1,
  SIGN_Y = 1,
  SWAP = false,
  ARRIVE_RADIUS = 3,
  CUT_TIME = 0.1,
  NOSE_OFFSET = 12.7,
  TAIL_OFFSET = 4.8,
  GRAV = 10.5,
  DRAG = 0.13,
  CUT_TILT = 60,
  CUT_ERR = 60,
  CUT_CLOSE = 5,
  CUT_SPIN = 2.5,
  CUT_HOLD = 0.3,
  CUT_HOLD_ERR = 1.0,
  CUT_GRACE = 4,
  CUT_LOW = 20,
  REC_TILT = 45,
  REC_EXIT = 15,
  REC_SPIN = 0.3,
  REC_HOLD = 1.0,
  REC_CMD_TILT = 15,
  REC_MIN_HOVER = 0.9,
  REC_CLIMB_V = 3,
  REC_MAX_T = 8,
  REC_TIMEOUT_TILT = 30,
  NET_BIAS = 8,
  NET_HOVER = 30,
  NET_VMAX = 30,
  NET_ABRAKE = 4,
  NET_KVEL = 1.0,
  KVEL_MIN = 0.5,
  KVEL_V = 15,
  NET_KI = 0.25,
  NET_TILT = 30,
  NET_SINK_TILT = 6,
  NET_POS_GAIN = 0.2,
  SINK_POS_GAIN = 0.25,
  SINK_POS_V = 1.5,
  TOUCH_VY = 0.3,
  TOUCH_TIME = 1.0,
  NET_SINK_RADIUS = 4,
  NET_SINK_SPEED = 4,
  NET_SINK = 2,
  NET_VDESC = 12,
  NET_CLEAR = 0.5,
  NET_SLOW = 15,
  NET_MIN_ACC = 8,
  CMD_RATE = 20,
  NET_MIN_THR = 0.21,
  NET_MAX_THR = 0.93,
  NET_HOLD_ALT = false,
  NET_ATT_BOOST = false,
  NET_ATT_REF = 0.64,
  NET_ATT_MAXK = 3,
  SINK_FIX = true,
  SINK_KI = 0.8,
  SINK_KD = 2.4,
  SINK_IWIN = 35,
  SINK_REF = 0.4,
  SINK_MAXK = 2.5,
  FAST_VMAX = 100,
  FAST_ABRAKE = 7,
  EXPRESS = true,
  AB_MIN_DIST = 1000,
  AB_VMAX = 500,
  AB_MARGIN = 120,
  AB_MAX_THR = 1.0,
  AB_PITCH_UP = 55,
  AB_PITCH_DOWN = 10,
  AB_VDECEL = 5,
  AB_LIFT = 0.4,
  AB_ATT_K = 1.0,
  AB_KALT = 0.05,
  AB_KVY = 0.4,
  AB_AMAX = 4,
  AB_KA = 1.5,
  AB_ACC_TAU = 0.5,
  AB_AOA_MIN = 10,
  AB_AOA_START = 5,
  AB_AOA_MAX = 45,
  AB_KLAT = 1.0,
  AB_LAT_VREF = 20,
  AB_PITCH_RATE = 15,
  AB_PAD = 60,
  AB_BRAKE = 7,
  AB_REARM_D = 400,
  AB_REARM_K = 0.5,
  WATER_FAST_THR = 1.0,
  WATER_SLOW_DIST = 300,
  WATER_RAMP = 0.3,
  APPROACH_NAV = 3,
  ARC_AB_MIN = 0,
  ARC_APPROACH_H = 60,
  DIVE_V = 15,
  DIVE_H = 250,
  DIVE_R = 0.4,
  DIVE_AH = 4,
  DIVE_VH = 20,
  DIVE_KX = 0.5,
  DIVE_TEND = 2,
  DIVE_VHMIN = 5,
  DIVE_VYMIN = 20,
  DIVE_KVY = 0.5,
  DIVE_XV = 15,
  DIVE_PN_H = 150,
  DIVE_AV = 12,
  DIVE_TGO_MIN = 0.5,
  DIVE_NAV = 3,
  DIVE_KV = 0.8,
  DIVE_VMIN = 15,
  DIVE_MIN_THR = 0.21,
  DIVE_MAX_THR = 0.9,
  DIVE_AOA = 25,
  DIVE_WFF_TAU = 0.3,
  DIVE_WFF_MAX = 1.0,
  DIVE_WFF_STEP = 5,
  DIVE_STOP_V = 2,
  DIVE_STOP_T = 0.5,
  AUTH_ERR = 20,
  AUTH_V = 15,
  AUTH_THR = 0.45,
  CEILING = 700,
  CEIL_THR = 0.1,
  SIM_TIME = 240,
  SIM_TURN = 3,
  LOG_DT = 0.25,
  SAVE = "rocket.dest",
  LOG = "rocket.log",
  FLIGHT_LOG = "flight.log",
  MODES = { "WATER", "ARC", "NET" },
}
local climb = C.CLIMB
local mode = "WATER"

local E = { names = {}, eng = {}, tanks = {}, cmd = {}, sx = {}, sy = {}, kp = 1, kr = 1, ms = 0 }
for _, n in ipairs(peripheral.getNames()) do
  if peripheral.hasType(n, C.ENGINE_TYPE) then E.names[#E.names + 1] = n end
end
table.sort(E.names)
do
  local ord, full = {}, #C.ENGINE_ORDER == #E.names
  for _, n in ipairs(E.names) do
    local k = tonumber(n:match("_(%d+)$"))
    for i, want in ipairs(C.ENGINE_ORDER) do if k == want then ord[i] = n end end
  end
  for i = 1, #C.ENGINE_ORDER do if not ord[i] then full = false end end
  if full then E.names = ord end
end
if #E.names == 0 then error("no " .. C.ENGINE_TYPE .. " found", 0) end
E.N = #E.names
for i, n in ipairs(E.names) do
  E.eng[i] = peripheral.wrap(n)
  E.cmd[i] = { 0, 0, 0 }
end
if E.N == 4 then E.sx, E.sy = { 1, 1, -1, -1 }, { 1, -1, 1, -1 } end
for _, n in ipairs(peripheral.getNames()) do
  if peripheral.hasType(n, C.TANK_TYPE) and not peripheral.hasType(n, C.ENGINE_TYPE) then E.tanks[#E.tanks + 1] = n end
end

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

function E.io(send)
  local fns, p, v, o = {}
  if send then
    for i = 1, E.N do
      local e, c = E.eng[i], E.cmd[i]
      local thr, gx, gy = c[1], c[2], c[3]
      fns[#fns + 1] = function() e.setThrustNormalized(thr) end
      fns[#fns + 1] = function() e.setVector(gx, gy) end
    end
  end
  fns[#fns + 1] = function() p = sublevel.getLogicalPose() end
  fns[#fns + 1] = function() v = sublevel.getLinearVelocity() end
  fns[#fns + 1] = function() o = sublevel.getAngularVelocity() end
  local a = os.epoch("utc")
  parallel.waitForAll(table.unpack(fns))
  E.ms = os.epoch("utc") - a
  local w, u = quat(p.orientation)
  return v3(p.position), w, u, v3(v), v3(o)
end

function E.set(thr, gx, gy, r)
  thr = clamp(thr, 0, 1)
  for i = 1, E.N do
    local c = E.cmd[i]
    c[1] = thr
    c[2] = clamp(gx + (E.sx[i] or 0) * r, -1, 1)
    c[3] = clamp(gy + (E.sy[i] or 0) * r, -1, 1)
  end
end

function E.cut()
  E.set(0, 0, 0, 0)
  if not pcall(E.io, true) then
    for i = 1, E.N do
      pcall(E.eng[i].setThrustNormalized, 0)
      pcall(E.eng[i].setVector, 0, 0)
    end
  end
end

local function heading(w, u)
  local b = rotate(w, u, { x = 1, y = 0, z = 0 })
  return atan2(-b.z, b.x)
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

local function align(dir, vel)
  local sp = len(vel)
  if sp < C.ALIGN_MIN_V then return dir end
  return unit(add(dir, mul(sub(dir, mul(vel, 1 / sp)), C.KV)))
end

local attI = { x = 0, z = 0 }

local sinkI = { x = 0, z = 0 }
local function steer(dirW, w, u, om, dt, thr, boost, sinkFix, ab, vel, wff)
  local d = unit(toBody(w, u, dirW))
  local axis = { x = d.z, y = 0, z = -d.x }
  local s = len(axis)
  local e = { x = 0, y = 0, z = 0 }
  if s > 1e-9 then e = mul(axis, atan2(s, d.y) / s) end
  if C.ATT_KI > 0 and dt and not sinkFix and len(e) < math.rad(C.ATT_IWIN) then
    attI.x = clamp(attI.x + e.x * dt, -C.ATT_IMAX, C.ATT_IMAX)
    attI.z = clamp(attI.z + e.z * dt, -C.ATT_IMAX, C.ATT_IMAX)
  end
  local k = ((mode == "WATER" or ab) and thr and thr > C.ATT_REF_THR) and C.ATT_REF_THR / thr or 1
  if boost and thr and thr > 0 then k = clamp(C.NET_ATT_REF / thr, 1, C.NET_ATT_MAXK) end
  local ix, iz, kd = 0, 0, (k < 1) and C.KD * C.CRUISE_KD_MULT or C.KD
  local ki = sinkFix and 0 or C.ATT_KI
  if sinkFix then
    kd = C.SINK_KD
    if dt and len(e) < math.rad(C.SINK_IWIN) then
      sinkI.x = clamp(sinkI.x + e.x * dt, -1, 1)
      sinkI.z = clamp(sinkI.z + e.z * dt, -1, 1)
    end
    ix, iz = C.SINK_KI * sinkI.x, C.SINK_KI * sinkI.z
    if thr and thr > 0 then k = clamp(C.SINK_REF / thr, 1, C.SINK_MAXK) end
  end
  k = k * E.kp
  if ab then k = k * C.AB_ATT_K end
  local fx, fy = 0, 0
  if C.AERO_FF and vel and thr and thr > 0.05 then
    local du = unit(dirW)
    local bv = toBody(w, u, wff and sub(vel, mul(du, dot(vel, du))) or vel)
    local sp = len(vel)
    local side = (wff or sp <= 1) and 1 or 1 - math.max(bv.y, 0) / sp
    local a = C.AERO_A0 * thr / C.AERO_THR0 / side
    fx = clamp(C.AERO_K * clamp(bv.x, -C.AERO_VMAX, C.AERO_VMAX) / a, -C.AERO_MAX, C.AERO_MAX)
    fy = clamp(C.AERO_K * clamp(bv.z, -C.AERO_VMAX, C.AERO_VMAX) / a, -C.AERO_MAX, C.AERO_MAX)
  end
  local wx, wz = om.x, om.z
  if wff then wx, wz = wx - wff.x, wz - wff.z end
  local gx = k * (C.KP * e.z + ki * attI.z + iz - kd * wz) + fx
  local gy = -k * (C.KP * e.x + ki * attI.x + ix - kd * wx) + fy
  local gm = math.max(math.abs(gx), math.abs(gy))
  if wff and gm > C.VMAX then gx, gy = gx * C.VMAX / gm, gy * C.VMAX / gm end
  gx = clamp(gx, -C.VMAX, C.VMAX) * C.SIGN_X
  gy = clamp(gy, -C.VMAX, C.VMAX) * C.SIGN_Y
  if C.SWAP then gx, gy = gy, gx end
  return gx, gy
end

local function rollCmd(w, u, om, nose, thr, gx, gy)
  if not C.ROLL or E.N ~= 4 then return 0 end
  local r = -C.ROLL_KD * om.y
  if C.ROLL_HOLD and E.psi0 and nose.y > C.ROLL_HOLD_MINY then
    local e = (E.psi0 - heading(w, u) + math.pi) % (2 * math.pi) - math.pi
    r = r + C.ROLL_KP * e
  end
  local k = (thr > C.ATT_REF_THR and C.ATT_REF_THR / thr or 1) * E.kr
  local lim = math.min(C.ROLL_MAX, math.max(1 - math.max(math.abs(gx), math.abs(gy)), 0))
  return clamp(k * r, -lim, lim)
end

local function diag(t)
  if type(t) ~= "table" then return nil end
  if type(t[1]) == "number" and #t >= 9 then return { t[1], t[5], t[9] } end
  local r, ks = {}, { "x", "y", "z" }
  for i = 1, 3 do
    local k = ks[i]
    local row = t[i] or t[k]
    if type(row) == "table" then
      r[i] = row[i] or row[k]
    elseif type(row) == "number" then
      r[i] = row
    end
    r[i] = r[i] or t[k .. k] or t["m" .. (i - 1) .. (i - 1)]
    if type(r[i]) ~= "number" then return nil end
  end
  return r
end

local function tune()
  local ok, m = pcall(function() return sublevel.getMass() end)
  if ok and type(m) == "number" and m > 0 then
    E.mass = m
    C.THRUST_ACC = C.THRUST_FORCE / m
  end
  local ok2, it = pcall(function() return sublevel.getInertiaTensor() end)
  local d = ok2 and diag(it)
  if d then
    E.kp = clamp(math.max(d[1], d[3]) / C.REF_I_PITCH, 1, C.GAIN_MAX)
    E.kr = clamp(d[2] / C.REF_I_ROLL, 1, C.GAIN_MAX)
  end
  local ta = C.THRUST_ACC
  E.hover = C.GRAV / ta
  C.START_THRUST = clamp(E.hover * C.HOVER_START, 0.1, 0.8)
  C.MAX_THRUST = clamp(math.max(C.IGN_MAX, E.hover + 0.25), 0, 1)
  C.NET_MIN_THR = clamp(C.NET_MIN_ACC / ta, 0.05, 0.6)
end

local function newGuide(start, tgt, md, cl)
  local climbY = start.y + cl
  return {
    start = start, tgt = tgt, mode = md, climb = cl, climbY = climbY,
    P = { x = tgt.x, y = math.max(climbY, tgt.y + (md == "ARC" and C.DIVE_H or C.NET_HOVER)), z = tgt.z },
    phase = "IGNITION", tmode = "RAMP", throttle = C.START_THRUST,
    cmd = { x = 0, y = 1, z = 0 }, bias = 0,
  }
end

local function contactPoint(g, pos, nose)
  if g.mode == "NET" or (g.mode == "ARC" and nose.y > 0) then return sub(pos, mul(nose, C.TAIL_OFFSET)) end
  return add(pos, mul(nose, C.NOSE_OFFSET))
end

local function travelSpeed(d, slow, margin)
  local handoff = C.NET_VMAX * C.NET_VMAX / (2 * C.NET_ABRAKE) + margin
  local fast = math.sqrt(2 * C.FAST_ABRAKE * math.max(d - handoff, 0))
  return math.min(C.FAST_VMAX, math.max(slow, fast)), d > handoff
end

local function velCmd(g, vDes, vel, nose, dt, maxTilt, maxThr, capLift)
  if math.abs(vDes.y - vel.y) < 3 then g.bias = clamp(g.bias + C.NET_KI * (vDes.y - vel.y) * dt, -C.NET_BIAS, C.NET_BIAS) end
  local kv = C.KVEL_MIN + (C.NET_KVEL - C.KVEL_MIN) * clamp(len(vDes) / C.KVEL_V, 0, 1)
  local acc = add(mul(sub(vDes, vel), kv), mul(vel, C.DRAG))
  acc.y = (vDes.y - vel.y) * C.NET_KVEL + vel.y * C.DRAG
  acc.y = acc.y + C.GRAV + g.bias
  local push = dot(nose, acc)
  if capLift and nose.y > 0.3 then push = math.min(push, math.max(acc.y, 0) / nose.y) end
  g.throttle = clamp(push / C.THRUST_ACC, C.NET_MIN_THR, maxThr or C.NET_MAX_THR)
  return limitTilt(unit(acc), maxTilt)
end

local function approachDir(g, pos, vel, nose, via, nav)
  local r = via and sub(via, pos) or sub(g.tgt, add(pos, mul(nose, C.NOSE_OFFSET)))
  local dist = len(r)
  if dist < 1e-6 then return nose end
  local rh = mul(r, 1 / dist)
  local tArr = clamp(dist / math.max(dot(vel, rh), 1), 0.3, 20)
  local acc = mul(vel, -C.DRAG)
  acc.y = acc.y - C.GRAV
  local drift = sub(r, add(mul(vel, tArr), mul(acc, 0.5 * tArr * tArr)))
  local zp = sub(drift, mul(rh, dot(drift, rh)))
  local T = math.max(g.throttle, 0.02) * C.THRUST_ACC
  local aLat = mul(zp, (nav or C.APPROACH_NAV) / (tArr * tArr))
  local al = len(aLat)
  if al > 3 * T then aLat = mul(aLat, 3 * T / al) end
  return unit(add(mul(rh, T), aLat))
end

local function arcDir(g, pos, vel)
  local tgt = g.tgt
  local hx, hz = tgt.x - pos.x, tgt.z - pos.z
  local x = math.sqrt(hx * hx + hz * hz)
  local h = math.max(pos.y - tgt.y - C.NOSE_OFFSET, 0)
  local sp = math.min(C.DIVE_VH, math.sqrt(2 * C.DIVE_AH * x), C.DIVE_KX * x)
  local ux, uz = 0, 0
  if x > 1e-6 then ux, uz = hx / x, hz / x end
  local vyd = -math.max(h / (2 * x / math.max(sp, C.DIVE_VHMIN) + C.DIVE_TEND), C.DIVE_VYMIN)
  local fy = vel.y < vyd and C.DIVE_KVY * (vyd - vel.y) * clamp(x / C.DIVE_XV, 0, 1) or 0
  local f = { x = C.DIVE_KV * (ux * sp - vel.x) + C.DRAG * vel.x, y = fy, z = C.DIVE_KV * (uz * sp - vel.z) + C.DRAG * vel.z }
  if h < C.DIVE_PN_H and -vel.y > C.DIVE_VMIN then
    local vdn = -vel.y
    local tArr = math.max((math.sqrt(vdn * vdn + 2 * C.DIVE_AV * h) - vdn) / C.DIVE_AV, C.DIVE_TGO_MIN)
    local e = math.exp(-C.DRAG * tArr)
    local k = (1 - e) / C.DRAG
    local n = C.DIVE_NAV / (tArr * tArr)
    f = { x = n * (hx - vel.x * k), y = 0, z = n * (hz - vel.z * k) }
  end
  local ta = C.THRUST_ACC
  local v = len(vel)
  if v < C.DIVE_VMIN then
    g.throttle = C.DIVE_MIN_THR
    return unit({ x = f.x, y = -C.GRAV, z = f.z })
  end
  local vu = mul(vel, 1 / v)
  local fp = dot(f, vu)
  local fn = sub(f, mul(vu, fp))
  local fnl = len(fn)
  local T = clamp(fp, C.DIVE_MIN_THR * ta, C.DIVE_MAX_THR * ta)
  g.throttle = T / ta
  if fnl < 1e-6 then return vu end
  local sn = clamp(fnl / (T + C.AB_LIFT * v), 0, math.sin(math.rad(C.DIVE_AOA)))
  return add(mul(vu, math.sqrt(1 - sn * sn)), mul(fn, sn / fnl))
end

local function levelDir(g, pos, vel, fx, fz, dt)
  dt = dt or 0.05
  local T = math.max(g.throttle, 0.05) * C.THRUST_ACC
  local sp = math.sqrt(vel.x * vel.x + vel.z * vel.z)
  local gam = math.deg(math.atan2(vel.y, math.max(sp, 1)))
  if g.abVy then
    local am = (vel.y - g.abVy) / dt
    g.abAm = (g.abAm or am) + (am - (g.abAm or am)) * math.min(1, dt / C.AB_ACC_TAU)
  end
  g.abVy = vel.y
  local pitch
  if not g.abLock then
    g.abY = pos.y
    pitch = math.deg(math.asin(clamp((C.GRAV - C.AB_VDECEL) / T, -1, 1)))
    if vel.y <= 0.5 then g.abLock, g.abA = true, math.max(pitch - gam, math.deg(math.asin(clamp(C.GRAV / T, 0, 1))) + C.AB_AOA_START) end
  else
    local e = g.abY - pos.y
    local ad = clamp(C.AB_KALT * e - C.AB_KVY * vel.y, -C.AB_AMAX, C.AB_AMAX)
    g.abA = clamp(g.abA + C.AB_KA * (ad - (g.abAm or 0)) * dt, -C.AB_AOA_MIN, C.AB_AOA_MAX)
    pitch = gam + g.abA
  end
  pitch = clamp(pitch, -C.AB_PITCH_DOWN, C.AB_PITCH_UP)
  local cur = g.abPitch or math.deg(math.asin(clamp(g.noseY or 1, -1, 1)))
  g.abPitch = cur + clamp(pitch - cur, -C.AB_PITCH_RATE * dt, C.AB_PITCH_RATE * dt)
  local vs = fx * vel.x + fz * vel.z
  local px, pz = vel.x - fx * vs, vel.z - fz * vs
  local vref = math.max(sp, C.AB_LAT_VREF)
  local hx, hz = fx - C.AB_KLAT * px / vref, fz - C.AB_KLAT * pz / vref
  local hl = math.sqrt(hx * hx + hz * hz)
  hx, hz = hx / hl, hz / hl
  local th = math.rad(g.abPitch)
  return { x = hx * math.cos(th), y = math.sin(th), z = hz * math.cos(th) }
end

local function guideStep(g, pos, vel, nose, dt)
  local tgt = g.tgt
  g.noseY = nose.y
  if g.phase == "IGNITION" then
    if vel.y > C.LIFTOFF_VY then
      g.throttle = math.min(g.throttle + C.THR_MARGIN, C.MAX_THRUST)
      g.phase, g.tmode = "ASCENT", "HOLD"
    else
      g.throttle = math.min(g.throttle + C.THR_RAMP * dt, C.MAX_THRUST)
      return nil
    end
  end
  if g.mode == "WATER" then
    local r = sub(tgt, add(pos, mul(nose, C.NOSE_OFFSET)))
    local dn = len(r)
    local vc = dn > 1e-6 and dot(vel, r) / dn or 0
    local closing = g.lastDn and dn < g.lastDn
    g.lastDn = dn
    local final = g.phase == "CRUISE"
    local passed = false
    if final and len(vel) < 2 then g.stuck = (g.stuck or 0) + dt else g.stuck = 0 end
    if g.stuck > 1 then passed = true end
    if dn < C.ARRIVE_RADIUS or passed or (final and ((vc > 1 and dn / vc < C.CUT_TIME) or (closing == false and dn < 20))) then
      g.phase, g.throttle, g.tmode = "TOUCHDOWN", 0, "OFF"
      return nil, true
    end
  end
  local los = sub(tgt, pos)
  if g.phase == "ASCENT" and pos.y >= g.climbY then
    if g.mode == "WATER" then
      g.phase = "CRUISE"
    else
      g.phase, g.tmode, g.bias = "TRANSFER", "AUTO", 0
      local bx, bz = tgt.x - pos.x, tgt.z - pos.z
      if C.EXPRESS and math.sqrt(bx * bx + bz * bz) > (g.mode == "ARC" and C.ARC_AB_MIN or C.AB_MIN_DIST) then
        g.phase, g.tmode = "EXPRESS", "AB"
      elseif g.mode == "ARC" then
        g.phase, g.tmode = "DIVE", "DIVE"
      end
    end
  end
  if g.phase == "EXPRESS" then
    local hx, hz = tgt.x - pos.x, tgt.z - pos.z
    local dh = math.sqrt(hx * hx + hz * hz)
    local arc = g.mode == "ARC"
    local vEnd = arc and C.DIVE_V or C.NET_VMAX
    local handoff = arc and C.DIVE_V * C.DIVE_V / (2 * C.DIVE_AH) + C.DIVE_R * math.max(pos.y - tgt.y - C.NOSE_OFFSET, 0) or C.NET_VMAX * C.NET_VMAX / (2 * C.NET_ABRAKE) + C.AB_MARGIN
    local v = len(vel)
    if g.tmode == "BRAKE" and dh - handoff > C.AB_REARM_D then
      local sp0 = math.sqrt(vEnd * vEnd + 2 * C.AB_BRAKE * math.max(dh - handoff, 0))
      if v < C.AB_REARM_K * sp0 then g.tmode = "AB" end
    end
    if dh <= handoff or (g.tmode == "BRAKE" and v <= vEnd + 5 and dh - handoff <= C.AB_REARM_D) then
      g.phase, g.tmode, g.bias = arc and "DIVE" or "TRANSFER", arc and "DIVE" or "AUTO", 0
    elseif g.tmode == "BRAKE" or (dh - handoff) <= math.max(v * v - vEnd * vEnd, 0) / (2 * C.AB_BRAKE) + C.AB_PAD then
      g.tmode = "BRAKE"
      local sp = math.min(C.AB_VMAX, math.sqrt(vEnd * vEnd + 2 * C.AB_BRAKE * math.max(dh - handoff, 0)))
      return velCmd(g, { x = hx / dh * sp, y = clamp((g.P.y - pos.y) * 0.5, -10, 10), z = hz / dh * sp }, vel, nose, dt, C.NET_TILT, C.AB_MAX_THR, true)
    else
      g.tmode = "AB"
      local abFloor = math.min(C.AB_MAX_THR, (C.GRAV + 1) / (C.THRUST_ACC * math.max(math.sin(math.rad(C.AB_PITCH_UP)), 0.05)))
      if v < C.AB_VMAX - 5 or g.throttle < abFloor then
        g.throttle = math.min(g.throttle + C.WATER_RAMP * dt, C.AB_MAX_THR)
      elseif v > C.AB_VMAX then
        g.throttle = math.max(g.throttle - 1.5 * dt, abFloor)
      end
      return levelDir(g, pos, vel, hx / dh, hz / dh, dt)
    end
  end
  if g.phase == "DIVE" then
    local tip = add(pos, mul(nose, C.NOSE_OFFSET))
    if len(vel) < C.DIVE_STOP_V then g.stuck = (g.stuck or 0) + dt else g.stuck = 0 end
    if tip.y <= tgt.y or g.stuck >= C.DIVE_STOP_T then
      g.phase, g.throttle, g.tmode = "TOUCHDOWN", 0, "OFF"
      return nil, true
    end
    return arcDir(g, pos, vel)
  end
  if g.phase == "ASCENT" then
    local want = UP
    if g.mode == "WATER" then
      local f = clamp((pos.y - g.start.y) / g.climb, 0, 1)
      f = f * f
      want = unit(add(mul(UP, 1 - f), mul(unit(los), f)))
    end
    g.cmd = turnToward(g.cmd, want, math.rad(C.TURN_RATE) * dt)
    return align(g.cmd, vel)
  elseif g.phase == "CRUISE" then
    local want = len(los) > C.WATER_SLOW_DIST and C.WATER_FAST_THR or C.CRUISE_THR
    if g.throttle < want then
      g.throttle = math.min(g.throttle + C.WATER_RAMP * dt, want)
    else
      g.throttle = want
    end
    return approachDir(g, pos, vel, nose)
  elseif g.phase == "TRANSFER" then
    local to = sub(g.P, pos)
    local dP = len(to)
    local hx, hz = tgt.x - pos.x, tgt.z - pos.z
    local dh = math.sqrt(hx * hx + hz * hz)
    if dh < C.NET_SINK_RADIUS and len(vel) < C.NET_SINK_SPEED then
      g.phase = "SINK"
    else
      if C.NET_HOLD_ALT then
        local sp = math.min(C.NET_VMAX, math.sqrt(2 * C.NET_ABRAKE * dh), C.NET_POS_GAIN * dh)
        local vx, vz = 0, 0
        if dh > 1e-6 then vx, vz = hx / dh * sp, hz / dh * sp end
        return velCmd(g, { x = vx, y = clamp((g.P.y - pos.y) * 0.5, -10, 10), z = vz }, vel, nose, dt, C.NET_TILT)
      end
      return velCmd(g, mul(unit(to), math.min(C.NET_VMAX, math.sqrt(2 * C.NET_ABRAKE * dP), C.NET_POS_GAIN * dP)), vel, nose, dt, C.NET_TILT)
    end
  end
  if g.phase == "SINK" then
    local h = pos.y - C.TAIL_OFFSET - (tgt.y + C.NET_CLEAR)
    if h < C.NET_SLOW and vel.y > -C.TOUCH_VY then g.touch = (g.touch or 0) + dt else g.touch = 0 end
    if h <= 0 or g.touch >= C.TOUCH_TIME then
      g.phase, g.throttle, g.tmode = "RELEASE", 0, h <= 0 and "OFF" or "TOUCH"
      return nil, true
    end
    local vy = -math.max(C.NET_SINK, math.min(C.NET_VDESC, math.sqrt(2 * C.NET_ABRAKE * math.max(h - C.NET_SLOW, 0))))
    return velCmd(g, { x = clamp((tgt.x - pos.x) * C.SINK_POS_GAIN, -C.SINK_POS_V, C.SINK_POS_V), y = vy, z = clamp((tgt.z - pos.z) * C.SINK_POS_GAIN, -C.SINK_POS_V, C.SINK_POS_V) }, vel, nose, dt, C.NET_SINK_TILT)
  end
end

local function newTracker() return { bestD = math.huge } end

local function track(tr, p, tgt)
  local d = len(sub(tgt, p))
  if d < tr.bestD then tr.bestD, tr.best = d, p end
  if tr.prev and not tr.pass and tr.prev.y > tgt.y and p.y <= tgt.y then
    local f = (tr.prev.y - tgt.y) / (tr.prev.y - p.y)
    tr.pass = add(tr.prev, mul(sub(p, tr.prev), f))
  end
  tr.prev = p
end

local function missOf(tr, start, tgt)
  local p = tr.pass or tr.best
  if not p then return nil end
  local mx, mz = p.x - tgt.x, p.z - tgt.z
  local fx, fz = tgt.x - start.x, tgt.z - start.z
  local fd = math.sqrt(fx * fx + fz * fz)
  if fd < 1e-6 then fx, fz, fd = 0, -1, 1 end
  fx, fz = fx / fd, fz / fd
  return { total = math.sqrt(mx * mx + mz * mz), long = mx * fx + mz * fz, right = mz * fx - mx * fz, crossed = tr.pass ~= nil }
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
  while t < C.SIM_TIME do
    local near = len(sub(tgt, pos)) < 150
    local dt = (g.phase == "IGNITION" or g.phase == "ASCENT" or near) and 0.1 or 0.25
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
      local rate = clamp(C.SIM_TURN * thr + 0.02 * len(vel), 0.3, 3)
      nose = turnToward(nose, dd, math.min(rate * dt, ang * math.min(1, dt / 0.3)))
    end
    local acc = add(mul(nose, thr * C.THRUST_ACC), mul(vel, -C.DRAG))
    acc = sub(acc, mul(sub(vel, mul(nose, dot(vel, nose))), C.AB_LIFT))
    acc.y = acc.y - C.GRAV
    vel = add(vel, mul(acc, dt))
    pos = add(pos, mul(vel, dt))
    if g.phase == "IGNITION" and pos.y < start.y then
      pos.y = start.y
      vel = { x = 0, y = 0, z = 0 }
    end
    t = t + dt
    if g.phase ~= "IGNITION" and g.phase ~= "ASCENT" then track(tr, contactPoint(g, pos, nose), tgt) end
    if t >= nextRec then
      pts[#pts + 1] = pos
      nextRec = t + 0.25
    end
    if done and (tr.pass or t - tDone > 4) then break end
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

local function fluidSum(list)
  local s, any = 0, false
  for _, n in ipairs(list) do
    local a = fluidAmount(n)
    if a then s, any = s + a, true end
  end
  return any and s or nil
end

local function telemetry()
  while true do
    local ok, r = pcall(function() return sublevel.isInPlotGrid() end)
    tel.sub = ok and r or false
    if not tel.flying then
      tel.pos = nil
      if tel.sub then
        local ok2, p = pcall(function() return sublevel.getLogicalPose() end)
        if ok2 and p then tel.pos = v3(p.position) end
        pcall(tune)
      end
    end
    tel.eng = fluidSum(E.names)
    tel.tank = fluidSum(E.tanks)
    os.queueEvent("telemetry")
    sleep(0.5)
  end
end

local function fuel()
  while true do
    local fns = {}
    for i = 1, E.N do
      local e = E.eng[i]
      for _, t in ipairs(E.tanks) do
        fns[#fns + 1] = function() pcall(e.pullFluid, t) end
      end
    end
    if #fns > 0 then parallel.waitForAll(table.unpack(fns)) end
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
  text(2, 1, "ROCKET4 x" .. E.N, colors.white, colors.gray)
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
    local m = start and "NO DESTINATION" or "NO POSITION"
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
  end
end

local function fieldRow(i) return 3 + i end

local function destination()
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
  return table.concat({ fmt(start.x), fmt(start.y), fmt(start.z), tgt.x, tgt.y, tgt.z, md, cl, string.format("%.1f", C.THRUST_ACC) }, ",")
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
  text(2, 3, "DESTINATION", colors.yellow)
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
  text(2, 14, "MASS " .. fmt(E.mass) .. "  HOVER " .. (E.hover and (fmt(E.hover * 100) .. "%") or "--"), colors.white)
  text(2, 15, "ENGINE " .. fuelStr(tel.eng), colors.white)
  text(2, 16, "TANK   " .. fuelStr(tel.tank), colors.white)
  climb = climbValue() or C.CLIMB
  local tgt = destination()
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
  local t = destination()
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

local fl = { phase = "IGNITION", trail = {} }

local SAFE_UP = { IGNITION = true, ASCENT = true, TRANSFER = true, SINK = true }

local function safety(g, pos, nose, dir, om, dt, now)
  local tilt = math.deg(math.acos(clamp(nose.y, -1, 1)))
  local err = dir and math.deg(math.acos(clamp(dot(nose, unit(dir)), -1, 1))) or 0
  fl.tilt, fl.err = tilt, err
  if g.safePhase ~= g.phase then
    g.safeFrom, g.safePhase, g.safeT, g.badT, g.errPrev, g.closing = g.safePhase, g.phase, now, 0, nil, 0

  end
  local up = SAFE_UP[g.phase] and not (g.phase == "ASCENT" and g.mode == "WATER")
  local low = pos.y - math.min(g.start.y, g.tgt.y) < C.CUT_LOW
  if (up or g.rec) and low and tilt > C.CUT_TILT then
    g.cutT = (g.cutT or 0) + dt
    if g.cutT >= C.CUT_HOLD then return "TILT", true end
  else
    g.cutT = 0
  end
  if g.rec or g.phase == "IGNITION" then return nil end
  local why
  if up then
    local grace = g.safeFrom == "EXPRESS" and C.CUT_GRACE or 0
    if tilt > C.REC_TILT and now - g.safeT >= grace then why = "TILT" end
  elseif dir then
    if g.errPrev then g.closing = g.closing + ((g.errPrev - err) / dt - g.closing) * 0.2 end
    if err > C.CUT_ERR and g.closing < C.CUT_CLOSE then why = "ATTITUDE" end
  end
  g.errPrev = err
  if not why and len(om) > C.CUT_SPIN then why = "SPIN" end
  g.badT = why and g.badT + dt or 0
  if why and g.badT >= (why == "ATTITUDE" and C.CUT_HOLD_ERR or C.CUT_HOLD) then return why, false end
end

local function recoverStep(g, pos, vel, nose, om, dt)
  local tilt = math.deg(math.acos(clamp(nose.y, -1, 1)))
  g.recT = (g.recT or 0) + dt
  if (tilt < C.REC_EXIT and len(om) < C.REC_SPIN) or (g.recT > C.REC_MAX_T and tilt < C.REC_TIMEOUT_TILT) then
    g.recOk = g.recOk + dt
    if g.recOk >= C.REC_HOLD then
      g.rec = false
      if g.phase == "EXPRESS" or g.phase == "DIVE" then g.phase, g.tmode, g.bias = "TRANSFER", "AUTO", 0 end
      return nil
    end
  else
    g.recOk = 0
  end
  g.tmode = "RECOVER"
  local d = velCmd(g, { x = 0, y = clamp((g.recY - pos.y) * 0.3, -5, 5), z = 0 }, vel, nose, dt, C.REC_CMD_TILT)
  if vel.y < C.REC_CLIMB_V then g.throttle = math.max(g.throttle, math.min((E.hover or 0.34) * C.REC_MIN_HOVER, C.NET_MAX_THR)) end
  if vel.y > C.REC_CLIMB_V then g.throttle = math.min(g.throttle, (E.hover or 0.34) * 0.8) end
  return d
end

local function drawFlight(start, tgt)
  clear()
  local colorsFor = { IGNITION = colors.yellow, ASCENT = colors.orange, CRUISE = colors.cyan, TRANSFER = colors.cyan, EXPRESS = colors.red, DIVE = colors.magenta, SINK = colors.lightBlue, RELEASE = colors.lime, TOUCHDOWN = colors.red, CUT = colors.red, ABORT = colors.red, RECOVER = colors.orange }
  header(fl.phase .. (fl.cut and (" " .. fl.cut) or "") .. " ", colorsFor[fl.phase] or colors.white)
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
  text(2, 12, "Y " .. (fl.gy and string.format("%+.2f", fl.gy) or "--") .. "  R " .. string.format("%+.2f", fl.r or 0), colors.white)
  text(2, 13, string.format("TILT %.0f  LOOP %d ms", fl.tilt or 0, math.floor(E.ms or 0)), colors.gray)
  text(2, 14, "POS " .. fmt(p and p.x) .. " " .. fmt(p and p.y) .. " " .. fmt(p and p.z), colors.gray)
  text(2, 15, "ENGINE " .. fuelStr(tel.eng), colors.white)
  text(2, 16, "TANK   " .. fuelStr(tel.tank), colors.white)
  plot(start, tgt, p, fl.pred, fl.trail)
  local r = fl.result
  if r then
    text(2, 17, string.format("MISS %.1f  L%+.1f R%+.1f", r.total, r.long, r.right), r.total <= 2 and colors.lime or colors.orange)
  end
  text(2, 18, "TGT " .. fmt(tgt.x) .. " " .. fmt(tgt.y) .. " " .. fmt(tgt.z), colors.gray)
  text(2, 19, "BKSP cut engines", colors.gray)
end

local function writeLog(tgt, r, speed, t, tag)
  local ok, d = pcall(os.date, "%Y-%m-%d %H:%M")
  local f = fs.open(C.LOG, "a")
  if not f then return end
  f.writeLine(string.format("%s %s TGT %d %d %d MISS %.1f LONG %+.1f RIGHT %+.1f SPEED %.0f TIME %.1f%s%s",
    ok and d or "?", mode, math.floor(tgt.x), math.floor(tgt.y), math.floor(tgt.z),
    r.total, r.long, r.right, speed or 0, t or 0, r.crossed and "" or " CLOSEST", tag and (" CUT " .. tag) or ""))
  f.close()
end

local function flight(tgt, pred)
  local start, sw, su = E.io(false)
  E.psi0 = heading(sw, su)
  local g = newGuide(start, tgt, mode, climb)
  local tr = newTracker()
  fl.trail, fl.pred, fl.result, fl.cut, fl.r, fl.tilt, fl.err = {}, pred, nil, nil, 0, 0, 0
  tel.flying = true
  local function setDir(dirW, w, u, om, dt, vel)
    local sinking = C.SINK_FIX and mode ~= "WATER" and g.phase == "SINK"
    if sinking and not g.sinkStarted then
      g.sinkStarted = true
      sinkI.x, sinkI.z = 0, 0
    end
    local d, wff = unit(dirW), nil
    if g.phase == "DIVE" and g.dPrev then
      local c = dot(g.dPrev, d) > math.cos(math.rad(C.DIVE_WFF_STEP)) and mul(cross(g.dPrev, d), 1 / dt) or { x = 0, y = 0, z = 0 }
      g.wff = g.wff and add(g.wff, mul(sub(c, g.wff), math.min(1, dt / C.DIVE_WFF_TAU))) or c
      local m = len(g.wff)
      wff = toBody(w, u, m > C.DIVE_WFF_MAX and mul(g.wff, C.DIVE_WFF_MAX / m) or g.wff)
    end
    g.dPrev = g.phase == "DIVE" and d or nil
    fl.gx, fl.gy = steer(dirW, w, u, om, dt, g.throttle, C.NET_ATT_BOOST and mode ~= "WATER" and (g.phase == "TRANSFER" or g.phase == "SINK"), sinking, g.phase == "EXPRESS" or g.phase == "DIVE", vel, wff)
  end
  local function guide()
    local t0 = os.clock()
    local last, nextTrail, nextLog = t0, t0, t0
    local flog = fs.open(C.FLIGHT_LOG, "w")
    if flog then
      flog.writeLine(string.format("%s TGT %d %d %d CLIMB %d MASS %.1f TACC %.1f HOVER %.3f", mode, math.floor(tgt.x), math.floor(tgt.y), math.floor(tgt.z), climb, E.mass or 0, C.THRUST_ACC, E.hover or 0))
      flog.writeLine("t phase thr speed alt dist dh err_deg gx gy spin tilt roll wy loop_ms")
    end
    E.set(g.throttle, 0, 0, 0)
    local pos, w, u, vel, om = E.io(true)
    while true do
      if fl.abort then
        if flog then
          flog.writeLine("ABORT")
          flog.close()
          flog = nil
        end
        E.cut()
        fl.thr, fl.gx, fl.gy, fl.r, fl.phase = 0, 0, 0, 0, "ABORT"
        return
      end
      local now = os.clock()
      local dt = clamp(now - last, 0.01, 0.5)
      last = now
      local nose = rotate(w, u, UP)
      local dir, done
      if g.rec then
        dir = recoverStep(g, pos, vel, nose, om, dt)
        if not g.rec and flog then flog.writeLine(string.format("RESUME %.2f", now - t0)) end
      end
      if not g.rec then dir, done = guideStep(g, pos, vel, nose, dt) end
      if not done and not g.rec then
        local ceil = math.max(g.start.y, g.tgt.y) + C.CEILING
        if vel.y > 0 and pos.y + vel.y * vel.y / (2 * C.GRAV) > ceil then
          g.throttle = math.min(g.throttle, C.CEIL_THR)
          if g.abLock and g.abY > ceil - 50 then g.abY = ceil - 50 end
        end
      end
      if dir and not done and (g.rec or g.phase == "TRANSFER" or g.phase == "SINK" or g.phase == "EXPRESS") then
        g.cmdS = turnToward(g.cmdS or nose, unit(dir), math.rad(C.CMD_RATE) * dt)
        dir = g.cmdS
      else
        g.cmdS = nil
      end
      if not done then
        local why, cut = safety(g, pos, nose, dir, om, dt, now)
        if cut then
          g.phase, g.tmode, g.throttle, g.rec, fl.cut, done = "CUT", why, 0, false, why, true
        elseif why and not g.rec then
          g.rec, g.recY, g.recOk, g.recT = true, pos.y, 0, 0
          if flog then flog.writeLine(string.format("RECOVER %s %.2f", why, now - t0)) end
          dir = recoverStep(g, pos, vel, nose, om, dt)
        end
      end
      fl.phase, fl.tmode, fl.thr = g.rec and "RECOVER" or g.phase, g.tmode, g.throttle
      fl.pos, fl.speed, fl.dist = pos, len(vel), len(sub(tgt, pos))
      if g.phase ~= "IGNITION" and g.phase ~= "ASCENT" then track(tr, contactPoint(g, pos, nose), tgt) end
      if now >= nextTrail then
        fl.trail[#fl.trail + 1] = pos
        nextTrail = now + 0.25
      end
      if flog and now >= nextLog then
        local dhx, dhz = tgt.x - pos.x, tgt.z - pos.z
        flog.writeLine(string.format("%.2f %s %.2f %.1f %.1f %.1f %.1f %.1f %+.2f %+.2f %.2f %.1f %+.2f %+.2f %d",
          now - t0, fl.phase, g.throttle, len(vel), pos.y, len(sub(tgt, pos)), math.sqrt(dhx * dhx + dhz * dhz), fl.err or 0,
          fl.gx or 0, fl.gy or 0, len(om), fl.tilt or 0, fl.r or 0, om.y, math.floor(E.ms or 0)))
        flog.flush()
        nextLog = now + C.LOG_DT
      end
      if done then
        if flog then
          if fl.cut then flog.writeLine("CUT " .. fl.cut) end
          flog.close()
          flog = nil
        end
        E.cut()
        fl.thr, fl.gx, fl.gy, fl.r = 0, 0, 0, 0
        local cutSpeed, cutTime = len(vel), now - t0
        local flareOff
        if mode == "WATER" and not fl.cut then
          pcall(redstone.setOutput, C.FLARE_SIDE, true)
          flareOff = os.clock() + C.FLARE_PULSE
        end
        local stop, still = os.clock() + 4, 0
        while os.clock() < stop and not tr.pass do
          if flareOff and os.clock() >= flareOff then
            pcall(redstone.setOutput, C.FLARE_SIDE, false)
            flareOff = nil
          end
          local ok, p2, w2, u2, v2 = pcall(E.io, false)
          if not ok then break end
          track(tr, contactPoint(g, p2, rotate(w2, u2, UP)), tgt)
          fl.pos, fl.speed, fl.dist = p2, len(v2), len(sub(tgt, p2))
          fl.trail[#fl.trail + 1] = p2
          still = len(v2) < 0.5 and still + 1 or 0
          if still > 30 then break end
        end
        if flareOff then
          sleep(math.max(0, flareOff - os.clock()))
          pcall(redstone.setOutput, C.FLARE_SIDE, false)
        end
        fl.result = missOf(tr, start, tgt)
        if fl.result then pcall(writeLog, tgt, fl.result, cutSpeed, cutTime, fl.cut) end
        if REMOTE and fl.result then pcall(REMOTE.result, fl.result, cutSpeed, cutTime) end
        return
      end
      if dir then
        setDir(dir, w, u, om, dt, vel)
        fl.r = (g.rec or g.phase ~= "IGNITION") and rollCmd(w, u, om, nose, g.throttle, fl.gx, fl.gy) or 0
      else
        fl.gx, fl.gy, fl.r = 0, 0, 0
      end
      if dir and (g.rec or g.phase == "EXPRESS" or g.phase == "TRANSFER" or g.phase == "DIVE") and (fl.err or 0) > C.AUTH_ERR and len(vel) > C.AUTH_V and not (vel.y > C.REC_CLIMB_V and nose.y > 0.7) then
        g.throttle = math.max(g.throttle, C.AUTH_THR)
      end
      E.set(g.throttle, fl.gx, fl.gy, fl.r)
      pos, w, u, vel, om = E.io(true)
    end
  end
  local function display()
    while true do
      drawFlight(start, tgt)
      sleep(0.2)
    end
  end
  local function keyAbort()
    while true do
      local _, k = os.pullEvent("key")
      if k == keys.backspace then fl.abort = true end
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
    parallel.waitForAny(guide, display, keyAbort, uplink)
  else
    parallel.waitForAny(guide, display, keyAbort)
  end
  tel.flying = false
  drawFlight(start, tgt)
end

local function main()
  E.cut()
  pcall(redstone.setOutput, C.FLARE_SIDE, false)
  pcall(tune)
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

local ok, err = pcall(parallel.waitForAny, main, telemetry, fuel)
E.cut()
term.setCursorBlink(false)
term.setBackgroundColor(colors.black)
term.setTextColor(colors.white)
if not ok then
  term.setCursorPos(1, H - 1)
  term.setTextColor(colors.red)
  term.write(tostring(err))
end
term.setCursorPos(1, H)
