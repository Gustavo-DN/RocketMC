local C = {
  OUT = "vector_map.txt",
  RAW = "vector_map.csv",
  DAT = "vector_map.dat",
  ENGINE_TYPE = "liquid_vector_thruster",
  TANK_TYPE = "fluid_storage",
  WHERE = {
    liquid_vector_thruster_0 = "front",
    liquid_vector_thruster_1 = "front-right",
    liquid_vector_thruster_2 = "under computer",
    liquid_vector_thruster_3 = "right",
  },
  DELAY = 5,
  GRAV = 10.5,
  THRUST_FORCE = 2000,
  CLIMB = 30,
  CLIMB_TIMEOUT = 30,
  IGN_TIMEOUT = 40,
  THR_RAMP = 0.05,
  MAX_THR = 0.85,
  LIFTOFF_VY = 0.5,
  HOLD_KV = 0.03,
  HOLD_KI = 0.02,
  KP = 1.5,
  KD = 1.6,
  KI = 0.5,
  IWIN = 0.35,
  IMAX = 0.6,
  VMAX = 1.0,
  ROLL_KP = 0.4,
  ROLL_KD = 0.8,
  ROLL_MAX = 0.3,
  DRIFT_K = 0.03,
  DRIFT_MAX = 0.08,
  LAG_AMP = 1.0,
  LAG_TIME = 1.5,
  CAL = { 0.3, 0.6, 0.85 },
  CAL_TIME = 1.5,
  AMP = 0.8,
  AMP_ALL = 0.4,
  AMP_ROLL = 0.4,
  DTHR = 0.15,
  PULSE = 0.6,
  SKIP = 0.15,
  SETTLE = 2.0,
  SETTLE_OM = 0.05,
  SETTLE_TILT = 3,
  SETTLE_MAX = 10,
  STEP_TILT = 10,
  STEP_TIME = 4,
  TEST_TILT = 25,
  CUT_TILT = 60,
  CUT_LOW = 20,
  DESC_V = 4,
  LAND_V = 1,
}

local names, eng = {}, {}
for _, n in ipairs(peripheral.getNames()) do
  if peripheral.hasType(n, C.ENGINE_TYPE) then names[#names + 1] = n end
end
table.sort(names)
if #names == 0 then error("no " .. C.ENGINE_TYPE .. " found", 0) end
for i, n in ipairs(names) do eng[i] = peripheral.wrap(n) end
local N = #eng
local SX, SY = { 1, 1, -1, -1 }, { 1, -1, 1, -1 }
local tanks = {}
for _, n in ipairs(peripheral.getNames()) do
  if peripheral.hasType(n, C.TANK_TYPE) and not peripheral.hasType(n, C.ENGINE_TYPE) then tanks[#tanks + 1] = n end
end

local atan2 = math.atan2 or math.atan
local UP = { x = 0, y = 1, z = 0 }
local function v3(a) return { x = a.x or a[1], y = a.y or a[2], z = a.z or a[3] } end
local function add(a, b) return { x = a.x + b.x, y = a.y + b.y, z = a.z + b.z } end
local function mul(a, s) return { x = a.x * s, y = a.y * s, z = a.z * s } end
local function cross(a, b) return { x = a.y * b.z - a.z * b.y, y = a.z * b.x - a.x * b.z, z = a.x * b.y - a.y * b.x } end
local function len(a) return math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z) end
local function unit(a)
  local l = len(a)
  if l < 1e-9 then return { x = 0, y = 1, z = 0 } end
  return mul(a, 1 / l)
end
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

local STOP, CUT = {}, {}
local raw = fs.open(C.RAW, "w")
local cmd = {}
for i = 1, N do cmd[i] = { 0, 0, 0 } end
local cur = { phase = "PAD", base = 0, alt = 0, loopMs = 0, tilt = 0, dt = 0.05 }
local st = {}
local ctl = { ix = 0, iz = 0, hov = 0.34, hovI = 0, holdY = 0 }
local res = { tests = {}, step = {}, lag = {}, cal = {} }
local start
local t0 = os.clock()

local function heading()
  local b = rotate(st.w, st.u, { x = 1, y = 0, z = 0 })
  return atan2(-b.z, b.x)
end

local function io(send)
  local fns, p, v, o = {}
  if send then
    for i = 1, N do
      local e, c = eng[i], cmd[i]
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
  cur.loopMs = os.epoch("utc") - a
  local w, u = quat(p.orientation)
  st.w, st.u, st.pos, st.vel, st.om = w, u, v3(p.position), v3(v), v3(o)
  st.nose = rotate(w, u, UP)
  cur.alt = st.pos.y
  cur.tilt = math.deg(math.acos(clamp(st.nose.y, -1, 1)))
  cur.n = (cur.n or 0) + 1
  local quiet = cur.phase == "SETTLE" or cur.phase == "CLIMB" or cur.phase == "LAND"
  if cur.rec and (not quiet or cur.n % 4 == 0) then
    local c = {}
    for i = 1, N do c[i] = string.format("%.3f,%.2f,%.2f", cmd[i][1], cmd[i][2], cmd[i][3]) end
    raw.writeLine(string.format("%.2f,%s,%d,", os.clock() - t0, cur.phase, cur.loopMs) .. table.concat(c, ",")
      .. string.format(",%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f", st.pos.x, st.pos.y, st.pos.z,
        st.vel.x, st.vel.y, st.vel.z, st.om.x, st.om.y, st.om.z, cur.tilt))
  end
end

local function setAll(thr, gx, gy, r)
  cur.base = thr
  for i = 1, N do
    cmd[i][1] = clamp(thr, 0, 1)
    cmd[i][2] = clamp(gx + (N == 4 and SX[i] or 0) * r, -1, 1)
    cmd[i][3] = clamp(gy + (N == 4 and SY[i] or 0) * r, -1, 1)
  end
end

local function cutAll()
  setAll(0, 0, 0, 0)
  if not pcall(io, true) then
    for i = 1, N do
      pcall(eng[i].setThrustNormalized, 0)
      pcall(eng[i].setVector, 0, 0)
    end
  end
end

local function tick()
  io(true)
  local now = os.clock()
  cur.dt = clamp(now - (cur.last or now), 0.01, 0.5)
  cur.last = now
  if cur.cut then cur.why = cur.why or "KEY" error(CUT, 0) end
  if start and cur.tilt > C.CUT_TILT and st.pos.y - start.y < C.CUT_LOW then
    cur.why = "TILT LOW"
    error(CUT, 0)
  end
  if cur.testing and cur.abort then cur.why = "KEY" error(STOP, 0) end
  if cur.testing and cur.tilt > C.TEST_TILT then cur.why = "TEST TILT" error(STOP, 0) end
end

local function vThr(tvy)
  ctl.hovI = clamp(ctl.hovI + C.HOLD_KI * (tvy - st.vel.y) * cur.dt, -0.1, 0.1)
  return clamp(ctl.hov + ctl.hovI + C.HOLD_KV * (tvy - st.vel.y), 0.02, C.MAX_THR)
end

local function attCmd(dirW)
  local d = unit(toBody(st.w, st.u, dirW))
  local ax, az = d.z, -d.x
  local s = math.sqrt(ax * ax + az * az)
  local ex, ez = 0, 0
  if s > 1e-9 then
    local a = atan2(s, d.y) / s
    ex, ez = ax * a, az * a
  end
  if math.sqrt(ex * ex + ez * ez) < C.IWIN then
    ctl.ix = clamp(ctl.ix + ex * cur.dt, -C.IMAX, C.IMAX)
    ctl.iz = clamp(ctl.iz + ez * cur.dt, -C.IMAX, C.IMAX)
  end
  local gx = clamp(C.KP * ez + C.KI * ctl.iz - C.KD * st.om.z, -C.VMAX, C.VMAX)
  local gy = clamp(-(C.KP * ex + C.KI * ctl.ix - C.KD * st.om.x), -C.VMAX, C.VMAX)
  local r = 0
  if N == 4 then
    r = -C.ROLL_KD * st.om.y
    if ctl.psi0 then r = r + C.ROLL_KP * ((ctl.psi0 - heading() + math.pi) % (2 * math.pi) - math.pi) end
    local lim = math.min(C.ROLL_MAX, math.max(1 - math.max(math.abs(gx), math.abs(gy)), 0))
    r = clamp(r, -lim, lim)
  end
  return gx, gy, r
end

local function fly(dirW, tvy)
  if not dirW then
    dirW = unit({
      x = clamp(-C.DRIFT_K * st.vel.x, -C.DRIFT_MAX, C.DRIFT_MAX),
      y = 1,
      z = clamp(-C.DRIFT_K * st.vel.z, -C.DRIFT_MAX, C.DRIFT_MAX),
    })
  end
  local gx, gy, r = attCmd(dirW)
  setAll(vThr(tvy), gx, gy, r)
end

local function holdV() return clamp((ctl.holdY - st.pos.y) * 0.3, -2, 2) end

local function slope(s, k, c, a, b)
  local n, sx, sy, sxx, sxy = 0, 0, 0, 0, 0
  for _, p in ipairs(s) do
    if p.t >= a and p.t <= b then
      local x, y = p.t, p[k][c]
      n, sx, sy, sxx, sxy = n + 1, sx + x, sy + y, sxx + x * x, sxy + x * y
    end
  end
  local d = n * sxx - sx * sx
  if n < 3 or math.abs(d) < 1e-9 then return 0 end
  return (n * sxy - sx * sy) / d
end

local function poll(getter)
  local vals, fns = {}, {}
  for i = 1, N do
    local e = eng[i]
    fns[i] = function()
      local ok, v = pcall(e[getter])
      vals[i] = ok and tonumber(v) or 0
    end
  end
  parallel.waitForAll(table.unpack(fns))
  return vals
end

local function series(getter, apply, secs)
  local v0 = poll(getter)
  apply()
  io(true)
  local a, s = os.epoch("utc"), {}
  while os.epoch("utc") - a < secs * 1000 do
    s[#s + 1] = { t = (os.epoch("utc") - a) / 1000, v = poll(getter) }
  end
  local out = {}
  for i = 1, N do
    local fin = s[#s].v[i]
    local d = fin - v0[i]
    local r = { from = v0[i], final = fin }
    for _, p in ipairs(s) do
      local f = math.abs(d) > 1e-9 and (p.v[i] - v0[i]) / d or 0
      if not r.t63 and f >= 0.63 then r.t63 = p.t end
      if not r.t90 and f >= 0.9 then r.t90 = p.t end
    end
    out[i] = r
  end
  return out
end

local function padTests()
  cur.phase = "PAD GIMBAL X"
  setAll(0, 0, 0, 0)
  io(true)
  sleep(0.5)
  res.lag.x = series("getVectorX", function() setAll(0, C.LAG_AMP, 0, 0) end, C.LAG_TIME)
  setAll(0, 0, 0, 0)
  io(true)
  sleep(0.5)
  cur.phase = "PAD GIMBAL Y"
  res.lag.y = series("getVectorY", function() setAll(0, 0, C.LAG_AMP, 0) end, C.LAG_TIME)
  setAll(0, 0, 0, 0)
  io(true)
  sleep(0.5)
  for k, f in ipairs(C.CAL) do
    if cur.abort then break end
    local lvl = f * res.hoverEst
    cur.phase = string.format("PAD THRUST %.2f", lvl)
    res.cal[k] = { thr = lvl, eng = series("getThrust", function() setAll(lvl, 0, 0, 0) end, C.CAL_TIME) }
  end
  cutAll()
  sleep(1)
end

local function settle()
  cur.phase = "SETTLE"
  local te, calm = os.clock() + C.SETTLE_MAX, nil
  while os.clock() < te do
    fly(nil, holdV())
    tick()
    local now = os.clock()
    if len(st.om) < C.SETTLE_OM and cur.tilt < C.SETTLE_TILT then
      calm = calm or now
      if now - calm >= C.SETTLE then return end
    else
      calm = nil
    end
  end
end

local function chan(list, k, amp)
  return function(sg, base)
    local s = 0
    for _, i in ipairs(list) do
      cmd[i][k] = clamp(base[i][k] + sg * amp, k == 1 and 0 or -1, 1)
      s = s + cmd[i][k] - base[i][k]
    end
    return s / #list
  end
end

local function rollChan(amp)
  return function(sg, base)
    local s = 0
    for i = 1, N do
      cmd[i][2] = clamp(base[i][2] + sg * SX[i] * amp, -1, 1)
      cmd[i][3] = clamp(base[i][3] + sg * SY[i] * amp, -1, 1)
      s = s + (cmd[i][2] - base[i][2]) * SX[i] + (cmd[i][3] - base[i][3]) * SY[i]
    end
    return s / (2 * N)
  end
end

local function doublet(label, mod)
  settle()
  cur.phase = label
  local base = {}
  for i = 1, N do base[i] = { cmd[i][1], cmd[i][2], cmd[i][3] } end
  local s, amp, a, P = {}, {}, os.clock(), cur.pulse
  for ph = 1, 2 do
    for i = 1, N do for k = 1, 3 do cmd[i][k] = base[i][k] end end
    amp[ph] = mod(ph == 1 and 1 or -1, base)
    local te = a + ph * P
    while os.clock() < te do
      tick()
      s[#s + 1] = { t = os.clock() - a, om = st.om, bv = toBody(st.w, st.u, st.vel) }
    end
  end
  for i = 1, N do for k = 1, 3 do cmd[i][k] = base[i][k] end end
  local eff = (amp[1] - amp[2]) / 2
  local div = 2 * (math.abs(eff) > 1e-6 and eff or 1)
  local r = { label = label, eff = eff, om = {}, acc = {} }
  for _, c in ipairs({ "x", "y", "z" }) do
    r.om[c] = (slope(s, "om", c, cur.skip, P) - slope(s, "om", c, P + cur.skip, 2 * P)) / div
    r.acc[c] = (slope(s, "bv", c, cur.skip, P) - slope(s, "bv", c, P + cur.skip, 2 * P)) / div
  end
  res.tests[#res.tests + 1] = r
end

local function stepTest(label, ax, az)
  settle()
  local a = math.rad(C.STEP_TILT)
  local tgt = { x = math.sin(a) * ax, y = math.cos(a), z = math.sin(a) * az }
  local out = { label = label }
  for ph = 1, 2 do
    local dirW = ph == 1 and tgt or UP
    cur.phase = label .. (ph == 1 and " OUT" or " BACK")
    local ts = os.clock()
    local rise, over, settleT = nil, 0, 0
    while os.clock() - ts < C.STEP_TIME do
      fly(dirW, holdV())
      tick()
      local t = os.clock() - ts
      local err = math.deg(math.acos(clamp(st.nose.x * dirW.x + st.nose.y * dirW.y + st.nose.z * dirW.z, -1, 1)))
      if not rise and err <= 0.2 * C.STEP_TILT then rise = t end
      if rise then over = math.max(over, err) end
      if err > 0.2 * C.STEP_TILT then settleT = t end
    end
    out[ph] = { rise = rise, over = over, settle = settleT }
  end
  res.step[#res.step + 1] = out
end

local function runTests()
  cur.testing = true
  local all = {}
  for i = 1, N do all[i] = i end
  for i = 1, N do
    doublet("E" .. i .. " GX", chan({ i }, 2, C.AMP))
    doublet("E" .. i .. " GY", chan({ i }, 3, C.AMP))
    doublet("E" .. i .. " THR", chan({ i }, 1, C.DTHR))
  end
  doublet("ALL GX", chan(all, 2, C.AMP_ALL))
  doublet("ALL GY", chan(all, 3, C.AMP_ALL))
  doublet("ALL THR", chan(all, 1, C.DTHR))
  if N == 4 then doublet("ROLL", rollChan(C.AMP_ROLL)) end
  stepTest("STEP X", 1, 0)
  stepTest("STEP Z", 0, 1)
  cur.testing = false
end

local function land()
  cur.testing = false
  cur.phase = "LAND"
  local calm = 0
  while true do
    local h = st.pos.y - start.y
    fly(nil, h > 10 and -C.DESC_V or -C.LAND_V)
    tick()
    if math.abs(st.vel.y) < 0.2 then calm = calm + 1 else calm = 0 end
    if (h < 3 and calm > 20) or calm > 60 then break end
  end
  cur.phase = "LANDED"
  cutAll()
end

local function flight()
  cur.rec = true
  t0 = os.clock()
  local h = { "t", "phase", "loop_ms" }
  for i = 1, N do h[#h + 1] = "thr" .. i .. ",gx" .. i .. ",gy" .. i end
  raw.writeLine(table.concat(h, ",") .. ",px,py,pz,vx,vy,vz,wx,wy,wz,tilt")
  io(false)
  start = st.pos
  ctl.psi0 = heading()
  cur.phase = "IGNITION"
  local thr = clamp(res.hoverEst * 0.75, 0.1, 0.8)
  local deadline = os.clock() + C.IGN_TIMEOUT
  setAll(thr, 0, 0, 0)
  tick()
  while st.vel.y <= C.LIFTOFF_VY do
    if os.clock() > deadline or cur.abort then
      cur.why = cur.abort and "KEY" or "IGNITION TIMEOUT"
      res.flightError = cur.why
      cutAll()
      return
    end
    thr = math.min(thr + C.THR_RAMP * cur.dt, C.MAX_THR)
    setAll(thr, 0, 0, 0)
    tick()
  end
  res.liftoff = thr
  ctl.hov = thr - 0.04
  cur.phase = "CLIMB"
  ctl.holdY = start.y + C.CLIMB
  local dl = os.clock() + C.CLIMB_TIMEOUT
  while st.pos.y < ctl.holdY - 1 and os.clock() < dl and not cur.abort do
    fly(nil, clamp((ctl.holdY - st.pos.y) * 0.5, 1, 5))
    tick()
  end
  if not cur.abort then
    cur.phase = "SETTLE"
    local te = os.clock() + C.SETTLE * 2
    while os.clock() < te and not cur.abort do
      fly(nil, holdV())
      tick()
    end
    res.hover = ctl.hov + ctl.hovI
    local ok, e = pcall(runTests)
    cur.testing = false
    if not ok then
      if e ~= STOP then error(e, 0) end
      res.stopped = cur.why or "ABORT"
    end
  end
  land()
end

local function num(x, f) return type(x) == "number" and string.format(f or "%.3f", x) or "--" end

local function report()
  local o = fs.open(C.OUT, "w")
  if not o then return end
  local function w(s) o.writeLine(s) end
  w("== VECTOR MAP ==")
  w("mass " .. num(res.mass, "%.2f") .. "  hover_est " .. num(res.hoverEst) .. "  liftoff " .. num(res.liftoff) .. "  hover " .. num(res.hover))
  w("skip " .. num(cur.skip, "%.2f") .. "  pulse " .. num(cur.pulse, "%.2f"))
  if res.stopped then w("TESTS STOPPED " .. res.stopped) end
  if res.padError then w("PAD ERROR " .. res.padError) end
  if res.flightError then w("FLIGHT " .. res.flightError) end
  local ok, it = pcall(textutils.serialize, res.inertia, { compact = true })
  w("inertia " .. (ok and it or "--"))
  w("== ENGINES ==")
  for i, n in ipairs(names) do w(string.format("E%d %s %s", i, n, C.WHERE[n] or "?")) end
  w("== GIMBAL LAG (pad, 0 -> " .. C.LAG_AMP .. ") ==")
  for _, ax in ipairs({ "x", "y" }) do
    local L = res.lag[ax]
    if L then
      for i = 1, N do
        local r = L[i]
        w(string.format("E%d %s t63 %s t90 %s from %s final %s", i, ax:upper(), num(r.t63, "%.2f"), num(r.t90, "%.2f"), num(r.from), num(r.final)))
      end
    end
  end
  w("== THRUST (pad, getThrust) ==")
  for _, c in ipairs(res.cal) do
    local p = {}
    for i = 1, N do
      local r = c.eng[i]
      p[i] = "E" .. i .. " " .. num(r.final, "%.1f") .. " t90 " .. num(r.t90, "%.2f")
    end
    w("thr " .. num(c.thr) .. ": " .. table.concat(p, "  "))
  end
  w("== RESPONSE PER UNIT COMMAND (body frame, y = nose axis) ==")
  w(string.format("%-8s %6s | %8s %8s %8s | %8s %8s %8s", "test", "amp", "ang_x", "ang_y", "ang_z", "acc_x", "acc_y", "acc_z"))
  for _, r in ipairs(res.tests) do
    w(string.format("%-8s %6.2f | %+8.3f %+8.3f %+8.3f | %+8.3f %+8.3f %+8.3f", r.label, r.eff,
      r.om.x, r.om.y, r.om.z, r.acc.x, r.acc.y, r.acc.z))
  end
  w("== ATTITUDE STEP " .. C.STEP_TILT .. " deg (rise to 80% s, max err after rise deg, settle within 20% s) ==")
  for _, s in ipairs(res.step) do
    for ph = 1, 2 do
      local p = s[ph]
      if p then
        w(string.format("%s %s rise %s max %s settle %s", s.label, ph == 1 and "OUT " or "BACK",
          num(p.rise, "%.2f"), num(p.over, "%.1f"), num(p.settle, "%.2f")))
      end
    end
  end
  o.close()
  local d = fs.open(C.DAT, "w")
  if d then
    local ok2, s = pcall(textutils.serialize, res)
    if ok2 then d.write(s) end
    d.close()
  end
end

local function run()
  local ok, m = pcall(function() return sublevel.getMass() end)
  res.mass = ok and type(m) == "number" and m or nil
  res.hoverEst = res.mass and clamp(C.GRAV * res.mass / C.THRUST_FORCE, 0.1, 0.9) or 0.34
  local ok2, it = pcall(function() return sublevel.getInertiaTensor() end)
  res.inertia = ok2 and it or nil
  res.names = names
  cur.skip, cur.pulse = C.SKIP, C.PULSE
  local okp, ep = pcall(padTests)
  if not okp then
    res.padError = tostring(ep)
    cutAll()
  end
  local mx = 0
  for _, L in pairs(res.lag) do
    for i = 1, N do
      if L[i] and L[i].t90 then mx = math.max(mx, L[i].t90) end
    end
  end
  if mx > 0 then
    cur.skip = clamp(mx + 0.05, 0.1, 0.35)
    cur.pulse = math.max(C.PULSE, cur.skip + 0.35)
  end
  for i = C.DELAY, 1, -1 do
    if cur.abort then break end
    cur.phase = "LAUNCH IN " .. i
    sleep(1)
  end
  if not cur.abort then
    local okf, ef = pcall(flight)
    if not okf then
      res.flightError = ef == CUT and ("CUT " .. (cur.why or "KEY")) or tostring(ef)
      cutAll()
    end
  end
  cutAll()
  report()
end

local function screen()
  while true do
    term.setBackgroundColor(colors.black)
    term.clear()
    term.setCursorPos(2, 2)
    term.setTextColor(colors.yellow)
    term.write("VECTOR MAP x" .. N)
    term.setTextColor(colors.white)
    term.setCursorPos(2, 4)
    term.write("PHASE " .. cur.phase)
    term.setCursorPos(2, 5)
    term.write(string.format("THR %.2f  ALT %.1f", cur.base, cur.alt))
    term.setCursorPos(2, 6)
    term.write(string.format("TILT %.1f  LOOP %d ms", cur.tilt, cur.loopMs))
    term.setCursorPos(2, 7)
    term.write("TESTS " .. #res.tests .. "  STEPS " .. #res.step)
    term.setCursorPos(2, 9)
    term.setTextColor(cur.abort and colors.red or colors.gray)
    term.write(cur.abort and "BKSP again: CUT ENGINES" or "BKSP: stop tests and land")
    raw.flush()
    sleep(0.2)
  end
end

local function keysLoop()
  while true do
    local _, k = os.pullEvent("key")
    if k == keys.backspace then
      if cur.abort then cur.cut = true else cur.abort = true end
    end
  end
end

local function fuel()
  while true do
    local fns = {}
    for i = 1, N do
      local e = eng[i]
      for _, t in ipairs(tanks) do
        fns[#fns + 1] = function() pcall(e.pullFluid, t) end
      end
    end
    if #fns > 0 then parallel.waitForAll(table.unpack(fns)) end
    sleep(0.5)
  end
end

cutAll()
local ok, err = pcall(parallel.waitForAny, run, screen, keysLoop, fuel)
cutAll()
raw.close()
term.setBackgroundColor(colors.black)
term.clear()
term.setCursorPos(2, 2)
term.setTextColor(ok and colors.lime or colors.red)
term.write(ok and ("Saved to " .. C.OUT) or tostring(err))
term.setCursorPos(1, 4)
