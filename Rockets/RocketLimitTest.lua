local C = {
  ENGINE_TYPE = "liquid_vector_thruster",
  HEIGHT = 150,
  HEIGHT_MIN = 100,
  HEIGHT_MAX = 200,
  DIRECTION = "north",
  COUNTDOWN = 5,
  GRAV = 10.5,
  THRUST_FORCE = 427,
  START_TILT = 15,

  RAMP = 0.2,
  MAX_LIFT_THR = 0.9,
  LIFTOFF_VY = 0.4,
  LIFTOFF_DH = 0.3,
  ID_SETTLE = 0.6,
  ID_AMP = 0.15,
  ID_T = 0.3,
  ID_GAP = 0.3,
  ID_MIN_DET = 0.05,
  ID_TAU = 0.23,

  KP = 2.0,
  KD = 2.2,
  KI = 0.4,
  I_WIN = 15,
  I_MAX = 2.5,
  ERR_MAX = 0.6,
  ALPHA_MAX = 2.0,
  GMAX = 0.8,
  CMD_RATE = 8,
  SETTLE_T = 3,

  CLIMB_V = 6,
  CLIMB_DEC = 1.5,
  KVC = 1.0,
  KIV = 0.3,
  KZ = 0.16,
  KVZ = 0.8,
  KIZ = 0.02,
  IY_MAX = 4,
  AY_MAX = 5,
  KC = 0.16,
  KCV = 0.8,
  AL_MAX = 4,
  HOLD_T = 5,
  HOLD_MAX = 30,
  HOLD_ERR = 5,
  HOLD_SPIN = 0.3,

  ACC_MIN = 1.5,
  ACC_K = 0.015,
  ACC_MAX = 8,
  V_LEAD = 15,
  KSP = 0.5,
  KIF = 0.08,
  THR_MAX = 1,

  SPEED_LIMIT = 1000,
  MAX_CROSS = 40,
  MAX_ALT = 40,
  BAD_T = 1,
  ERR_BAD = 25,
  ERR_HARD = 45,
  SPIN_BAD = 1.5,
  SPIN_HARD = 3,
  SPIN_T = 0.5,
  GSAT_T = 2,
  TOP_WIN = 10,
  TOP_GAIN = 3,
  MAX_T = 900,

  CLEAN_ERR = 10,
  CLEAN_SPIN = 0.3,
  CLEAN_CROSS = 10,
  CLEAN_ALT = 10,

  BRAKE_TILT = 50,
  KBR = 0.5,
  AOA_REC = 25,
  AOA_GROW = 1,
  FLIP_HI = 150,
  FLIP_LO = 40,
  DESC_V = 3,
  STOP_VH = 2,
  LAND_T = 1,
  GROUND_TILT = 70,
  GROUND_T = 2,

  LOG = "rocketlimit.log",
  SUMMARY = "rocketlimit.txt",
  LOG_DT = 0.25,
  LOG_DT_LATE = 0.5,
  LOG_LATE = 120,
  RING_T = 4,
  MIN_FREE = 60000,
  FUEL_DT = 0.5,
}

local args = { ... }
if type(args[1]) == "table" then args = {} end
local atan2 = math.atan2 or math.atan
local unpack = table.unpack or unpack
local UP = { x = 0, y = 1, z = 0 }
local function v3(a) return { x = a.x or a[1], y = a.y or a[2], z = a.z or a[3] } end
local function add(a, b) return { x = a.x + b.x, y = a.y + b.y, z = a.z + b.z } end
local function sub(a, b) return { x = a.x - b.x, y = a.y - b.y, z = a.z - b.z } end
local function mul(a, s) return { x = a.x * s, y = a.y * s, z = a.z * s } end
local function dot(a, b) return a.x * b.x + a.y * b.y + a.z * b.z end
local function cross(a, b) return { x = a.y * b.z - a.z * b.y, y = a.z * b.x - a.x * b.z, z = a.x * b.y - a.y * b.x } end
local function len(a) return math.sqrt(dot(a, a)) end
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
local function now() return os.epoch("utc") / 1000 end
local function turnToward(a, b, maxStep)
  local c = clamp(dot(a, b), -1, 1)
  local ang = math.acos(c)
  if ang <= maxStep then return b end
  local axis = cross(a, b)
  if len(axis) < 1e-6 then axis = math.abs(a.x) < 0.9 and { x = 1, y = 0, z = 0 } or { x = 0, y = 0, z = 1 } end
  axis = unit(axis)
  local p = unit(cross(axis, a))
  return unit(add(mul(a, math.cos(maxStep)), mul(p, math.sin(maxStep))))
end
local function solve3(A, b)
  local m = {}
  for i = 1, 3 do m[i] = { A[i][1], A[i][2], A[i][3], b[i] } end
  for c = 1, 3 do
    local p = c
    for r = c + 1, 3 do if math.abs(m[r][c]) > math.abs(m[p][c]) then p = r end end
    m[c], m[p] = m[p], m[c]
    if math.abs(m[c][c]) < 1e-12 then return nil end
    for r = 1, 3 do
      if r ~= c then
        local f = m[r][c] / m[c][c]
        for k = c, 4 do m[r][k] = m[r][k] - f * m[c][k] end
      end
    end
  end
  return { m[1][4] / m[1][1], m[2][4] / m[2][2], m[3][4] / m[3][3] }
end

local function parseDir(s)
  s = tostring(s or C.DIRECTION):lower()
  local named = { north = 180, n = 180, south = 0, s = 0, east = -90, e = -90, west = 90, w = 90 }
  local yaw = named[s] or tonumber(s)
  if not yaw then return nil end
  local r = math.rad(yaw)
  return { x = -math.sin(r), y = 0, z = math.cos(r) }, yaw
end
local function dirName(f)
  local yaw = math.deg(atan2(-f.x, f.z))
  local names = { "south", "south-west", "west", "north-west", "north", "north-east", "east", "south-east" }
  return names[(math.floor((yaw % 360) / 45 + 0.5) % 8) + 1], yaw
end

local height = tonumber(args[1]) or C.HEIGHT
local fwd, yawIn = parseDir(args[2])
if not fwd or height < C.HEIGHT_MIN or height > C.HEIGHT_MAX then
  print("Usage: RocketLimitTest [height " .. C.HEIGHT_MIN .. "-" .. C.HEIGHT_MAX .. "] [north|south|east|west|F3 yaw]")
  print("Example: RocketLimitTest 150 east")
  return
end
local rgt = { x = -fwd.z, y = 0, z = fwd.x }

local engName
for _, n in ipairs(peripheral.getNames()) do
  if peripheral.hasType(n, C.ENGINE_TYPE) and (not engName or n == "bottom") then engName = n end
end
if not engName then print("No liquid vector thruster found.") return end
if not sublevel then print("Not on a Sable sub-level (no sublevel API).") return end
local eng = peripheral.wrap(engName)
local tanks = {}
for _, n in ipairs(peripheral.getNames()) do
  if n ~= engName and peripheral.hasType(n, "fluid_storage") and not peripheral.hasType(n, C.ENGINE_TYPE) then tanks[#tanks + 1] = n end
end

local S = { thr = 0, gx = 0, gy = 0, eT = 0, eX = 0, eY = 0, nextFuel = 0, fuel = nil, abort = false }
local pos, w, u, vel, om, agx, agy, loopMs

local function tick(send)
  local p, v, o
  local fns = {
    function() p = sublevel.getLogicalPose() end,
    function() v = sublevel.getLinearVelocity() end,
    function() o = sublevel.getAngularVelocity() end,
  }
  if send ~= false then
    fns[#fns + 1] = function() eng.setThrustNormalized(S.thr) end
    fns[#fns + 1] = function() eng.setVector(S.gx, S.gy) end
  end
  agx, agy = nil, nil
  if eng.getVectorX then fns[#fns + 1] = function() local ok, r = pcall(eng.getVectorX) if ok and type(r) == "number" then agx = r end end end
  if eng.getVectorY then fns[#fns + 1] = function() local ok, r = pcall(eng.getVectorY) if ok and type(r) == "number" then agy = r end end end
  local t = now()
  if t >= S.nextFuel then
    S.nextFuel = t + C.FUEL_DT
    fns[#fns + 1] = function()
      local ok, c = pcall(peripheral.call, engName, "tanks")
      if ok and type(c) == "table" then
        local a = 0
        for _, f in pairs(c) do a = a + (type(f) == "table" and f.amount or 0) end
        S.fuel = a
      end
    end
    for _, n in ipairs(tanks) do
      local tn = n
      fns[#fns + 1] = function() pcall(eng.pullFluid, tn) end
    end
  end
  parallel.waitForAll(unpack(fns))
  loopMs = (now() - t) * 1000
  w, u = quat(p.orientation)
  pos, vel, om = v3(p.position), v3(v), v3(o)
end

local function dither(x, key)
  local v = x * 15 + S[key]
  local q = math.floor(v + 0.5)
  S[key] = clamp(v - q, -1, 1)
  return q / 15
end

local R = { reason = nil, stage = "PAD", bands = {}, peak = 0, events = {} }
local M, thrRef, Mb
local attI = { x = 0, z = 0 }

local function steer(cmd, thrAvg, dt)
  local d = unit(toBody(w, u, cmd))
  local s = math.sqrt(d.x * d.x + d.z * d.z)
  local ex, ez = 0, 0
  if s > 1e-9 then
    local a = math.min(atan2(s, d.y), C.ERR_MAX) / s
    ex, ez = d.z * a, -d.x * a
  elseif d.y < 0 then
    ex = C.ERR_MAX
  end
  if math.sqrt(ex * ex + ez * ez) < math.rad(C.I_WIN) then
    attI.x = clamp(attI.x + ex * dt, -C.I_MAX / math.max(C.KI, 1e-6), C.I_MAX / math.max(C.KI, 1e-6))
    attI.z = clamp(attI.z + ez * dt, -C.I_MAX / math.max(C.KI, 1e-6), C.I_MAX / math.max(C.KI, 1e-6))
  end
  local ax = clamp(C.KP * ex + C.KI * attI.x - C.KD * om.x - Mb[1], -C.ALPHA_MAX, C.ALPHA_MAX)
  local az = clamp(C.KP * ez + C.KI * attI.z - C.KD * om.z - Mb[2], -C.ALPHA_MAX, C.ALPHA_MAX)
  local k = math.max(thrAvg, 1 / 15) / thrRef
  local det = (M[1][1] * M[2][2] - M[1][2] * M[2][1]) * k
  local gx = (M[2][2] * ax - M[1][2] * az) / det
  local gy = (-M[2][1] * ax + M[1][1] * az) / det
  local gm = math.max(math.abs(gx), math.abs(gy))
  local sat = gm > C.GMAX
  if sat then gx, gy = gx * C.GMAX / gm, gy * C.GMAX / gm end
  return clamp(dither(gx, "eX"), -1, 1), clamp(dither(gy, "eY"), -1, 1), sat
end

local log, ring
local function note(s)
  R.events[#R.events + 1] = s
  if log then log.writeLine("EVENT " .. s) log.flush() end
  print(s)
end
local function freeOk()
  if not fs.getFreeSpace then return true end
  local ok, f = pcall(fs.getFreeSpace, "/")
  return not ok or f > C.MIN_FREE
end

local BAND_EDGES = {}
for v = 0, 100, 10 do BAND_EDGES[#BAND_EDGES + 1] = v end
for v = 125, 1000, 25 do BAND_EDGES[#BAND_EDGES + 1] = v end
local function bandOf(v)
  local i = 1
  while BAND_EDGES[i + 1] and v >= BAND_EDGES[i + 1] do i = i + 1 end
  return i
end
local function record(v, err, spin, crossT, altE, aoa, gm, sat)
  local i = bandOf(math.max(v, 0))
  local b = R.bands[i]
  if not b then
    b = { n = 0, err = 0, spin = 0, cross = 0, alt = 0, aoa = 0, gsum = 0, gmax = 0, sat = 0, t0 = R.runT }
    R.bands[i] = b
  end
  b.n = b.n + 1
  b.err = math.max(b.err, err)
  b.spin = math.max(b.spin, spin)
  b.cross = math.max(b.cross, math.abs(crossT))
  b.alt = math.max(b.alt, math.abs(altE))
  b.aoa = math.max(b.aoa, aoa)
  b.gsum = b.gsum + gm
  b.gmax = math.max(b.gmax, gm)
  if sat then b.sat = b.sat + 1 end
  b.t1 = R.runT
end

local function save(status)
  local out = fs.open(C.SUMMARY, "w")
  if not out then return end
  local L = function(s) out.writeLine(s) end
  L("SINGLE THRUSTER LIMIT TEST: " .. status)
  L(string.format("MASS %.2f  THRUST %.1f m/s2 per full throttle  HOVER %.3f", R.mass or 0, R.tacc or 0, R.hover or 0))
  if M then L(string.format("GIMBAL MAP at throttle %.3f: ang_x = %+.2f*gx %+.2f*gy  ang_z = %+.2f*gx %+.2f*gy  (bias %+.2f %+.2f)", thrRef, M[1][1], M[1][2], M[2][1], M[2][2], Mb[1], Mb[2])) end
  if R.slew then L(string.format("GIMBAL measured max move %.3f per tick (%.2f per s)", R.slew, R.slew * 20)) end
  L(string.format("DIRECTION %s (yaw %.0f)  HEIGHT +%d  START %.1f %.1f %.1f", R.dirName or "?", R.yaw or 0, height, R.x0 or 0, R.y0 or 0, R.z0 or 0))
  L("STOP: " .. (R.reason or "not stopped yet"))
  if R.stopV then L(string.format("SPEED AT STOP %.1f m/s at %.1f s of run, %.0f blocks down track", R.stopV, R.stopT, R.stopD or 0)) end
  L(string.format("PEAK SPEED %.1f m/s", R.peak))
  L(string.format("FASTEST CLEAN SPEED %s (every band up to it: error < %d deg, spin < %.1f, off track < %d, altitude < %d)",
    R.clean and string.format("%.0f m/s", R.clean) or "none", C.CLEAN_ERR, C.CLEAN_SPIN, C.CLEAN_CROSS, C.CLEAN_ALT))
  if R.endPos then L(string.format("ENDED AT %.0f %.0f %.0f", R.endPos.x, R.endPos.y, R.endPos.z)) end
  L("")
  L("speed_band   time_s  max_err  max_spin  max_off  max_alt  max_aoa  gimbal_avg  gimbal_max  sat%")
  for i = 1, #BAND_EDGES do
    local b = R.bands[i]
    if b then
      L(string.format("%4d-%-4d %8.1f %8.1f %9.2f %8.1f %8.1f %8.1f %11.3f %11.3f %5.0f", BAND_EDGES[i], BAND_EDGES[i + 1] or 9999,
        (b.t1 or 0) - (b.t0 or 0), b.err, b.spin, b.cross, b.alt, b.aoa, b.gsum / b.n, b.gmax, 100 * b.sat / b.n))
    end
  end
  L("")
  for _, e in ipairs(R.events) do L(e) end
  out.close()
end
local function updateClean()
  local best
  for i = 1, #BAND_EDGES do
    local b = R.bands[i]
    if not b then break end
    if b.err > C.CLEAN_ERR or b.spin > C.CLEAN_SPIN or b.cross > C.CLEAN_CROSS or b.alt > C.CLEAN_ALT then break end
    best = BAND_EDGES[i + 1] or BAND_EDGES[i]
  end
  R.clean = best and math.min(best, R.peak) or nil
end

local function stop(why, along, t)
  if R.reason then return end
  R.reason, R.stopV, R.stopT = why, along, R.runT or 0
  R.stopD = R.startPos and dot(sub(pos, R.startPos), fwd) or 0
  R.stage = "RECOVER"
  updateClean()
  note(string.format("TEST END: %s at %.1f m/s (t %.1f)", why, along, t))
  if ring then
    for _, r in ipairs(ring) do log.writeLine("FAST " .. r.s) end
    ring = nil
  end
  save("ENDED, RECOVERING")
end

local function run()
  R.mass = sublevel.getMass()
  local it = sublevel.getInertiaTensor()
  local ix, iy, iz = it[1][1] or it[1].x, it[2][2] or it[2].y, it[3][3] or it[3].z
  if not R.mass or R.mass <= 0 or not ix then error("Could not read mass/inertia", 0) end
  if not (iy < ix and iy < iz) then error("Rocket is not assembled upright (body Y must be the long axis). Re-assemble standing up.", 0) end
  R.tacc = C.THRUST_FORCE / R.mass
  R.hover = C.GRAV / R.tacc
  tick(false)
  local nose = rotate(w, u, UP)
  if nose.y < math.cos(math.rad(C.START_TILT)) then error("Stand the rocket upright first", 0) end
  R.dirName, R.yaw = dirName(fwd)
  R.x0, R.y0, R.z0 = pos.x, pos.y, pos.z
  local start = { x = pos.x, y = pos.y, z = pos.z }
  local yT = pos.y + height
  if fs.exists(C.LOG) then pcall(fs.delete, C.LOG .. ".old") pcall(fs.move, C.LOG, C.LOG .. ".old") end
  if fs.exists(C.SUMMARY) then pcall(fs.delete, C.SUMMARY .. ".old") pcall(fs.move, C.SUMMARY, C.SUMMARY .. ".old") end
  log = fs.open(C.LOG, "w")
  note(string.format("START mass %.2f, thrust %.1f m/s2, hover throttle %.3f, %d fuel tank(s), heading %s, height +%d", R.mass, R.tacc, R.hover, #tanks, R.dirName, height))
  print("Backspace or Ctrl+T: end the test and land.")
  for i = C.COUNTDOWN, 1, -1 do
    if S.abort then R.reason = "CANCELLED ON PAD" save("CANCELLED") return end
    print("Launch in " .. i)
    sleep(1)
  end
  log.writeLine("t stage steps thr vT along vy cross alt_err x y z err aoa_p aoa_y wx wz gx gy agx agy spin sat ms fuel")
  local t0, tl = now(), now()
  local stage, st = "LIFT", 0
  local thrAvg, samples, prevOm, prevT = 0, {}, nil, nil
  local gl = { x = 0, y = 0 }
  local cmd = UP
  local Iy, If, vT = 0, 0, 0
  local bad = { cross = 0, alt = 0, err = 0, spin = 0, sat = 0, ground = 0, land = 0 }
  local hist, nextLog = {}, 0
  local holdT, descT = 0, 0
  local altHold
  while true do
    S.thr = thrAvg > 0 and clamp(dither(thrAvg, "eT"), stage == "LIFT" and 0 or 1 / 15, 1) or 0
    tick(true)
    local tn = now()
    local dt = clamp(tn - tl, 0.01, 0.3)
    tl = tn
    local t = tn - t0
    st = st + dt
    nose = rotate(w, u, UP)
    local rel = sub(pos, start)
    local along, sideV = dot(vel, fwd), dot(vel, rgt)
    local alongP, crossT = dot(rel, fwd), dot(rel, rgt)
    local altE = pos.y - yT
    local vh = math.sqrt(vel.x * vel.x + vel.z * vel.z)
    local err = math.deg(math.acos(clamp(dot(nose, cmd), -1, 1)))
    local spin = math.sqrt(om.x * om.x + om.z * om.z)
    local vb = toBody(w, u, vel)
    local sp = len(vel)
    local aoaP = sp > 3 and math.deg(atan2(vb.z, vb.y)) or 0
    local aoaY = sp > 3 and math.deg(atan2(vb.x, vb.y)) or 0
    local aoa = sp > 3 and math.deg(math.acos(clamp(vb.y / sp, -1, 1))) or 0
    local sat = false
    local tiltNow = math.deg(math.acos(clamp(nose.y, -1, 1)))
    if stage ~= "LIFT" and stage ~= "ID" and tiltNow > C.GROUND_TILT and sp < 3 and math.abs(vel.y) < 1 and pos.y < yT - 20 then
      bad.ground = bad.ground + dt
    else
      bad.ground = 0
    end
    if bad.ground > C.GROUND_T then
      S.thr, S.gx, S.gy = 0, 0, 0
      tick(true)
      R.endPos = pos
      if not R.reason then R.reason = "FELL OVER NEAR THE GROUND" end
      note("ENGINE CUT: lying on the ground")
      save("FELL OVER, ENGINE CUT")
      return
    end
    if stage == "LIFT" then
      thrAvg = math.min((thrAvg > 0 and thrAvg or 0) + C.RAMP * dt, C.MAX_LIFT_THR)
      S.gx, S.gy = 0, 0
      if vel.y > C.LIFTOFF_VY or rel.y > C.LIFTOFF_DH then
        note(string.format("LIFTOFF at throttle %.3f, %.1f s", thrAvg, t))
        thrAvg = R.hover
        stage, st = "ID", 0
      elseif thrAvg >= C.MAX_LIFT_THR and st > 15 then
        S.thr, S.gx, S.gy = 0, 0, 0
        tick(true)
        R.reason = "NO LIFTOFF (still docked or too heavy?)"
        note(R.reason)
        save("NO LIFTOFF")
        return
      end
    elseif stage == "ID" then
      thrAvg = R.hover * (1 + clamp(C.KVC * (0.5 - vel.y), -3, 3) / C.GRAV)
      local a = math.min(1, dt / C.ID_TAU)
      gl.x = gl.x + (S.gx - gl.x) * a
      gl.y = gl.y + (S.gy - gl.y) * a
      local gxm, gym = agx or gl.x, agy or gl.y
      if prevOm and st > C.ID_SETTLE - 0.2 then
        samples[#samples + 1] = { gxm, gym, (om.x - prevOm.x) / dt, (om.z - prevOm.z) / dt }
        if agx and #samples > 1 then
          local pr = samples[#samples - 1]
          R.slew = math.max(R.slew or 0, math.abs(gxm - pr[1]), math.abs(gym - pr[2]))
        end
      end
      local s1 = C.ID_SETTLE
      local s2, s3 = s1 + C.ID_T, s1 + 2 * C.ID_T
      local s4 = s3 + C.ID_GAP
      local s5, s6 = s4 + C.ID_T, s4 + 2 * C.ID_T
      local s7 = s6 + C.ID_GAP
      local gx, gy = 0, 0
      if st >= s1 and st < s2 then gx = C.ID_AMP
      elseif st >= s2 and st < s3 then gx = -C.ID_AMP
      elseif st >= s4 and st < s5 then gy = C.ID_AMP
      elseif st >= s5 and st < s6 then gy = -C.ID_AMP end
      S.gx, S.gy = gx, gy
      if st >= s7 then
        local A, bx, bz = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } }, { 0, 0, 0 }, { 0, 0, 0 }
        for _, sm in ipairs(samples) do
          local r = { sm[1], sm[2], 1 }
          for i = 1, 3 do
            for j = 1, 3 do A[i][j] = A[i][j] + r[i] * r[j] end
            bx[i] = bx[i] + r[i] * sm[3]
            bz[i] = bz[i] + r[i] * sm[4]
          end
        end
        local cx, cz = solve3(A, bx), solve3(A, bz)
        local det = cx and cz and (cx[1] * cz[2] - cx[2] * cz[1]) or 0
        if math.abs(det) < C.ID_MIN_DET then
          S.thr, S.gx, S.gy = 0, 0, 0
          tick(true)
          R.reason = "GIMBAL MAP FAILED (gimbal had no clear effect)"
          note(R.reason)
          save("MAP FAILED, ENGINE CUT NEAR GROUND")
          return
        end
        M = { { cx[1], cx[2] }, { cz[1], cz[2] } }
        Mb = { clamp(cx[3], -0.5, 0.5), clamp(cz[3], -0.5, 0.5) }
        thrRef = R.hover
        note(string.format("GIMBAL MAP ang_x = %+.2f*gx %+.2f*gy   ang_z = %+.2f*gx %+.2f*gy   (actual gimbal %s)", M[1][1], M[1][2], M[2][1], M[2][2], agx and "read" or "modelled"))
        save("CLIMBING")
        stage, st = "CLIMB", 0
        attI = { x = 0, z = 0 }
        S.gx, S.gy = 0, 0
      end
      prevOm = om
    end
    if stage == "CLIMB" or stage == "HOLD" or stage == "RUN" or stage == "RECOVER" or stage == "DESCEND" then
      if R.reason and stage ~= "RECOVER" and stage ~= "DESCEND" then stage, st = "RECOVER", 0 altHold = math.max(pos.y, yT - C.MAX_ALT) end
      local aY, aF, aL
      local hv = { x = vel.x, y = 0, z = vel.z }
      if stage == "CLIMB" then
        local vyT = math.min(C.CLIMB_V, math.sqrt(2 * C.CLIMB_DEC * math.max(-altE, 0)))
        Iy = clamp(Iy + C.KIV * (vyT - vel.y) * dt, -C.IY_MAX, C.IY_MAX)
        aY = C.GRAV + clamp(C.KVC * (vyT - vel.y), -C.AY_MAX, C.AY_MAX) + Iy
        aF = clamp(-C.KC * alongP - C.KCV * along, -C.AL_MAX, C.AL_MAX)
        aL = clamp(-C.KC * crossT - C.KCV * sideV, -C.AL_MAX, C.AL_MAX)
        if altE > -2 and math.abs(vel.y) < 1.5 then stage, st = "HOLD", 0 note(string.format("AT HEIGHT, holding (t %.1f)", t)) end
      elseif stage == "RECOVER" or stage == "DESCEND" then
        if stage == "RECOVER" then
          Iy = clamp(Iy - C.KIZ * (pos.y - altHold) * dt, -C.IY_MAX, C.IY_MAX)
          aY = C.GRAV + clamp(-C.KZ * (pos.y - altHold) - C.KVZ * vel.y, -C.AY_MAX, C.AY_MAX) + Iy
          local hdir = vh > 0.1 and mul(hv, 1 / vh) or fwd
          local f = clamp((C.FLIP_HI - vh) / (C.FLIP_HI - C.FLIP_LO), 0, 1)
          local lim = math.min(C.AOA_REC + (90 + C.BRAKE_TILT - C.AOA_REC) * f + C.AOA_GROW * st, 90 + C.BRAKE_TILT)
          local aH = -math.min(C.KBR * vh, aY * math.tan(math.rad(C.BRAKE_TILT)))
          if lim < 89.5 then aH = math.max(aH, aY / math.tan(math.rad(lim)))
          elseif lim > 90.5 then aH = math.max(aH, -aY * math.tan(math.rad(lim - 90))) end
          aF, aL = aH * dot(hdir, fwd), aH * dot(hdir, rgt)
          if vh < C.STOP_VH then stage, st = "DESCEND", 0 note(string.format("STOPPED at %.0f %.0f %.0f, descending", pos.x, pos.y, pos.z)) end
        else
          aY = C.GRAV + clamp(C.KVC * (-C.DESC_V - vel.y), -C.AY_MAX, C.AY_MAX) + Iy
          aF, aL = clamp(-C.KCV * along, -C.AL_MAX, C.AL_MAX), clamp(-C.KCV * sideV, -C.AL_MAX, C.AL_MAX)
          descT = descT + dt
          if descT > 3 and math.abs(vel.y) < 0.4 and vh < 1 then bad.land = bad.land + dt else bad.land = 0 end
          if bad.land > C.LAND_T then
            S.thr, S.gx, S.gy = 0, 0, 0
            tick(true)
            R.endPos = pos
            note(string.format("LANDED at %.0f %.0f %.0f, tilt %.0f deg", pos.x, pos.y, pos.z, tiltNow))
            save("LANDED")
            return
          end
        end
      else
        Iy = clamp(Iy - C.KIZ * altE * dt, -C.IY_MAX, C.IY_MAX)
        aY = C.GRAV + clamp(-C.KZ * altE - C.KVZ * vel.y, -C.AY_MAX, C.AY_MAX) + Iy
        aL = clamp(-C.KC * crossT - C.KCV * sideV, -C.AL_MAX, C.AL_MAX)
        if stage == "HOLD" then
          aF = clamp(-C.KC * alongP - C.KCV * along, -C.AL_MAX, C.AL_MAX)
          if err < C.HOLD_ERR and spin < C.HOLD_SPIN and math.abs(altE) < 5 then holdT = holdT + dt else holdT = 0 end
          if holdT >= C.HOLD_T then
            stage, st = "RUN", 0
            R.runT, R.startPos, vT = 0, { x = pos.x, y = pos.y, z = pos.z }, math.max(along, 0)
            ring = {}
            note(string.format("RUN: accelerating %s (t %.1f)", R.dirName, t))
            save("RUNNING")
          elseif st > C.HOLD_MAX then
            stop("COULD NOT HOVER STEADY", along, t)
          end
        else
          R.runT = R.runT + dt
          local acc = clamp(C.ACC_K * math.max(along, 0), C.ACC_MIN, C.ACC_MAX)
          vT = math.min(vT + acc * dt, math.max(along, 0) + C.V_LEAD, C.SPEED_LIMIT + 20)
          If = clamp(If + C.KIF * (vT - along) * dt, -5, R.tacc)
          aF = acc + C.KSP * (vT - along) + If
        end
      end
      local lim2 = (R.tacc * C.THR_MAX) ^ 2 - aY * aY - aL * aL
      local aFmax = math.sqrt(math.max(lim2, 0))
      local thrustSat = aF > aFmax
      if thrustSat then aF = aFmax end
      local W = add(add(mul(fwd, aF), mul(rgt, aL)), mul(UP, aY))
      local want = unit(W)
      cmd = turnToward(cmd, want, math.rad(C.CMD_RATE) * dt)
      thrAvg = clamp(len(W) / R.tacc / math.max(dot(nose, want), 0.5), 1 / 15, 1)
      S.gx, S.gy, sat = steer(cmd, thrAvg, dt)
      if stage == "RUN" and not R.reason then
        R.peak = math.max(R.peak, along)
        local gm = math.max(math.abs(S.gx), math.abs(S.gy))
        if R.runT > C.SETTLE_T then record(along, err, spin, crossT, altE, aoa, gm, sat) end
        bad.cross = math.abs(crossT) > C.MAX_CROSS and bad.cross + dt or 0
        bad.alt = math.abs(altE) > C.MAX_ALT and bad.alt + dt or 0
        bad.err = err > C.ERR_BAD and bad.err + dt or 0
        bad.spin = spin > C.SPIN_BAD and bad.spin + dt or 0
        bad.sat = sat and bad.sat + dt or 0
        hist[#hist + 1] = { t = R.runT, v = along, s = thrustSat }
        while #hist > 2 and hist[1].t < R.runT - C.TOP_WIN do table.remove(hist, 1) end
        local allSat = true
        for _, hr in ipairs(hist) do if not hr.s then allSat = false break end end
        if along >= C.SPEED_LIMIT then stop("REACHED " .. C.SPEED_LIMIT .. " M/S", along, t)
        elseif bad.cross > C.BAD_T then stop(string.format("OFF TRACK (%.0f blocks sideways)", crossT), along, t)
        elseif bad.alt > C.BAD_T then stop(string.format("ALTITUDE LOST (%+.0f blocks)", altE), along, t)
        elseif err > C.ERR_HARD or bad.err > C.SPIN_T then stop(string.format("UNSTABLE: nose %.0f deg off", err), along, t)
        elseif spin > C.SPIN_HARD or bad.spin > C.SPIN_T then stop(string.format("UNSTABLE: spinning %.2f rad/s", spin), along, t)
        elseif bad.sat > C.GSAT_T then stop("UNSTABLE: steering maxed out for " .. C.GSAT_T .. " s", along, t)
        elseif allSat and R.runT - hist[1].t > C.TOP_WIN - 0.2 and along - hist[1].v < C.TOP_GAIN then stop(string.format("TOP SPEED (full throttle, gaining < %d m/s in %d s)", C.TOP_GAIN, C.TOP_WIN), along, t)
        elseif S.abort then stop("STOPPED BY YOU", along, t)
        elseif R.runT > C.MAX_T then stop("TIME LIMIT", along, t) end
        if R.bandSaved ~= bandOf(math.max(along, 0)) then R.bandSaved = bandOf(math.max(along, 0)) updateClean() save("RUNNING") end
      elseif not R.reason and S.abort then
        stop("STOPPED BY YOU", along, t)
      elseif not R.reason and stage == "CLIMB" and st > 120 then
        stop("CLIMB TIMEOUT", along, t)
      end
    end
    local row = string.format("%.2f %s %d %.3f %.1f %.2f %+.2f %+.2f %+.2f %.1f %.1f %.1f %.1f %+.1f %+.1f %+.3f %+.3f %+.3f %+.3f %s %s %.2f %s %.0f %s",
      t, stage, math.floor(S.thr * 15 + 0.5), thrAvg, vT, along, vel.y, crossT, altE, pos.x, pos.y, pos.z, err, aoaP, aoaY, om.x, om.z, S.gx, S.gy,
      agx and string.format("%+.3f", agx) or "NA", agy and string.format("%+.3f", agy) or "NA", spin, sat and "S" or "-", loopMs, S.fuel and tostring(S.fuel) or "NA")
    if ring then
      ring[#ring + 1] = { t = t, s = row }
      while #ring > 1 and ring[1].t < t - C.RING_T do table.remove(ring, 1) end
    end
    local logDt = (stage == "RUN" and R.runT > C.LOG_LATE) and C.LOG_DT_LATE or C.LOG_DT
    if stage == "ID" or stage == "LIFT" then logDt = 0 end
    if log and tn >= nextLog then
      if freeOk() then
        log.writeLine(row)
        log.flush()
      end
      nextLog = tn + logDt
    end
  end
end

local function keysIn()
  while true do
    local ev, k = os.pullEventRaw()
    if ev == "terminate" or (ev == "key" and k == keys.backspace) then S.abort = true end
  end
end

local oldPull = os.pullEvent
os.pullEvent = os.pullEventRaw or os.pullEvent
local ok, e = pcall(parallel.waitForAny, run, keysIn)
os.pullEvent = oldPull
if not ok then
  R.reason = R.reason or ("ERROR " .. tostring(e))
  local high = pos and R.y0 and pos.y > R.y0 + 3
  S.thr = high and clamp(R.hover or 0.14, 1 / 15, 1) or 0
  S.gx, S.gy = 0, 0
  pcall(parallel.waitForAll, function() eng.setThrustNormalized(S.thr) end, function() eng.setVector(0, 0) end)
  pcall(save, "PROGRAM ERROR: " .. tostring(e))
  print("ERROR: " .. tostring(e))
  if high then print("Engine left at hover with the gimbal centred. Take over manually.") end
end
if log then pcall(log.close) end
print("Results: " .. C.SUMMARY .. " and " .. C.LOG)
