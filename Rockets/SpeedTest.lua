local args = { ... }
local tx, tz = tonumber(args[1]), tonumber(args[2])
local mode = string.upper(args[3] or "NET")
local climb = tonumber(args[4]) or 200
local OUT = "speedtest.log"
local ROW_DT = 0.25
local START_THR = 0.99
local END_THR = 0.9
local END_HOLD = 2.0
local LEVEL_VY = 10

if not tx or not tz or (mode ~= "NET" and mode ~= "ARC") then
  print("Usage: SpeedTest <x> <z> [NET|ARC] [climb]")
  print("Flies Rocket4 to x z and records the full-throttle cruise.")
  print("Pick a destination 20000+ blocks away. Default NET, climb 200.")
  return
end

local file
for _, n in ipairs({ "Rocket4.lua", "Rocket4" }) do
  local p = shell and shell.resolve(n) or n
  if fs.exists(p) then file = p break end
end
if not file then
  print("Rocket4.lua not found next to SpeedTest.")
  return
end
local env = setmetatable({}, { __index = _ENV or getfenv(1) })
local rocket, lerr = loadfile(file, nil, env)
if rocket and setfenv then pcall(setfenv, rocket, env) end
if not rocket then
  print("Could not load " .. file .. ": " .. tostring(lerr))
  return
end

local function v3(a) return { x = a.x or a[1], y = a.y or a[2], z = a.z or a[3] } end
local function dot(a, b) return a.x * b.x + a.y * b.y + a.z * b.z end
local function len(a) return math.sqrt(dot(a, a)) end
local function cross(a, b) return { x = a.y * b.z - a.z * b.y, y = a.z * b.x - a.x * b.z, z = a.x * b.y - a.y * b.x } end
local function clamp(x, lo, hi) return math.max(lo, math.min(hi, x)) end
local function quat(q)
  if q.w then return q.w, { x = q.x, y = q.y, z = q.z } end
  if q.a and q.v then return q.a, v3(q.v) end
  if q.a then return q.a, { x = q.b, y = q.c, z = q.d } end
  return q[4], { x = q[1], y = q[2], z = q[3] }
end
local function noseOf(q)
  local w, u = quat(q)
  local v = { x = 0, y = 1, z = 0 }
  local t = cross(u, v)
  t = { x = 2 * t.x, y = 2 * t.y, z = 2 * t.z }
  local c = cross(u, t)
  return { x = v.x + w * t.x + c.x, y = v.y + w * t.y + c.y, z = v.z + w * t.z + c.z }
end

local okPose, pose0 = pcall(sublevel.getLogicalPose)
if not okPose or not pose0 then
  print("No sub-level pose. Is the computer on the rocket?")
  return
end
local start = v3(pose0.position)

local S = {
  thr = {}, gv = {}, rows = {}, state = "wait", phase = "IGNITION", rec = 0,
  vmaxAll = 0, vmax = 0, tmax = 0, marks = {}, n = 0, altMin = math.huge, altMax = -math.huge,
  pitchSum = 0, pitch2 = 0, pitchMin = 90, pitchMax = -90, aoaSum = 0, aoaMax = 0,
  spinSum = 0, spinMax = 0, g2 = 0, sat = 0, thrSum = 0, vy2 = 0, ts = {}, vs = {}, nextRow = 0, lowT = 0,
}

local function avgThr()
  local s, k = 0, 0
  for _, v in pairs(S.thr) do s, k = s + v, k + 1 end
  return k > 0 and s / k or 0
end

local function gimbal()
  local gx, gy, k, sat = 0, 0, 0, false
  for _, g in pairs(S.gv) do
    gx, gy, k = gx + g[1], gy + g[2], k + 1
    if math.abs(g[1]) >= 0.99 or math.abs(g[2]) >= 0.99 then sat = true end
  end
  if k == 0 then return 0, 0, false end
  return gx / k, gy / k, sat
end

local function sample()
  if not (S.pose and S.vel and S.om) then return end
  local t = os.clock()
  local vel, om = v3(S.vel), v3(S.om)
  local v = len(vel)
  S.vmaxAll = math.max(S.vmaxAll, v)
  if t < S.nextRow then return end
  S.nextRow = t + ROW_DT
  local pos = v3(S.pose.position)
  local nose = noseOf(S.pose.orientation)
  local thr = avgThr()
  if S.state == "wait" then
    if pos.y >= start.y + climb - 5 and thr >= START_THR and math.abs(vel.y) < LEVEL_VY then
      S.state, S.t0, S.p0, S.fuel0 = "on", t, pos, S.fuel
    else
      return
    end
  end
  if S.state ~= "on" then return end
  if thr < END_THR then S.lowT = S.lowT + ROW_DT else S.lowT = 0 end
  if S.lowT >= END_HOLD then
    S.state, S.ended, S.fuel1 = "done", S.phase .. " (throttle dropped)", S.fuel
    return
  end
  local rt = t - S.t0
  S.tl, S.pl, S.fuel1 = t, pos, S.fuel
  if v > S.vmax then S.vmax, S.tmax = v, rt end
  for _, m in ipairs({ 100, 150, 200, 240, 280 }) do
    if v >= m and not S.marks[m] then S.marks[m] = rt end
  end
  local pitch = math.deg(math.asin(clamp(nose.y, -1, 1)))
  local aoa = v > 1 and math.deg(math.acos(clamp(dot(nose, vel) / v, -1, 1))) or 0
  local spin = len(om)
  local gx, gy, sat = gimbal()
  S.n = S.n + 1
  S.altMin, S.altMax = math.min(S.altMin, pos.y), math.max(S.altMax, pos.y)
  S.pitchSum, S.pitch2 = S.pitchSum + pitch, S.pitch2 + pitch * pitch
  S.pitchMin, S.pitchMax = math.min(S.pitchMin, pitch), math.max(S.pitchMax, pitch)
  S.aoaSum, S.aoaMax = S.aoaSum + aoa, math.max(S.aoaMax, aoa)
  S.spinSum, S.spinMax = S.spinSum + spin, math.max(S.spinMax, spin)
  S.g2 = S.g2 + gx * gx + gy * gy
  if sat then S.sat = S.sat + 1 end
  S.thrSum, S.vy2 = S.thrSum + thr, S.vy2 + vel.y * vel.y
  S.ts[#S.ts + 1], S.vs[#S.vs + 1] = rt, v
  S.rows[#S.rows + 1] = string.format("%.2f %s %.1f %.1f %+.1f %+.1f %.1f %.2f %+.2f %+.2f %.2f",
    rt, S.phase, v, pos.y, vel.y, pitch, aoa, spin, gx, gy, thr)
end

local realSub = sublevel
local realWrap = peripheral.wrap
env.sublevel = setmetatable({
  getLogicalPose = function() local p = realSub.getLogicalPose() S.pose = p return p end,
  getLinearVelocity = function() local v = realSub.getLinearVelocity() S.vel = v return v end,
  getAngularVelocity = function() local o = realSub.getAngularVelocity() S.om = o pcall(sample) return o end,
}, { __index = realSub })
peripheral.wrap = function(n)
  local p = realWrap(n)
  if p and peripheral.hasType(n, "liquid_vector_thruster") then
    return setmetatable({
      setThrustNormalized = function(v) S.thr[n] = v return p.setThrustNormalized(v) end,
      setVector = function(x, y) S.gv[n] = { x, y } return p.setVector(x, y) end,
    }, { __index = p })
  end
  return p
end

local REMOTE = {
  mission = { mode = mode, x = tx, y = math.floor(start.y + 0.5), z = tz, climb = climb },
  state = {},
  launchIn = function() return 0 end,
  plan = function() end,
  report = function(r)
    if r.phase and r.phase ~= S.phase then
      if r.phase == "RECOVER" and S.state == "on" then S.rec = S.rec + 1 end
      S.phase = r.phase
    end
    if r.tank_mb or r.engine_mb then S.fuel = (r.tank_mb or 0) + (r.engine_mb or 0) end
    return false
  end,
  result = function(r, sp, t) S.result = { r = r, sp = sp, t = t } end,
}

term.clear()
term.setCursorPos(1, 1)
print(string.format("SPEED TEST %s to %d %d, climb %d", mode, tx, tz, climb))
print(string.format("Distance %.0f blocks. Launch in 5 s.", math.sqrt((tx - start.x) ^ 2 + (tz - start.z) ^ 2)))
print("Backspace to cancel.")
for i = 5, 1, -1 do
  local timer = os.startTimer(1)
  while true do
    local e, a = os.pullEvent()
    if e == "key" and a == keys.backspace then
      peripheral.wrap = realWrap
      print("Cancelled.")
      return
    end
    if e == "timer" and a == timer then break end
  end
end

local ok, err = pcall(rocket, REMOTE)
peripheral.wrap = realWrap
if S.state == "on" then S.state, S.ended, S.fuel1 = "done", "flight ended" end

local lines = {}
local function out(fmt, ...) lines[#lines + 1] = string.format(fmt, ...) end
local function mark(v) return S.marks[v] and string.format("%.1fs", S.marks[v]) or "--" end
out("SPEED TEST %s  TGT %d %d  CLIMB %d", mode, tx, tz, climb)
if S.n == 0 then
  out("NO FULL-THROTTLE CRUISE RECORDED (phase %s)", S.phase)
else
  local dur = S.tl - S.t0
  local sum, k = 0, 0
  for i = 1, #S.ts do
    if S.ts[i] >= S.ts[#S.ts] - 10 then sum, k = sum + S.vs[i], k + 1 end
  end
  local pm = S.pitchSum / S.n
  local used = S.fuel0 and S.fuel1 and (S.fuel0 - S.fuel1) or nil
  out("CRUISE        %.1f s, %.0f blocks, ended by %s", dur, math.sqrt((S.pl.x - S.p0.x) ^ 2 + (S.pl.z - S.p0.z) ^ 2), S.ended or "?")
  out("MAX SPEED     %.1f m/s at %.1f s (whole flight %.1f)", S.vmax, S.tmax, S.vmaxAll)
  out("LAST 10 S AVG %.1f m/s", k > 0 and sum / k or 0)
  out("TIME TO       100 %s  150 %s  200 %s  240 %s  280 %s", mark(100), mark(150), mark(200), mark(240), mark(280))
  out("ALTITUDE      %.0f to %.0f (swing %.0f), vy RMS %.1f m/s", S.altMin, S.altMax, S.altMax - S.altMin, math.sqrt(S.vy2 / S.n))
  out("PITCH         %.1f to %.1f deg, avg %.1f, wobble (std) %.1f", S.pitchMin, S.pitchMax, pm, math.sqrt(math.max(S.pitch2 / S.n - pm * pm, 0)))
  out("NOSE VS PATH  avg %.1f deg, max %.1f deg", S.aoaSum / S.n, S.aoaMax)
  out("SPIN          avg %.2f, max %.2f rad/s", S.spinSum / S.n, S.spinMax)
  out("GIMBAL        RMS %.2f, at limit %.0f%% of time", math.sqrt(S.g2 / S.n), 100 * S.sat / S.n)
  out("THROTTLE      avg %.2f", S.thrSum / S.n)
  out("RECOVERIES    %d", S.rec)
  out(used and string.format("FUEL          %.0f mB, %.1f mB/s", used, dur > 0 and used / dur or 0) or "FUEL          --")
end
if S.result and S.result.r then
  out("LANDING       miss %.1f, speed %.0f m/s, time %.1f s", S.result.r.total or 0, S.result.sp or 0, S.result.t or 0)
end
if not ok then out("ROCKET4 ERROR %s", tostring(err)) end

local f = fs.open(OUT, "w")
if f then
  for _, l in ipairs(lines) do f.writeLine(l) end
  f.writeLine("")
  f.writeLine("t phase speed alt vy pitch nose_vs_path spin gx gy thr")
  for _, r in ipairs(S.rows) do f.writeLine(r) end
  f.close()
end
term.setBackgroundColor(colors.black)
term.setTextColor(colors.white)
term.clear()
term.setCursorPos(1, 1)
for _, l in ipairs(lines) do print(l) end
print("Saved to " .. OUT)
