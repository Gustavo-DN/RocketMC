local C = {
  LOG = "planeflight.log",
  OUT = "planeflight.txt",
  ENGINE_TYPE = "liquid_vector_thruster",
  GRAV = 10.5,
  REL_MASS = 0.7,
  REL_FREEFALL = true,
  REL_FALL_ACC = 6,
  REL_FALL_T = 0.4,
  REL_MIN_T = 1,
  FREE_WAIT = 120,
  FWD = { x = 0, y = 0, z = 1 },
  MAP_PITCH = -4.37,
  MAP_YAW = -3.88,
  KP = 4,
  KD = 3,
  KI = 3,
  IMAX = 1.0,
  ERR_MAX = 0.6,
  GMAX = 0.9,
  SEG_T = 3,
  AVG_FROM = 1,
  SEGMENTS = { { 3, 0 }, { 1, 0 }, { 2, 0 }, { 2, 15 }, { 3, 15 }, { 4, 20 }, { 5, 30 }, { 6, 35 }, { 8, 40 }, { 6, 20 }, { 4, 10 } },
  CUT_SPIN = 4,
  STOP_V = 0.5,
  STOP_T = 1,
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
note("PLANE FLIGHT TEST " .. tostring(os.date and os.date() or ""))
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

local function waitRelease()
  print("")
  print("ARMED. Starts on: separation from the carrier,")
  print("free fall, redstone on any side, or ENTER.")
  print("BACKSPACE cancels.")
  local why
  local function watch()
    local _, _, _, v0, _, m0 = tick(false, true)
    local t0, last, lastVy, fall = os.clock(), os.clock(), v0.y, 0
    while true do
      local _, _, _, vel, _, m = tick(false, true)
      local now = os.clock()
      local dt = now - last
      last = now
      if m0 and m and m < m0 * C.REL_MASS then why = "SEPARATED" return end
      for _, side in ipairs(redstone.getSides()) do
        if redstone.getInput(side) then why = "REDSTONE " .. side return end
      end
      if dt > 0 then
        if (vel.y - lastVy) / dt < -C.REL_FALL_ACC then fall = fall + dt else fall = 0 end
        lastVy = vel.y
      end
      if C.REL_FREEFALL and now - t0 > C.REL_MIN_T and fall >= C.REL_FALL_T then why = "FREE FALL" return end
    end
  end
  local function keysIn()
    while true do
      local _, k = os.pullEvent("key")
      if k == keys.enter or k == keys.numPadEnter then why = "MANUAL" return end
      if k == keys.backspace then why = false return end
    end
  end
  parallel.waitForAny(watch, keysIn)
  return why
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
  local ax = C.KP * ex + C.KI * I.x - C.KD * om.x
  local ay = C.KP * ey + C.KI * I.y - C.KD * om.y
  local gy = clamp(ax / (C.MAP_PITCH * steps), -C.GMAX, C.GMAX)
  local gx = clamp(ay / (C.MAP_YAW * steps), -C.GMAX, C.GMAX)
  return gx, gy
end

local function main()
  cut()
  local why = waitRelease()
  if not why then R.reason = "cancelled" return end
  print("Waiting until it moves freely (not held)...")
  local p0, _, _, v0 = tick(false)
  local wt0, moved = os.clock(), false
  while os.clock() - wt0 < C.FREE_WAIT do
    local p1, _, _, v1 = tick(false)
    if len(sub(p1, p0)) > 0.02 or len(sub(v1, v0)) > 0.05 then moved = true break end
    p0, v0 = p1, v1
  end
  if not moved then R.reason = "never moved (still held?)" return end
  local _, w0, u0, vs = tick(false)
  local f0 = rotate(w0, u0, C.FWD)
  local hl = math.sqrt(f0.x * f0.x + f0.z * f0.z)
  local head = hl > 0.1 and { x = f0.x / hl, y = 0, z = f0.z / hl } or { x = vs.x, y = 0, z = vs.z }
  local hn = math.sqrt(head.x * head.x + head.z * head.z)
  head = { x = head.x / math.max(hn, 1e-6), y = 0, z = head.z / math.max(hn, 1e-6) }
  local okm, mass = pcall(sublevel.getMass)
  note(string.format("START %s, free after %.1f s  MASS %s", why, os.clock() - wt0, tostring(okm and mass or "?")))
  note("seg steps pitch_cmd | pitch err_max aoa | vy speed climb_m | gimbal_pitch gimbal_yaw | held")
  flog = fs.open(C.LOG, "w")
  flog.writeLine("t seg steps pcmd thr gx gy px py pz vx vy vz speed pitch aoa err wx wy wz")
  local t0 = os.clock()
  local last = t0
  local stillT, maxSpin = 0, 0
  local seg, st = 1, 0
  local acc = {}
  local engineOff = false
  local cmdThr, cmdX, cmdY = 0, 0, 0
  while seg <= #C.SEGMENTS do
    local S = C.SEGMENTS[seg]
    local steps, pcmd = S[1], math.rad(S[2])
    cmd[1], cmd[2], cmd[3] = cmdThr, cmdX, cmdY
    local pos, w, u, vel, om = tick(true)
    local now = os.clock()
    local dt = math.max(now - last, 0.01)
    last = now
    st = st + dt
    local nose = rotate(w, u, C.FWD)
    local pitch = math.asin(clamp(nose.y, -1, 1))
    local sp = len(vel)
    local bv = toBody(w, u, vel)
    local aoa = sp > 1 and math.deg(atan2(-bv.y, bv.z)) or 0
    local err = math.deg(math.abs(pitch - pcmd))
    local spin = len(om)
    maxSpin = math.max(maxSpin, spin)
    if spin > C.CUT_SPIN then engineOff = true end
    if engineOff then
      cmdThr, cmdX, cmdY = 0, 0, 0
    else
      cmdThr = steps / 15 + 0.003
      cmdX, cmdY = hold(w, u, om, head, pcmd, steps, dt)
    end
    if st >= C.AVG_FROM then
      local A = acc[seg] or { n = 0, pitch = 0, aoa = 0, vy = 0, sp = 0, gx = 0, gy = 0, emax = 0, y0 = pos.y, t0 = now }
      A.n = A.n + 1
      A.pitch, A.aoa, A.vy, A.sp = A.pitch + math.deg(pitch), A.aoa + aoa, A.vy + vel.y, A.sp + sp
      A.gx, A.gy = A.gx + math.abs(cmdX), A.gy + math.abs(cmdY)
      A.emax = math.max(A.emax, err)
      A.y1, A.t1 = pos.y, now
      acc[seg] = A
    end
    flog.writeLine(string.format("%.2f %d %d %d %.3f %+.2f %+.2f %.1f %.1f %.1f %+.2f %+.2f %+.2f %.1f %+.1f %+.1f %.1f %+.3f %+.3f %+.3f",
      now - t0, seg, steps, S[2], cmdThr, cmdX, cmdY, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, sp, math.deg(pitch), aoa, err, om.x, om.y, om.z))
    if sp < C.STOP_V then stillT = stillT + dt else stillT = 0 end
    if stillT > C.STOP_T and now - t0 > 2 then R.reason = string.format("stopped (landed?) at %.1f s in segment %d", now - t0, seg) break end
    if st >= C.SEG_T then
      local A = acc[seg]
      if A and A.n > 0 then
        local n = A.n
        note(string.format("%2d %d %3d | %+5.1f %4.1f %+5.1f | %+6.1f %5.1f %+6.1f | %.2f %.2f | %s",
          seg, steps, S[2], A.pitch / n, A.emax, A.aoa / n, A.vy / n, A.sp / n, A.y1 - A.y0, A.gy / n, A.gx / n, A.emax < 8 and "yes" or "NO"))
      end
      seg, st = seg + 1, 0
    end
  end
  if R.reason == "?" then R.reason = "all segments done" end
  if engineOff then note("ENGINE OFF after a spin over " .. C.CUT_SPIN .. " rad/s") end
  note(string.format("MAX SPIN %.2f rad/s", maxSpin))
end

local ok, err = pcall(main)
cut()
if flog then flog.close() end
if not ok then R.reason = "error: " .. tostring(err) end
note("END " .. R.reason)
save()
print("Saved " .. C.OUT .. " and " .. C.LOG)
