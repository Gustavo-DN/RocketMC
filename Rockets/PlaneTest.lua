local C = {
  LOG = "planetest.log",
  OUT = "planetest.txt",
  ENGINE_TYPE = "liquid_vector_thruster",
  GRAV = 10.5,
  REL_MASS = 0.7,
  REL_FREEFALL = true,
  REL_FALL_ACC = 6,
  REL_FALL_T = 0.4,
  REL_MIN_T = 1,
  COAST_T = 1.0,
  THR_1 = 0.07,
  THR_2 = 0.14,
  THR_3 = 0.2,
  FREE_WAIT = 120,
  BURN_T = 1.5,
  ID_THR = 0.07,
  ID_AMP = 0.4,
  ID_T = 0.4,
  ID_GAP = 0.4,
  GIMBAL_TAU = 0.23,
  HOLD_T = 4,
  GLIDE_T = 20,
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
note("PLANE TEST " .. tostring(os.date and os.date() or ""))
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
  local okm, mass = pcall(sublevel.getMass)
  note(string.format("START %s, free after %.1f s  MASS %s", why, os.clock() - wt0, tostring(okm and mass or "?")))
  flog = fs.open(C.LOG, "w")
  flog.writeLine("t phase thr gx gy px py pz vx vy vz bvx bvy bvz wx wy wz upx upy upz ms")
  local sched = {}
  local function plan(ph, t, thr, gx, gy) sched[#sched + 1] = { ph, t, thr, gx or 0, gy or 0 } end
  plan("COAST", C.COAST_T, 0)
  plan("IDGAP", C.ID_GAP, C.ID_THR)
  plan("IDX+", C.ID_T, C.ID_THR, C.ID_AMP, 0)
  plan("IDX-", C.ID_T, C.ID_THR, -C.ID_AMP, 0)
  plan("IDGAP", C.ID_GAP, C.ID_THR)
  plan("IDY+", C.ID_T, C.ID_THR, 0, C.ID_AMP)
  plan("IDY-", C.ID_T, C.ID_THR, 0, -C.ID_AMP)
  plan("COAST1", C.COAST_T, 0)
  plan("BURN1", C.BURN_T, C.THR_1)
  plan("COAST2", C.COAST_T, 0)
  plan("BURN2", C.BURN_T, C.THR_2)
  plan("COAST3", C.COAST_T, 0)
  plan("BURN3", C.BURN_T, C.THR_3)
  plan("HOLD", C.HOLD_T, C.ID_THR)
  plan("GLIDE", C.GLIDE_T, 0)
  local acc = {}
  local samples = {}
  local gs = { 0, 0 }
  local t0 = os.clock()
  local last, prevV, prevW, prevT = t0
  local si, st = 1, 0
  local stillT, maxSpin = 0, 0
  local first = {}
  while si <= #sched do
    local s = sched[si]
    cmd[1], cmd[2], cmd[3] = s[3], s[4], s[5]
    local e0 = os.epoch("utc")
    local pos, w, u, vel, om = tick(true)
    local ms = os.epoch("utc") - e0
    local now = os.clock()
    local dt = math.max(now - last, 0.01)
    last = now
    st = st + dt
    local T = now - t0
    local bv = toBody(w, u, vel)
    local up = toBody(w, u, { x = 0, y = 1, z = 0 })
    local spin = len(om)
    maxSpin = math.max(maxSpin, spin)
    if not first.bv then
      first.bv, first.up = bv, up
    end
    local a = math.min(1, dt / C.GIMBAL_TAU)
    gs[1] = gs[1] + (cmd[2] - gs[1]) * a
    gs[2] = gs[2] + (cmd[3] - gs[2]) * a
    if prevV then
      local aw = mul(sub(vel, prevV), 1 / (now - prevT))
      local ab = toBody(w, u, { x = aw.x, y = aw.y + C.GRAV, z = aw.z })
      local key = s[1]
      local A = acc[key] or { n = 0, x = 0, y = 0, z = 0, wx = 0, wy = 0, wz = 0, sp = 0 }
      A.n = A.n + 1
      A.x, A.y, A.z = A.x + ab.x, A.y + ab.y, A.z + ab.z
      A.wx, A.wy, A.wz = A.wx + aw.x, A.wy + aw.y, A.wz + aw.z
      A.sp = A.sp + len(vel)
      acc[key] = A
      if key:sub(1, 2) == "ID" then
        local k = 1 / (now - prevT)
        samples[#samples + 1] = { gs[1], gs[2], (om.x - prevW.x) * k, (om.y - prevW.y) * k, (om.z - prevW.z) * k }
      end
    end
    prevV, prevW, prevT = vel, om, now
    flog.writeLine(string.format("%.2f %s %.3f %+.2f %+.2f %.1f %.1f %.1f %+.2f %+.2f %+.2f %+.2f %+.2f %+.2f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %d",
      T, s[1], cmd[1], cmd[2], cmd[3], pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, bv.x, bv.y, bv.z, om.x, om.y, om.z, up.x, up.y, up.z, ms))
    if spin > C.CUT_SPIN and s[3] > 0 then
      note(string.format("SPIN %.2f rad/s at %.1f s: engine off, gliding", spin, T))
      while si <= #sched and sched[si][1] ~= "GLIDE" do si = si + 1 end
      st = 0
    end
    if len(vel) < C.STOP_V then stillT = stillT + dt else stillT = 0 end
    if stillT > C.STOP_T and T > 2 then R.reason = string.format("stopped at %.1f s", T) break end
    if st >= s[2] then si, st = si + 1, 0 end
  end
  if R.reason == "?" then R.reason = "schedule done" end
  cut()
  note(string.format("FIRST body velocity %s  world-up in body %s", f3(first.bv or { x = 0, y = 0, z = 0 }), f3(first.up or { x = 0, y = 0, z = 0 })))
  local function avg(k)
    local A = acc[k]
    if not A or A.n == 0 then return nil end
    return { x = A.x / A.n, y = A.y / A.n, z = A.z / A.n }, { x = A.wx / A.n, y = A.wy / A.n, z = A.wz / A.n }, A.sp / A.n
  end
  for _, k in ipairs({ "COAST", "COAST1", "BURN1", "COAST2", "BURN2", "COAST3", "BURN3", "HOLD", "GLIDE" }) do
    local b, wv, sp = avg(k)
    if b then note(string.format("%-6s air+thrust force/mass in body %s  world acc %s  speed %.1f", k, f3(b), f3(wv), sp)) end
  end
  local c1, c2, b1, b2 = avg("COAST1"), avg("COAST2"), avg("BURN1"), avg("BURN2")
  if c1 and b1 then
    local d = sub(b1, c1)
    note(string.format("THRUST step 1 (%.2f): %s  => %.1f m/s2, forward axis in body = %s", C.THR_1, f3(d), len(d), f3(mul(d, 1 / math.max(len(d), 1e-6)))))
  end
  if c2 and b2 then
    local d = sub(b2, c2)
    note(string.format("THRUST step 2 (%.2f): %s  => %.1f m/s2", C.THR_2, f3(d), len(d)))
  end
  local c3, b3 = avg("COAST3"), avg("BURN3")
  if c3 and b3 then
    local d = sub(b3, c3)
    note(string.format("THRUST step 3 (%.2f): %s  => %.1f m/s2", C.THR_3, f3(d), len(d)))
  end
  if #samples > 8 then
    local A, bx, by, bz = {}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }
    for i = 1, 3 do A[i] = { 0, 0, 0 } end
    for _, s in ipairs(samples) do
      local r = { s[1], s[2], 1 }
      for i = 1, 3 do
        for j = 1, 3 do A[i][j] = A[i][j] + r[i] * r[j] end
        bx[i] = bx[i] + r[i] * s[3]
        by[i] = by[i] + r[i] * s[4]
        bz[i] = bz[i] + r[i] * s[5]
      end
    end
    local cx, cy, cz = solve(A, bx, 3), solve(A, by, 3), solve(A, bz, 3)
    if cx and cy and cz then
      note(string.format("GIMBAL MAP at throttle %.2f (rad/s2 per unit):", C.ID_THR))
      note(string.format("  ang_x = %+.2f*gx %+.2f*gy %+.2f", cx[1], cx[2], cx[3]))
      note(string.format("  ang_y = %+.2f*gx %+.2f*gy %+.2f", cy[1], cy[2], cy[3]))
      note(string.format("  ang_z = %+.2f*gx %+.2f*gy %+.2f", cz[1], cz[2], cz[3]))
    else
      note("GIMBAL MAP: not enough response")
    end
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
