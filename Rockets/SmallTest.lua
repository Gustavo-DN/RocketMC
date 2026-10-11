local C = {
  LOG = "smalltest.log",
  OUT = "smalltest.txt",
  ENGINE_TYPE = "liquid_vector_thruster",
  COUNTDOWN = 5,
  GRAV = 10.5,
  START_TILT_MAX = 15,
  GIMBAL_TEST = 0.5,
  GIMBAL_TICKS = 30,
  GIMBAL_TAU = 0.23,
  RAMP = 0.2,
  MAX_LIFT_THR = 0.9,
  LIFTOFF_VY = 0.4,
  LIFTOFF_DH = 0.3,
  ID_FREE = 0.3,
  ID_AMP = 0.15,
  ID_T = 0.3,
  ID_GAP = 0.2,
  ID_MIN_DET = 1e-4,
  HOP = 8,
  KP_ATT = 1.5,
  KD_ATT = 1.6,
  KI_ATT = 0.3,
  ALPHA_MAX = 3,
  GMAX = 0.6,
  KH = 0.8,
  KV = 1.2,
  KI_H = 0.01,
  CLIMB_T = 6,
  HOVER_T = 4,
  STEP_DEG = 5,
  STEP_T = 2.5,
  LAND_V = 1.5,
  LAND_DONE_T = 0.5,
  LAND_H = 1,
  LAND_UNDER = 3,
  CUT_TILT = 40,
  CUT_SPIN = 4,
  MAX_T = 120,
}

local atan2 = math.atan2 or math.atan
local function v3(a) return { x = a.x or a[1], y = a.y or a[2], z = a.z or a[3] } end
local function add(a, b) return { x = a.x + b.x, y = a.y + b.y, z = a.z + b.z } end
local function mul(a, s) return { x = a.x * s, y = a.y * s, z = a.z * s } end
local function cross(a, b) return { x = a.y * b.z - a.z * b.y, y = a.z * b.x - a.x * b.z, z = a.x * b.y - a.y * b.x } end
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

local out = {}
local function note(s)
  out[#out + 1] = s
  print(s)
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

local names, tanks = {}, {}
for _, n in ipairs(peripheral.getNames()) do
  if peripheral.hasType(n, C.ENGINE_TYPE) then
    names[#names + 1] = n
  elseif peripheral.hasType(n, "fluid_storage") then
    tanks[#tanks + 1] = n
  end
end
table.sort(names)

term.clear()
term.setCursorPos(1, 1)
note("SMALL TEST " .. os.date())
for _, n in ipairs(peripheral.getNames()) do
  note("PERIPHERAL " .. n .. " : " .. table.concat({ peripheral.getType(n) }, ","))
end
if #names == 0 then
  note("No thrusters found on the network.")
  local f = fs.open(C.OUT, "w") f.write(table.concat(out, "\n")) f.close()
  return
end
note("THRUSTERS " .. table.concat(names, " "))
note("METHODS " .. table.concat(peripheral.getMethods(names[1]), " "))
note("TANKS " .. (#tanks > 0 and table.concat(tanks, " ") or "none"))
if not sublevel then
  note("No sublevel API: the computer is not on a Sable sub-level.")
  local f = fs.open(C.OUT, "w") f.write(table.concat(out, "\n")) f.close()
  return
end
pcall(function() note("ON SUBLEVEL " .. tostring(sublevel.isInPlotGrid())) end)
pcall(function() note("MASS " .. tostring(sublevel.getMass())) end)
pcall(function() note("INERTIA " .. textutils.serialize(sublevel.getInertiaTensor(), { compact = true })) end)

local eng = {}
for i, n in ipairs(names) do eng[i] = peripheral.wrap(n) end
local cmd = { 0, 0, 0 }

local function io(send)
  local fns, p, v, o = {}
  if send then
    for i = 1, #eng do
      local e = eng[i]
      fns[#fns + 1] = function() e.setThrustNormalized(cmd[1]) end
      fns[#fns + 1] = function() e.setVector(cmd[2], cmd[3]) end
    end
  end
  fns[#fns + 1] = function() p = sublevel.getLogicalPose() end
  fns[#fns + 1] = function() v = sublevel.getLinearVelocity() end
  fns[#fns + 1] = function() o = sublevel.getAngularVelocity() end
  local a = os.epoch("utc")
  parallel.waitForAll(table.unpack(fns))
  local w, u = quat(p.orientation)
  return v3(p.position), w, u, v3(v), v3(o), os.epoch("utc") - a
end

local function cut()
  cmd[1], cmd[2], cmd[3] = 0, 0, 0
  if not pcall(io, true) then
    for i = 1, #eng do
      pcall(eng[i].setThrustNormalized, 0)
      pcall(eng[i].setVector, 0, 0)
    end
  end
end

local function fuel()
  while true do
    local fns = {}
    for i = 1, #eng do
      local e = eng[i]
      for _, t in ipairs(tanks) do fns[#fns + 1] = function() pcall(e.pullFluid, t) end end
    end
    if #fns > 0 then parallel.waitForAll(table.unpack(fns)) end
    sleep(0.5)
  end
end

local R = { tau = C.GIMBAL_TAU, reason = "?" }
local flog

local function gimbalTest()
  local e = eng[1]
  pcall(e.setThrustNormalized, 0)
  local ok = pcall(e.setVector, C.GIMBAL_TEST, 0)
  if not ok then note("GIMBAL setVector failed") return end
  local t0, t63, last = now()
  local rows = {}
  for _ = 1, C.GIMBAL_TICKS do
    local okx, gx = pcall(e.getVectorX)
    local okt, tx = pcall(e.getTargetVectorX)
    local t = now() - t0
    if okx and type(gx) == "number" then
      last = gx
      if not t63 and gx >= 0.632 * C.GIMBAL_TEST then t63 = t end
    end
    rows[#rows + 1] = string.format("%.2f %s %s", t, okx and tostring(gx) or "err", okt and tostring(tx) or "err")
  end
  pcall(e.setVector, 0, 0)
  note("GIMBAL STEP 0 -> " .. C.GIMBAL_TEST .. " (t getVectorX getTargetVectorX):")
  for _, r in ipairs(rows) do note("  " .. r) end
  if t63 then
    R.tau = t63
    note(string.format("GIMBAL t63 %.2f s, final %.3f", t63, last or 0))
  else
    note("GIMBAL t63 not seen, using " .. C.GIMBAL_TAU)
  end
  sleep(1)
end

local function main()
  gimbalTest()
  local p0, w0, u0 = io(false)
  local up0 = toBody(w0, u0, { x = 0, y = 1, z = 0 })
  local tilt0 = math.deg(math.acos(clamp(up0.y, -1, 1)))
  note(string.format("START pos %.1f %.1f %.1f tilt %.1f", p0.x, p0.y, p0.z, tilt0))
  if tilt0 > C.START_TILT_MAX then
    note("START TILT TOO BIG: the thruster may not point along body Y. Not launching.")
    R.reason = "start tilt"
    return
  end
  for i = C.COUNTDOWN, 1, -1 do
    print("LAUNCH IN " .. i .. "  (Ctrl+T cuts the engine)")
    sleep(1)
  end
  flog = fs.open(C.LOG, "w")
  flog.writeLine("t phase thr gx gy h vy vh tilt upx upz wx wy wz ms")
  local y0 = p0.y
  local t0, tl = now(), now()
  local phase, pt = "RAMP", 0
  local thr, thrH, hT = 0, 0, 0
  local gs = { 0, 0 }
  local ix, iz = 0, 0
  local M, b
  local samples = {}
  local prevW, prevT
  local hovSum, hovN = 0, 0
  local step = { maxTilt = 0 }
  local stepAx, stepSign = 0, 0
  local maxSpin, maxRoll, landT = 0, 0, 0
  while true do
    local pos, w, u, vel, om, ms = io(true)
    local t = now()
    local dt = clamp(t - tl, 0.01, 0.3)
    tl = t
    local T = t - t0
    pt = pt + dt
    local up = toBody(w, u, { x = 0, y = 1, z = 0 })
    local tilt = math.deg(math.acos(clamp(up.y, -1, 1)))
    local h = pos.y - y0
    local vh = math.sqrt(vel.x * vel.x + vel.z * vel.z)
    local spin = math.sqrt(om.x * om.x + om.z * om.z)
    maxSpin = math.max(maxSpin, spin)
    maxRoll = math.max(maxRoll, math.abs(om.y))
    local a = math.min(1, dt / R.tau)
    gs[1] = gs[1] + (cmd[2] - gs[1]) * a
    gs[2] = gs[2] + (cmd[3] - gs[2]) * a
    if prevW and phase == "ID" then
      samples[#samples + 1] = { gs[1], gs[2], (om.x - prevW.x) / (t - prevT), (om.z - prevW.z) / (t - prevT) }
    end
    prevW, prevT = om, t
    if phase ~= "RAMP" and (tilt > C.CUT_TILT or spin > C.CUT_SPIN) then
      R.reason = string.format("CUT tilt %.1f spin %.2f at %.1f s", tilt, spin, T)
      break
    end
    if T > C.MAX_T then R.reason = "time limit" break end
    local gx, gy = 0, 0
    if phase == "RAMP" then
      thr = math.min(thr + C.RAMP * dt, C.MAX_LIFT_THR)
      if vel.y > C.LIFTOFF_VY or h > C.LIFTOFF_DH then
        R.liftThr, R.liftT = thr, T
        thrH = thr
        note(string.format("LIFTOFF throttle %.3f at %.1f s", thr, T))
        phase, pt = "ID", 0
      elseif thr >= C.MAX_LIFT_THR and pt > 15 then
        R.reason = "no liftoff at max throttle (docked or too heavy?)"
        break
      end
    elseif phase == "ID" then
      local s1 = C.ID_FREE
      local s2 = s1 + C.ID_T
      local s3 = s2 + C.ID_T
      local s4 = s3 + C.ID_GAP
      local s5 = s4 + C.ID_T
      local s6 = s5 + C.ID_T
      if pt < s1 then
      elseif pt < s2 then gx = C.ID_AMP
      elseif pt < s3 then gx = -C.ID_AMP
      elseif pt < s4 then
      elseif pt < s5 then gy = C.ID_AMP
      elseif pt < s6 then gy = -C.ID_AMP
      else
        local A, bx, bz = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } }, { 0, 0, 0 }, { 0, 0, 0 }
        for _, s in ipairs(samples) do
          local r = { s[1], s[2], 1 }
          for i = 1, 3 do
            for j = 1, 3 do A[i][j] = A[i][j] + r[i] * r[j] end
            bx[i] = bx[i] + r[i] * s[3]
            bz[i] = bz[i] + r[i] * s[4]
          end
        end
        local cx, cz = solve3(A, bx), solve3(A, bz)
        if cx and cz then
          M = { { cx[1], cx[2] }, { cz[1], cz[2] } }
          b = { cx[3], cz[3] }
          R.det = M[1][1] * M[2][2] - M[1][2] * M[2][1]
          note(string.format("GIMBAL MAP ang_x = %+.2f*gx %+.2f*gy %+.2f   ang_z = %+.2f*gx %+.2f*gy %+.2f  (rad/s2)",
            M[1][1], M[1][2], b[1], M[2][1], M[2][2], b[2]))
        end
        if not M or math.abs(R.det) < C.ID_MIN_DET then
          R.reason = "gimbal had no measurable effect"
          note("GIMBAL MAP FAILED")
          break
        end
        R.M, R.b = M, b
        phase, pt, hT = "CLIMB", 0, math.max(h, 0)
      end
      thr = thrH * (1 + clamp(-C.KV * vel.y, -3, 3) / C.GRAV)
    else
      local want = { x = 0, y = 1, z = 0 }
      if phase == "CLIMB" then
        hT = math.min(C.HOP, hT + C.HOP / C.CLIMB_T * dt)
        if pt > C.CLIMB_T + 2 then phase, pt = "HOVER", 0 end
      elseif phase == "HOVER" then
        hovSum, hovN = hovSum + thr * math.max(up.y, 0.5), hovN + 1
        if pt > C.HOVER_T then phase, pt, stepAx, stepSign = "STEP", 0, 1, 1 end
      elseif phase == "STEP" then
        local r = math.rad(C.STEP_DEG) * stepSign
        if pt < C.STEP_T then
          want = stepAx == 1 and { x = math.sin(r), y = math.cos(r), z = 0 } or { x = 0, y = math.cos(r), z = math.sin(r) }
          step.maxTilt = math.max(step.maxTilt, tilt)
        end
        if pt > 2 * C.STEP_T then
          note(string.format("STEP %s %+d deg: peak tilt %.1f", stepAx == 1 and "X" or "Z", C.STEP_DEG * stepSign, step.maxTilt))
          step.maxTilt = 0
          pt = 0
          if stepAx == 1 then stepAx = 2 else phase, pt = "LAND", 0 end
        end
      elseif phase == "LAND" then
        hT = hT - C.LAND_V * dt
        if (h < C.LAND_H or hT < -C.LAND_UNDER) and math.abs(vel.y) < 0.3 then landT = landT + dt else landT = 0 end
        if landT > C.LAND_DONE_T then R.reason = "landed" break end
      end
      local d = toBody(w, u, want)
      local ex, ez = d.z, -d.x
      ix = clamp(ix + ex * dt, -0.5, 0.5)
      iz = clamp(iz + ez * dt, -0.5, 0.5)
      local ax = clamp(C.KP_ATT * ex + C.KI_ATT * ix - C.KD_ATT * om.x - b[1], -C.ALPHA_MAX, C.ALPHA_MAX)
      local az = clamp(C.KP_ATT * ez + C.KI_ATT * iz - C.KD_ATT * om.z - b[2], -C.ALPHA_MAX, C.ALPHA_MAX)
      local det = R.det
      gx = (M[2][2] * ax - M[1][2] * az) / det
      gy = (-M[2][1] * ax + M[1][1] * az) / det
      local gm = math.max(math.abs(gx), math.abs(gy))
      if gm > C.GMAX then gx, gy = gx * C.GMAX / gm, gy * C.GMAX / gm end
      thrH = clamp(thrH + C.KI_H * (hT - h) * dt, 0.05, 1)
      thr = thrH * (1 + clamp(C.KH * (hT - h) - C.KV * vel.y, -5, 5) / C.GRAV) / math.max(up.y, 0.7)
    end
    cmd[1], cmd[2], cmd[3] = clamp(thr, 0, 1), clamp(gx, -1, 1), clamp(gy, -1, 1)
    flog.writeLine(string.format("%.2f %s %.3f %+.3f %+.3f %.2f %+.2f %.2f %.1f %+.3f %+.3f %+.3f %+.3f %+.3f %d",
      T, phase, cmd[1], cmd[2], cmd[3], h, vel.y, vh, tilt, up.x, up.z, om.x, om.y, om.z, ms))
  end
  R.hover = hovN > 0 and hovSum / hovN or nil
  R.maxSpin, R.maxRoll = maxSpin, maxRoll
end

local ok, err = pcall(parallel.waitForAny, main, fuel)
cut()
if flog then flog.close() end
if not ok then R.reason = "error: " .. tostring(err) end
note("END " .. R.reason)
if R.liftThr then note(string.format("LIFTOFF THROTTLE %.3f", R.liftThr)) end
if R.hover then
  note(string.format("HOVER THROTTLE %.3f  => thrust %.1f m/s2 per unit throttle", R.hover, C.GRAV / R.hover))
end
if R.maxSpin then note(string.format("MAX PITCH/YAW RATE %.2f rad/s, MAX ROLL RATE %.2f rad/s", R.maxSpin, R.maxRoll)) end
local f = fs.open(C.OUT, "w")
f.write(table.concat(out, "\n") .. "\n")
f.close()
print("Saved " .. C.OUT .. " and " .. C.LOG)
