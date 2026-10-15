local C = {
  ENGINE_SIDE = "bottom",
  CLIMB = 150,
  COUNTDOWN = 5,
  START_TILT = 15,
  GRAV = 10.5,
  THRUST_FORCE = 427,
  REF_I = 15.57,
  KP = 5,
  KD = 3,
  ATT_KI = 3,
  ATT_IWIN = 20,
  ATT_IMAX = 1,
  RATE_LEAD = 0.2,
  RATE_TAU = 0.15,
  RATE_ACC_MAX = 12,
  FF_TAU = 0.2,
  FF_MAX = 1.5,
  FF_JUMP = 0.94,
  ERR_MAX = 1.2,
  VMAX = 1,
  MAP_X = 3.37,
  MAP_Y = -1.95,
  MAP_CROSS = -0.94,
  MAP_THR = 2 / 15,
  CLIMB_SPEED = 10,
  CLIMB_K = 0.8,
  PITCH_RATE = 10,
  ALT_K = 0.15,
  VY_K = 0.8,
  ALT_ACC = 4,
  ALT_I = 0.015,
  ALT_I_MAX = 3,
  TRACK_K = 0.04,
  TRACK_D = 0.6,
  TRACK_ANGLE = 5,
  START_STEPS = 3,
  STEP_SECONDS = 10,
  STEP_WAIT_MAX = 45,
  FUEL_EMPTY_SECONDS = 1,
  STABLE_WINDOW = 5,
  STABLE_CROSS = 25,
  STABLE_ALT = 15,
  STABLE_ERROR = 10,
  STABLE_SPIN = 0.6,
  MAX_CROSS = 50,
  MAX_ALT_ERROR = 40,
  MAX_HEADING_ERROR = 15,
  MAX_ATT_ERROR = 35,
  MAX_SPIN = 2,
  HARD_SPIN = 4,
  BAD_SECONDS = 0.5,
  HEADING_MIN_SPEED = 20,
  SPEED_LIMIT = 1000,
  PLATEAU_WINDOW = 30,
  PLATEAU_ACCEL = 0.1,
  PLATEAU_SECONDS = 10,
  SETUP_TIMEOUT = 120,
  MAX_TEST_SECONDS = 600,
  RECOVERY_TILT = 30,
  RECOVERY_RATE = 15,
  RECOVERY_BRAKE = 0.3,
  DESCENT_SPEED = 3,
  LAND_HEIGHT = 2,
  LAND_SECONDS = 1,
  LOG_DT = 0.05,
  FUEL_DT = 0.5,
  LOG = "rocketlimit.log",
  SUMMARY = "rocketlimit.txt",
}
local originalPullEvent = os.pullEvent
local args = { ... }
local remote = type(args[1]) == "table" and args[1] or nil
local heading = not remote and tonumber(args[1]) or nil
local height = not remote and tonumber(args[2]) or nil
if (args[1] and not remote and not heading) or (args[2] and not height) or (height and (height < 100 or height > 200)) then
  print("Usage: RocketLimitTest [heading_degrees] [height_100_to_200]")
  print("Heading: 0=+Z, 90=+X, 180=-Z, 270=-X; default body +Z.")
  os.pullEvent = originalPullEvent
  return
end
C.CLIMB = height or C.CLIMB
local atan2 = math.atan2 or math.atan
local inertiaK = 1
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


local eng = peripheral.wrap(C.ENGINE_SIDE)
if not eng or not eng.setThrustNormalized or not eng.setVector then error("No vector thruster on " .. C.ENGINE_SIDE, 0) end
local tanks = {}
for _, name in ipairs(peripheral.getNames()) do
  if name ~= C.ENGINE_SIDE and peripheral.hasType(name, "fluid_storage") and not peripheral.hasType(name, "liquid_vector_thruster") then tanks[#tanks + 1] = name end
end
local throttle, gx, gy = 0, 0, 0
local nextFuel, nextLog, abort = 0, 0, false
local pos, w, u, vel, om, actualX, actualY
local mass, tacc, ground, forward, right, start, fuelAmount
local stage, reason, stageT = "CLIMB", nil, 0
local maxSpeed, stableSpeed, stableMean, stableAt, stableSteps = 0, 0, 0, 0, 0
local testSpeed, testTime, landed = nil, nil, false
local log
local function note(s)
  if log then log.writeLine("EVENT " .. s) log.flush() end
  print(s)
end
local function save(status)
  local out = fs.open(C.SUMMARY, "w")
  if not out then return end
  out.writeLine("SINGLE THRUSTER LIMIT TEST - " .. status)
  out.writeLine(string.format("MASS %.3f TACC %.3f GAIN %.3f HEIGHT %.1f", mass or 0, tacc or 0, inertiaK, C.CLIMB))
  if forward then out.writeLine(string.format("HEADING %.4f %.4f (world X,Z)", forward.x, forward.z)) end
  out.writeLine("STOP " .. (reason or "not reached"))
  out.writeLine("FUEL_MB " .. tostring(fuelAmount or "unknown"))
  out.writeLine(string.format("TEST_END_TIME %.2f TEST_END_SPEED %.2f", testTime or 0, testSpeed or 0))
  out.writeLine(string.format("PEAK_FORWARD_SPEED %.2f", maxSpeed))
  out.writeLine(string.format("BEST_STABLE_5S_MEAN %.2f END_SPEED %.2f AT %.2f STEPS %d", stableMean, stableSpeed, stableAt, stableSteps))
  out.writeLine(string.format("Limits: cross %.1f blocks, altitude +/-%.1f, heading %.1f deg, attitude %.1f deg, spin %.1f rad/s", C.MAX_CROSS, C.MAX_ALT_ERROR, C.MAX_HEADING_ERROR, C.MAX_ATT_ERROR, C.MAX_SPIN))
  out.writeLine("Stable means complete 5s window within cross/altitude/error/spin thresholds; peak alone is not a stable limit.")
  out.writeLine("This measures this craft and controller, not an intrinsic hardware limit. Recovery is not included in peak/stable speeds.")
  out.writeLine("Recovery assumes flat terrain at launch height; no automatic return to launch.")
  out.close()
end
local function tick()
  local p, v, o
  local fns = {
    function() p = sublevel.getLogicalPose() end,
    function() v = sublevel.getLinearVelocity() end,
    function() o = sublevel.getAngularVelocity() end,
    function() eng.setThrustNormalized(throttle) end,
    function() eng.setVector(gx, gy) end,
  }
  actualX, actualY = nil, nil
  if eng.getVectorX then fns[#fns + 1] = function() local ok, value = pcall(eng.getVectorX) if ok then actualX = value end end end
  if eng.getVectorY then fns[#fns + 1] = function() local ok, value = pcall(eng.getVectorY) if ok then actualY = value end end end
  if os.clock() >= nextFuel then
    fns[#fns + 1] = function()
      local ok, contents = pcall(peripheral.call, C.ENGINE_SIDE, "tanks")
      if ok and type(contents) == "table" then
        fuelAmount = 0
        for _, fluid in pairs(contents) do fuelAmount = fuelAmount + (fluid.amount or 0) end
      end
    end
    for _, name in ipairs(tanks) do
      local tank = name
      fns[#fns + 1] = function() pcall(eng.pullFluid, tank) end
    end
    nextFuel = os.clock() + C.FUEL_DT
  end
  parallel.waitForAll((table.unpack or unpack)(fns))
  w, u = quat(p.orientation)
  pos, vel, om = v3(p.position), v3(v), v3(o)
end
local function quantize(value)
  return clamp(math.floor(value * 15 + 0.5), 1, 15) / 15
end
local function stop(why, t)
  if reason then return end
  reason, testTime, testSpeed = why, t, dot(vel, forward)
  stage, stageT = "RECOVER", 0
  attI, ff, rates = { x = 0, z = 0 }, { w = { x = 0, y = 0, z = 0 } }, { acc = { x = 0, y = 0, z = 0 } }
  note(string.format("TEST_END %s t=%.2f forward=%.2f stable_mean=%.2f", reason, t, testSpeed, stableMean))
  save("TEST ENDED; RECOVERING")
end
local function run()
  mass = sublevel.getMass()
  local it = sublevel.getInertiaTensor()
  local ix = it[1][1] or it[1].x
  local iz = it[3][3] or it[3].z
  if not mass or mass <= 0 or not ix or not iz then error("Invalid mass/inertia", 0) end
  tacc = C.THRUST_FORCE / mass
  inertiaK = clamp(math.max(ix, iz) / C.REF_I, 0.25, 4)
  if tacc < C.GRAV * 1.5 then error("Insufficient measured thrust-to-mass ratio", 0) end
  tick()
  ground, start = pos.y, pos
  local nose = rotate(w, u, UP)
  if nose.y < math.cos(math.rad(C.START_TILT)) then error("Stand rocket upright before starting", 0) end
  local f = rotate(w, u, { x = 0, y = 0, z = 1 })
  forward = heading and { x = math.sin(math.rad(heading)), y = 0, z = math.cos(math.rad(heading)) } or unit({ x = f.x, y = 0, z = f.z })
  right = { x = forward.z, y = 0, z = -forward.x }
  if fs.exists(C.LOG) or fs.exists(C.SUMMARY) then
    local stamp = tostring(os.epoch("utc"))
    if fs.exists(C.LOG) then fs.move(C.LOG, C.LOG .. "." .. stamp) end
    if fs.exists(C.SUMMARY) then fs.move(C.SUMMARY, C.SUMMARY .. "." .. stamp) end
  end
  log = assert(fs.open(C.LOG, "w"))
  note(string.format("START heading %.1f deg height %.0f mass %.2f; fuel sources %d", math.deg(atan2(forward.x, forward.z)), C.CLIMB, mass, #tanks))
  note("Backspace ends acceleration and starts recovery; engine stays powered in the air.")
  log.writeLine("t stage steps forward_speed total_speed x y z cross alt_error vx vy vz nose_x nose_y nose_z cmd_x cmd_y cmd_z attitude_deg heading_deg wx wy wz gx gy actual_gx actual_gy loop_ms fuel_mb")
  for i = C.COUNTDOWN, 1, -1 do
    if abort then reason = "CANCELLED" landed = true save("CANCELLED ON PAD") return end
    print("Launch in " .. i)
    sleep(1)
  end
  local t0, last = os.clock(), os.clock()
  local cmd, pitch, bias = UP, 90, 0
  local bad, calm, ramp, contact, plateau, empty = 0, 0, 0, 0, 0, 0
  local steps = math.max(C.START_STEPS, math.floor(C.GRAV / tacc * 15) + 1)
  local hist = {}
  throttle = quantize(C.GRAV / tacc)
  while true do
    tick()
    local now = os.clock()
    local elapsed = now - last
    local dt = clamp(elapsed, 0.01, 0.5)
    last = now
    local t = now - t0
    stageT = stageT + dt
    local nose = rotate(w, u, UP)
    local speed, along = len(vel), dot(vel, forward)
    local crossTrack, sideV = dot(sub(pos, start), right), dot(vel, right)
    local altError = pos.y - (ground + C.CLIMB)
    local hspeed = math.sqrt(vel.x * vel.x + vel.z * vel.z)
    local herror = hspeed > C.HEADING_MIN_SPEED and math.deg(math.acos(clamp(along / hspeed, -1, 1))) or 0
    local err = math.deg(math.acos(clamp(dot(nose, cmd), -1, 1)))
    local spin = len(om)
    empty = fuelAmount == 0 and empty + dt or 0
    if not reason then
      maxSpeed = math.max(maxSpeed, along)
      if along >= C.SPEED_LIMIT then stop("1000 M/S", t)
      elseif empty >= C.FUEL_EMPTY_SECONDS then stop("FUEL EMPTY - LIMIT INVALID", t)
      elseif abort then stop("MANUAL", t)
      elseif spin > C.HARD_SPIN then stop("HARD SPIN", t)
      elseif stage ~= "RUN" and stageT > C.SETUP_TIMEOUT then stop("SETUP TIMEOUT " .. stage, t)
      elseif t > C.MAX_TEST_SECONDS then stop("TIME LIMIT", t)
      else
        local violation
        if math.abs(crossTrack) > C.MAX_CROSS then violation = "CROSS TRACK"
        elseif spin > C.MAX_SPIN then violation = "SPIN"
        elseif err > C.MAX_ATT_ERROR then violation = "ATTITUDE"
        elseif (stage == "LEVEL" or stage == "RUN") and math.abs(altError) > C.MAX_ALT_ERROR then violation = "ALTITUDE"
        elseif stage == "RUN" and herror > C.MAX_HEADING_ERROR then violation = "HEADING" end
        bad = violation and bad + dt or 0
        if bad >= C.BAD_SECONDS then stop(violation, t) end
      end
    end
    if stage == "CLIMB" then
      local targetVy = math.min(C.CLIMB_SPEED, math.sqrt(2 * 2 * math.max(ground + C.CLIMB - pos.y, 0)))
      throttle = quantize((C.GRAV + C.CLIMB_K * (targetVy - vel.y)) / tacc)
      cmd = UP
      if pos.y >= ground + C.CLIMB - 3 then stage, stageT = "LEVEL", 0 note("LEVEL") end
    elseif stage == "LEVEL" or stage == "RUN" then
      local isStable = math.abs(crossTrack) < C.STABLE_CROSS and math.abs(altError) < C.STABLE_ALT and err < C.STABLE_ERROR and spin < C.STABLE_SPIN and herror < C.MAX_HEADING_ERROR
      if stage == "LEVEL" then
        calm = isStable and math.abs(vel.y) < 3 and calm + dt or 0
        if calm >= C.STABLE_WINDOW then stage, stageT, ramp = "RUN", 0, 0 note("RUN") end
      else
        hist[#hist + 1] = { t = t, v = along, stable = isStable }
        while #hist > 1 and hist[2].t <= t - C.PLATEAU_WINDOW do table.remove(hist, 1) end
        local sum, n, clean, earliest = 0, 0, true, t
        for _, row in ipairs(hist) do
          if row.t >= t - C.STABLE_WINDOW then sum, n, earliest = sum + row.v, n + 1, math.min(earliest, row.t) if not row.stable then clean = false end end
        end
        if n > 0 and t - earliest >= C.STABLE_WINDOW - 0.1 and clean and sum / n > stableMean then
          stableMean, stableSpeed, stableAt, stableSteps = sum / n, along, t, steps
        end
        ramp = ramp + dt
        local windowStable = n > 0 and t - earliest >= C.STABLE_WINDOW - 0.1 and clean
        if ramp >= C.STEP_SECONDS and windowStable and steps < 15 then
          steps, ramp = steps + 1, 0
          note(string.format("STEP %d speed %.2f stable_mean %.2f", steps, along, stableMean))
          save("RUNNING")
        end
        local old = hist[1]
        local lowSpeed, highSpeed = along, along
        for _, row in ipairs(hist) do lowSpeed, highSpeed = math.min(lowSpeed, row.v), math.max(highSpeed, row.v) end
        local flat = steps == 15 and old and t - old.t >= C.PLATEAU_WINDOW - 0.1 and highSpeed - lowSpeed < C.PLATEAU_ACCEL * C.PLATEAU_WINDOW
        plateau = flat and plateau + dt or 0
        if ramp >= C.STEP_WAIT_MAX and steps < 15 and not windowStable then stop("STABILITY LIMIT", t)
        elseif plateau >= C.PLATEAU_SECONDS then stop("FULL THROTTLE PLATEAU", t) end
      end
      if not reason then
        throttle = steps / 15
        if math.abs(altError) < C.MAX_ALT_ERROR then bias = clamp(bias - C.ALT_I * altError * dt, -C.ALT_I_MAX, C.ALT_I_MAX) end
        local ay = clamp(-C.ALT_K * altError - C.VY_K * vel.y, -C.ALT_ACC, C.ALT_ACC)
        local wanted = math.deg(math.asin(clamp((C.GRAV + ay + bias) / (tacc * throttle), -1, 1)))
        pitch = pitch + clamp(wanted - pitch, -C.PITCH_RATE * dt, C.PITCH_RATE * dt)
        if stage == "LEVEL" then throttle = quantize((C.GRAV + ay + bias) / (tacc * math.max(nose.y, 0.3))) end
        local correction = clamp((-C.TRACK_K * crossTrack - C.TRACK_D * sideV) / math.max(math.abs(along), C.HEADING_MIN_SPEED), -math.tan(math.rad(C.TRACK_ANGLE)), math.tan(math.rad(C.TRACK_ANGLE)))
        local f = unit(add(forward, mul(right, correction)))
        cmd = add(mul(f, math.cos(math.rad(pitch))), mul(UP, math.sin(math.rad(pitch))))
      end
    end
    if reason then
      local braking = clamp(C.RECOVERY_BRAKE * hspeed, 0, C.GRAV * math.tan(math.rad(C.RECOVERY_TILT)))
      local desired = hspeed > 0.1 and { x = -vel.x / hspeed * braking, y = C.GRAV, z = -vel.z / hspeed * braking } or UP
      cmd = turnToward(cmd, unit(desired), math.rad(C.RECOVERY_RATE) * dt)
      local targetVy = hspeed < 5 and -C.DESCENT_SPEED or 0
      local lift = C.GRAV + clamp(C.CLIMB_K * (targetVy - vel.y), -4, 6)
      throttle = quantize(lift / (tacc * math.max(nose.y, 0.3)))
      if vel.y > 3 then throttle = quantize(0.8 * C.GRAV / tacc) end
      local touching = pos.y < ground + C.LAND_HEIGHT and math.abs(vel.y) < 0.3 and hspeed < 1 and stageT > 5
      contact = touching and contact + dt or 0
      if contact >= C.LAND_SECONDS then
        throttle, gx, gy, landed = 0, 0, 0, true
        tick()
        local outcome = nose.y > math.cos(math.rad(C.RECOVERY_TILT)) and "LANDED" or "GROUND CONTACT - TILTED"
        note(outcome) save(outcome)
        log.close() log = nil
        return
      end
    end
    gx, gy = steer(cmd, w, u, om, dt, throttle)
    if now >= nextLog then
      log.writeLine(string.format("%.2f %s %d %.2f %.2f %.2f %.2f %.2f %.2f %.2f %+.2f %+.2f %+.2f %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f %.2f %.2f %+.3f %+.3f %+.3f %+.3f %+.3f %s %s %.0f %s",
        t, stage, math.floor(throttle * 15 + 0.5), along, speed, pos.x, pos.y, pos.z, crossTrack, altError, vel.x, vel.y, vel.z,
        nose.x, nose.y, nose.z, cmd.x, cmd.y, cmd.z, err, herror, om.x, om.y, om.z, gx, gy,
        actualX and string.format("%+.3f", actualX) or "NA", actualY and string.format("%+.3f", actualY) or "NA", elapsed * 1000, fuelAmount and tostring(fuelAmount) or "NA"))
      log.flush()
      nextLog = now + C.LOG_DT
    end
  end
end
local function keysIn()
  while true do
    local event, key = os.pullEventRaw()
    if event == "terminate" or (event == "key" and key == keys.backspace) then abort = true end
  end
end
os.pullEvent = os.pullEventRaw or os.pullEvent
local ok, err = pcall(parallel.waitForAny, run, keysIn)
if not ok then
  reason = reason or ("ERROR " .. tostring(err))
  if landed or not ground or not pos or pos.y <= ground + C.LAND_HEIGHT then
    throttle, gx, gy = 0, 0, 0
  else
    throttle, gx, gy = clamp(C.GRAV / tacc, 1 / 15, 1), 0, 0
  end
  pcall(function() parallel.waitForAll(function() eng.setThrustNormalized(throttle) end, function() eng.setVector(gx, gy) end) end)
  save("ERROR - " .. tostring(err))
  if log then log.close() end
  print("ERROR " .. tostring(err))
  if not landed and ground and pos and pos.y > ground + C.LAND_HEIGHT then print("Telemetry/control failed. Engine left at hover command; manual takeover needed.") end
end

os.pullEvent = originalPullEvent
