T = 0
local TX, TZ, TY = tonumber(os.getenv("TX") or 300), tonumber(os.getenv("TZ") or 0), tonumber(os.getenv("TY") or -58)
local MASS = tonumber(os.getenv("MASS") or 64.06)
local TACC = 31.6 * 64.06 / MASS
local PAD = -53.3
local function qmul(a, b)
  return { w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z, x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
           y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x, z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w }
end
local function qrot(q, v)
  local p = qmul(qmul(q, { w = 0, x = v.x, y = v.y, z = v.z }), { w = q.w, x = -q.x, y = -q.y, z = -q.z })
  return { x = p.x, y = p.y, z = p.z }
end
local h0 = math.rad(37)
S = { pos = { x = 0, y = PAD, z = 0 }, vel = { x = 0, y = 0, z = 0 }, om = { x = 0, y = 0, z = 0 }, q = { w = math.cos(h0/2), x = 0, y = math.sin(h0/2), z = 0 }, maxTiltLow = 0 }
local E = {}
for i = 1, 4 do E[i] = { thr = 0, tx = 0, ty = 0, gx = 0, gy = 0 } end
local SX, SY = { 1, 1, -1, -1 }, { 1, -1, 1, -1 }
local GXZ = { 0.36, 0.44, 0.44, 0.44 }
local K_AERO = tonumber(os.getenv("KAERO") or 0.03)
local function phys(dt)
  local ax, ay, az, fx, fy, fz = 0, 0, 0, 0, 0, 0
  for i = 1, 4 do
    local e = E[i]
    local a = math.min(1, dt / 0.23)
    e.gx = e.gx + (e.tx - e.gx) * a
    e.gy = e.gy + (e.ty - e.gy) * a
    local tq = math.floor(e.thr * 15 + 1e-9) / 15
    local k = tq / 0.335 * TACC / 31.6
    az = az + GXZ[i] * e.gx * k
    ax = ax - 0.41 * e.gy * k
    ay = ay + (0.55 * SX[i] * e.gx + 0.52 * SY[i] * e.gy) * k
    fy = fy + TACC / 4 * tq
    fx = fx + 1.75 * e.gx * k
    fz = fz + 1.80 * e.gy * k
  end
  local bv = qrot({ w = S.q.w, x = -S.q.x, y = -S.q.y, z = -S.q.z }, S.vel)
  local TQ = tonumber(os.getenv("TQMAX") or 1.2)
  local tx_, tz_ = K_AERO * bv.z, -K_AERO * bv.x
  local tm = math.sqrt(tx_ * tx_ + tz_ * tz_)
  if tm > TQ then tx_, tz_ = tx_ * TQ / tm, tz_ * TQ / tm end
  ax = ax + tx_
  az = az + tz_
  S.om.x, S.om.y, S.om.z = S.om.x + ax * dt, S.om.y + ay * dt, S.om.z + az * dt
  local d = qmul(S.q, { w = 1, x = 0.5 * S.om.x * dt, y = 0.5 * S.om.y * dt, z = 0.5 * S.om.z * dt })
  local n = math.sqrt(d.w^2 + d.x^2 + d.y^2 + d.z^2)
  S.q = { w = d.w / n, x = d.x / n, y = d.y / n, z = d.z / n }
  local KL = tonumber(os.getenv("KLAT") or 0.5)
  local LM = tonumber(os.getenv("LATMAX") or 12)
  local lx_, lz_ = -KL * bv.x, -KL * bv.z
  local lm = math.sqrt(lx_ * lx_ + lz_ * lz_)
  if lm > LM then lx_, lz_ = lx_ * LM / lm, lz_ * LM / lm end
  fx, fy, fz = fx + lx_, fy - tonumber(os.getenv("CAX") or 0.12) * bv.y, fz + lz_
  local fw = qrot(S.q, { x = fx, y = fy, z = fz })
  S.vel.x = S.vel.x + fw.x * dt
  S.vel.y = S.vel.y + (fw.y - 10.5) * dt
  S.vel.z = S.vel.z + fw.z * dt
  S.maxY = math.max(S.maxY or -999, S.pos.y)
  S.maxV = math.max(S.maxV or 0, math.sqrt(S.vel.x^2 + S.vel.y^2 + S.vel.z^2))
  S.pos.x, S.pos.y, S.pos.z = S.pos.x + S.vel.x * dt, S.pos.y + S.vel.y * dt, S.pos.z + S.vel.z * dt
  local nose = qrot(S.q, { x = 0, y = 1, z = 0 })
  local floor = PAD
  if math.abs(S.pos.x - TX) < 8 and math.abs(S.pos.z - TZ) < 8 then floor = TY + math.max(4.8 * nose.y, -12.7 * nose.y) end
  local tilt = math.deg(math.acos(math.max(-1, math.min(1, nose.y))))
  if S.pos.y - floor < 15 and T > 20 then S.maxTiltLow = math.max(S.maxTiltLow, tilt) end
  if S.pos.y < floor then
    S.pos.y = floor
    if not S.landed and T > 20 then
      local tip = nose.y < 0 and 12.7 or -4.8
      S.impV = math.sqrt(S.vel.x^2 + S.vel.y^2 + S.vel.z^2)
      S.tipMiss = math.sqrt((S.pos.x + nose.x * tip - TX)^2 + (S.pos.z + nose.z * tip - TZ)^2)
      S.landTilt = math.deg(math.acos(math.max(-1, math.min(1, nose.y))))
    end
    if S.vel.y < 0 then S.vel = { x = 0, y = 0, z = 0 } S.om = { x = S.om.x * 0.5, y = 0, z = S.om.z * 0.5 } end
    if T > 20 then S.landed = S.landed or T end
  end
end
local TR = io.open("sim_trace.txt", "w")
local nextTr = 0
local function trace()
  if T >= nextTr then
    local n = qrot(S.q, { x = 0, y = 1, z = 0 })
    TR:write(string.format("%.2f %.1f %.1f %.1f %.2f %.2f %.2f %.3f %.3f %.3f\n", T, S.pos.x, S.pos.y, S.pos.z, S.vel.x, S.vel.y, S.vel.z, n.x, n.y, n.z))
    nextTr = T + 0.25
  end
end
local function advance(dt)
  trace()
  if T > tonumber(os.getenv("TMAX") or 300) then error("SIM TIME LIMIT", 0) end
  local n = math.floor(dt / 0.05 + 0.5)
  for _ = 1, math.max(n, 1) do phys(0.05) T = T + 0.05 end
end
os.clock = function() return T end
os.epoch = function() return math.floor(T * 1000 + 0.5) end
sleep = function(s) advance(s) end
os.pullEvent = function() return "timer", 1 end
os.startTimer = function() return 1 end
os.queueEvent = function() end
parallel = { waitForAll = function(...) for _, f in ipairs({ ... }) do f() end advance(0.05) end,
  waitForAny = function(f) f() end }
local names = { "liquid_vector_thruster_0", "liquid_vector_thruster_1", "liquid_vector_thruster_2", "liquid_vector_thruster_3", "tank" }
peripheral = { getNames = function() return names end,
  hasType = function(n, t) if t == "liquid_vector_thruster" then return n:find("thruster") ~= nil end return t == "fluid_storage" end,
  call = function() return { { amount = 900 } } end,
  wrap = function(n)
    local i = ({ [0] = 4, [1] = 3, [2] = 1, [3] = 2 })[tonumber(n:match("_(%d)$"))]
    local e = E[i]
    return { setThrustNormalized = function(v) e.thr = v end, setVector = function(x, y) e.tx, e.ty = x, y end,
      pullFluid = function() end }
  end }
sublevel = {
  isInPlotGrid = function() return true end,
  getLogicalPose = function() return { position = { x = S.pos.x, y = S.pos.y, z = S.pos.z }, orientation = { w = S.q.w, x = S.q.x, y = S.q.y, z = S.q.z } } end,
  getLinearVelocity = function() return { x = S.vel.x, y = S.vel.y, z = S.vel.z } end,
  getAngularVelocity = function() return { x = S.om.x, y = S.om.y, z = S.om.z } end,
  getMass = function() return MASS end,
  getInertiaTensor = function() return { { 1144.4, 0, 0 }, { 0, 61.5, 0 }, { 0, 0, 1144.4 } } end,
}
fs = { open = function(p, m)
  local f = io.open("sim_" .. p, m == "a" and "a" or (m == "r" and "r" or "w"))
  if not f then return nil end
  return { writeLine = function(s) f:write(s, "\n") end, write = function(s) f:write(s) end, flush = function() f:flush() end, close = function() f:close() end, readAll = function() return f:read("*a") end }
end, exists = function() return false end }
textutils = { serialize = function() return "{}" end, unserialize = function() return nil end }
term = setmetatable({ getSize = function() return 51, 19 end, write = function(s) if tostring(s):find("lua:") or tostring(s):find("attempt") then io.stderr:write("TERMERR: " .. tostring(s) .. "\n") end end }, { __index = function() return function() end end })
colors = setmetatable({}, { __index = function() return 1 end })
keys = { backspace = 14, enter = 28 }
redstone = { setOutput = function() end }
table.unpack = table.unpack or unpack
RESULT = nil
local REMOTE = { mission = { mode = os.getenv("MODE") or "NET", x = TX, y = TY, z = TZ, climb = tonumber(os.getenv("CLIMB") or 100) }, state = {},
  report = function() return false end, plan = function() end, launchIn = function() return 0 end,
  result = function(r, sp, t) RESULT = { r = r, sp = sp, t = t } end }
local ok, err = pcall(loadfile(arg[1]), REMOTE)
local nose = qrot(S.q, { x = 0, y = 1, z = 0 })
print(string.format("%s  ok=%s %s  landed_t=%s  end_t=%.1f  pos=(%.1f, %.2f, %.1f) miss_xz=%.1f tilt_end=%.1f maxTiltLow=%.1f maxY=%.0f maxV=%.0f tip_xz=%.1f impV=%.0f landTilt=%.0f",
  arg[1], tostring(ok), ok and "" or tostring(err), tostring(S.landed and math.floor(S.landed)), T, S.pos.x, S.pos.y, S.pos.z,
  math.sqrt((S.pos.x - TX)^2 + (S.pos.z - TZ)^2), math.deg(math.acos(nose.y)), S.maxTiltLow, S.maxY or 0, S.maxV or 0, S.tipMiss or -1, S.impV or -1, S.landTilt or -1))
