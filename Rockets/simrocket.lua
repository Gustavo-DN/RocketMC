local TX, TZ, TY = tonumber(os.getenv("TX") or 300), tonumber(os.getenv("TZ") or 0), tonumber(os.getenv("TY") or 0)
local MODE = os.getenv("MODE") or "WATER"
local GROUND = tonumber(os.getenv("GROUND") or 0)
local SY = tonumber(os.getenv("SY") or 300)
local SV = tonumber(os.getenv("SV") or 40)
local SACC = tonumber(os.getenv("SACC") or 15)
local REL = tonumber(os.getenv("REL") or 2)
local FREE_CARRIER = os.getenv("COAST")
local PLANE = os.getenv("PLANE") ~= nil
local MASS = tonumber(os.getenv("MASS") or (PLANE and 4.5 or 5.5))
local MP = tonumber(os.getenv("MP") or -3.47)
local MY = tonumber(os.getenv("MY") or -3.84)
local KW = tonumber(os.getenv("KW") or 0.5)
local DAMP = tonumber(os.getenv("DAMP") or 0.5)
local LIFT = tonumber(os.getenv("LIFT") or 0)
local KSIDE = tonumber(os.getenv("KSIDE") or 0.5)
local LMAX = tonumber(os.getenv("LMAX") or 1e9)
local KIND = tonumber(os.getenv("KIND") or 0)
local FRIC = tonumber(os.getenv("FRIC") or 2)
local DSAT = tonumber(os.getenv("DSAT") or 0)
local DV0 = tonumber(os.getenv("DV0") or 55)
local FRICV = tonumber(os.getenv("FRICV") or 0)
local GHOLD = tonumber(os.getenv("GHOLD") or 0)
local FWD = PLANE and { x = 0, y = 0, z = 1 } or { x = 0, y = 1, z = 0 }
local INER = tonumber(os.getenv("INERTIA") or (PLANE and 3.15 or 15.57))
local TACC = tonumber(os.getenv("TACC") or 427 / MASS)
local AX = tonumber(os.getenv("AX") or 3.37) * 15.57 / INER
local AY = tonumber(os.getenv("AY") or 1.95) * 15.57 / INER
local CPL = tonumber(os.getenv("CPL") or 0.94) * 15.57 / INER
local DRAGK = tonumber(os.getenv("DRAG") or 0.1)
local KAERO = tonumber(os.getenv("KAERO") or 0)
local TQMAX = tonumber(os.getenv("TQMAX") or 1e9)
local TMAX = tonumber(os.getenv("TMAX") or 300)
local SLEW = tonumber(os.getenv("SLEW") or 1e9)
local NOSE = tonumber(os.getenv("NOSE") or (PLANE and 1 or 3))
T = 0
local function qmul(a, b)
  return { w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z, x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
           y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x, z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w }
end
local function qrot(q, v)
  local p = qmul(qmul(q, { w = 0, x = v.x, y = v.y, z = v.z }), { w = q.w, x = -q.x, y = -q.y, z = -q.z })
  return { x = p.x, y = p.y, z = p.z }
end
local h0 = math.rad(tonumber(os.getenv("HEAD") or 20))
local shuttle = MODE == "SHUTTLE"
S = { pos = { x = 0, y = shuttle and SY or GROUND, z = 0 }, vel = PLANE and { x = SV * math.sin(h0), y = tonumber(os.getenv("SVY") or 0), z = SV * math.cos(h0) } or { x = 0, y = shuttle and SV or 0, z = 0 }, om = { x = 0, y = 0, z = 0 },
  q = { w = math.cos(h0 / 2), x = 0, y = math.sin(h0 / 2), z = 0 }, attached = shuttle, maxTilt = 0, maxSpin = 0, maxV = 0 }
local p0 = math.rad(tonumber(os.getenv("PITCH0") or 0))
if p0 ~= 0 then S.q = qmul(S.q, { w = math.cos(-p0 / 2), x = math.sin(-p0 / 2), y = 0, z = 0 }) end
local PROT = tonumber(os.getenv("PROT") or 0)
local QOFF = { w = math.cos(-PROT * math.pi / 4), x = 0, y = math.sin(-PROT * math.pi / 4), z = 0 }
local Eng = { thr = 0, tx = 0, ty = 0, gx = 0, gy = 0 }
local function mul3(v, s) return { x = v.x * s, y = v.y * s, z = v.z * s } end
local function q15(x) return math.floor(x * 15 + 0.5) / 15 end
local function phys(dt)
  local nose = qrot(S.q, FWD)
  if S.attached then
    if T >= REL then
      S.attached = false
      S.relT = T
    else
      if not PLANE then
        local a = FREE_CARRIER and -10.5 or SACC
        S.vel.y = S.vel.y + a * dt
      end
      S.pos.x, S.pos.y, S.pos.z = S.pos.x + S.vel.x * dt, S.pos.y + S.vel.y * dt, S.pos.z + S.vel.z * dt
      return
    end
  end
  local a = math.min(1, dt / 0.23)
  local sl = SLEW * dt
  Eng.gx = Eng.gx + math.max(-sl, math.min(sl, (q15(Eng.tx) - Eng.gx) * a))
  Eng.gy = Eng.gy + math.max(-sl, math.min(sl, (q15(Eng.ty) - Eng.gy) * a))
  local tq = math.floor(Eng.thr * 15 + 1e-9) / 15
  local k = tq / (2 / 15)
  local bv = qrot({ w = S.q.w, x = -S.q.x, y = -S.q.y, z = -S.q.z }, S.vel)
  local tx, tz = KAERO * bv.z, -KAERO * bv.x
  local tm = math.sqrt(tx * tx + tz * tz)
  if tm > TQMAX then tx, tz = tx * TQMAX / tm, tz * TQMAX / tm end
  if PLANE then
    local steps = tq * 15
    local px, py = -KW * bv.y, KW * bv.x
    local pm = math.sqrt(px * px + py * py)
    if pm > TQMAX then px, py = px * TQMAX / pm, py * TQMAX / pm end
    local axp = MP * Eng.gy * steps + px - DAMP * S.om.x
    if S.onGround and GHOLD > 0 then
      if axp < 0 then axp = math.min(0, axp + GHOLD) end
      if S.om.x < 0 and axp >= 0 then S.om.x = 0 end
    end
    S.onGround = false
    S.om.x = S.om.x + axp * dt
    S.om.y = S.om.y + (MY * Eng.gx * steps + py - DAMP * S.om.y) * dt
    S.om.z = S.om.z * (1 - 2 * dt)
  else
    local ax = -AY * Eng.gy * k + tx
    local az = AX * Eng.gx * k - CPL * Eng.gy * k + tz
    S.om.x, S.om.z = S.om.x + ax * dt, S.om.z + az * dt
    S.om.x, S.om.y, S.om.z = S.om.x * (1 - 0.2 * dt), S.om.y, S.om.z * (1 - 0.2 * dt)
  end
  local d = qmul(S.q, { w = 1, x = 0.5 * S.om.x * dt, y = 0.5 * S.om.y * dt, z = 0.5 * S.om.z * dt })
  local n = math.sqrt(d.w^2 + d.x^2 + d.y^2 + d.z^2)
  S.q = { w = d.w / n, x = d.x / n, y = d.y / n, z = d.z / n }
  local f = qrot(S.q, mul3(FWD, TACC * tq))
  if PLANE then
    local ly = -(LIFT + KSIDE) * bv.y
    if ly > LMAX then ly = LMAX elseif ly < -LMAX then ly = -LMAX end
    local l = qrot(S.q, { x = -KSIDE * bv.x, y = ly, z = 0 })
    local sp = math.sqrt(S.vel.x ^ 2 + S.vel.y ^ 2 + S.vel.z ^ 2)
    if DSAT > 0 and sp > 1 then
      local dd = DSAT * (1 - math.exp(-sp / DV0)) / sp
      l = { x = l.x - dd * S.vel.x, y = l.y - dd * S.vel.y, z = l.z - dd * S.vel.z }
    end
    if KIND > 0 and sp > 1 then
      local di = KIND * ly * ly / math.max(sp, 10) / sp
      l = { x = l.x - di * S.vel.x, y = l.y - di * S.vel.y, z = l.z - di * S.vel.z }
    end
    f = { x = f.x + l.x, y = f.y + l.y, z = f.z + l.z }
  end
  local pool = math.abs(S.pos.x - TX) < 40 and math.abs(S.pos.z - TZ) < 40
  local inWater = pool and S.pos.y < TY
  local dk = inWater and 4 or DRAGK
  S.vel.x = S.vel.x + (f.x - dk * S.vel.x) * dt
  S.vel.y = S.vel.y + (f.y - 10.5 - dk * S.vel.y) * dt
  S.vel.z = S.vel.z + (f.z - dk * S.vel.z) * dt
  local tip0 = { x = S.pos.x + nose.x * NOSE, y = S.pos.y + nose.y * NOSE, z = S.pos.z + nose.z * NOSE }
  local tipY0 = tip0.y
  S.pos.x, S.pos.y, S.pos.z = S.pos.x + S.vel.x * dt, S.pos.y + S.vel.y * dt, S.pos.z + S.vel.z * dt
  nose = qrot(S.q, FWD)
  local tip = { x = S.pos.x + nose.x * NOSE, y = S.pos.y + nose.y * NOSE, z = S.pos.z + nose.z * NOSE }
  if not S.splash and tipY0 > TY and tip.y <= TY and T > 1 then
    local f = (tip0.y - TY) / (tip0.y - tip.y)
    local hx, hz = tip0.x + (tip.x - tip0.x) * f, tip0.z + (tip.z - tip0.z) * f
    S.splash = { px = hx, pz = hz, cx = S.pos.x, cz = S.pos.z, miss = math.sqrt((hx - TX)^2 + (hz - TZ)^2), v = math.sqrt(S.vel.x^2 + S.vel.y^2 + S.vel.z^2), t = T,
      ang = math.deg(math.asin(math.max(-1, math.min(1, -S.vel.y / math.max(1e-6, math.sqrt(S.vel.x^2 + S.vel.y^2 + S.vel.z^2)))))) }
  end
  if S.pos.y > GROUND + 1 or shuttle then S.airborne = true end
  local floor = pool and TY - 6 or GROUND
  if S.pos.y < floor then
    S.pos.y = floor
    if PLANE and not pool then
      if S.vel.y < -8 and T > 1 then S.crash = S.crash or { x = S.pos.x, z = S.pos.z, t = T, vy = S.vel.y } end
      if S.vel.y < 0 then
        S.touch = S.touch or { t = T, vy = S.vel.y, v = math.sqrt(S.vel.x ^ 2 + S.vel.z ^ 2) }
        S.vel.y = 0
      end
      local hs = math.sqrt(S.vel.x ^ 2 + S.vel.z ^ 2)
      if hs > 0 then
        local k = math.max(0, hs - (FRIC + FRICV * hs) * dt) / hs
        S.vel.x, S.vel.z = S.vel.x * k, S.vel.z * k
      end
      S.om.x, S.om.z = S.om.x * 0.5, S.om.z * 0.5
      S.onGround = true
      local nw = qrot(S.q, FWD)
      if nw.y < 0 and math.abs(nw.y) > 0.05 then S.om.x = S.om.x + 2 * dt end
    else
      if S.airborne and T > 1 and not pool then S.crash = S.crash or { x = S.pos.x, z = S.pos.z, t = T } end
      S.vel = { x = 0, y = 0, z = 0 }
      S.om = { x = 0, y = 0, z = 0 }
    end
  end
  local sp = math.sqrt(S.vel.x^2 + S.vel.y^2 + S.vel.z^2)
  S.maxV = math.max(S.maxV, sp)
  if T > 1 then
    S.maxSpin = math.max(S.maxSpin, math.sqrt(S.om.x^2 + S.om.z^2))
  end
end
local function advance(dt)
  if T > TMAX then error("SIM TIME LIMIT", 0) end
  local n = math.floor(dt / 0.05 + 0.5)
  for _ = 1, math.max(n, 1) do if APPLYQ then APPLYQ() end phys(0.05) T = T + 0.05 end
end
sleep = function(s) advance(s) end
os.clock = function() return T end
os.epoch = function() return math.floor(T * 1000 + 0.5) end
os.date = function() return "sim" end
os.pullEvent = function() return "timer", 1 end
os.startTimer = function() return 1 end
os.queueEvent = function() end
parallel = { waitForAll = function(...) for _, f in ipairs({ ... }) do f() end advance(0.05) end,
  waitForAny = function(f) f() end }
local LAG = tonumber(os.getenv("LAG") or 0)
local Q = {}
local function push(k, v)
  if LAG == 0 then Eng[k] = v return end
  Q[#Q + 1] = { t = T + LAG * 0.05, k = k, v = v }
end
APPLYQ = function()
  local keep = {}
  for _, q in ipairs(Q) do if q.t <= T + 1e-9 then Eng[q.k] = q.v else keep[#keep + 1] = q end end
  Q = keep
end
local engine = { setThrustNormalized = function(v) push("thr", v) end, setVector = function(x, y) push("tx", x) push("ty", y) end, pullFluid = function() end, getVectorX = function() return Eng.gx end, getVectorY = function() return Eng.gy end }
peripheral = { wrap = function(s) if s == "bottom" or s == "back" then return engine end end,
  getNames = function() return { "bottom" } end, hasType = function(n, t) return n == "bottom" and t == "liquid_vector_thruster" end,
  call = function() return { { amount = 1000 } } end }
sublevel = {
  isInPlotGrid = function() return true end,
  getLogicalPose = function()
    local q = PROT ~= 0 and qmul(S.q, QOFF) or S.q
    return { position = { x = S.pos.x, y = S.pos.y, z = S.pos.z }, orientation = { w = q.w, x = q.x, y = q.y, z = q.z } }
  end,
  getLinearVelocity = function() return { x = S.vel.x, y = S.vel.y, z = S.vel.z } end,
  getAngularVelocity = function()
    if PROT ~= 0 then return qrot({ w = QOFF.w, x = -QOFF.x, y = -QOFF.y, z = -QOFF.z }, S.om) end
    return { x = S.om.x, y = S.om.y, z = S.om.z }
  end,
  getInertiaTensor = function()
    if PLANE then
      local a, b = 3.15, 2.25
      if PROT ~= 0 then a, b = b, a end
      return { { a, 0, 0 }, { 0, 4.65, 0 }, { 0, 0, b }, rows = 3, columns = 3 }
    end
    return { { INER, 0, 0 }, { 0, 0.92, 0 }, { 0, 0, INER }, rows = 3, columns = 3 }
  end,
  getMass = function() return (S.attached and not os.getenv("NOMASS")) and 60 or MASS end,
}
redstone = { setOutput = function(side, v) if os.getenv("DBG") then io.stderr:write(string.format("REDSTONE %s %s T=%.2f\n", side, tostring(v), T)) end end, getSides = function() return { "top", "bottom", "left", "right", "front", "back" } end, getInput = function(side) return os.getenv("RS") ~= nil and side == "top" and T >= REL end }
fs = { open = function(p, m)
  local f = io.open("sim_" .. p, m == "a" and "a" or (m == "r" and "r" or "w"))
  if not f then return nil end
  return { writeLine = function(s) f:write(s, "\n") end, write = function(s) f:write(s) end, flush = function() f:flush() end, close = function() f:close() end, readAll = function() return f:read("*a") end }
end, exists = function() return false end }
textutils = { serialize = function() return "{}" end, unserialize = function() return nil end }
term = setmetatable({ getSize = function() return 51, 19 end, write = function(s) if tostring(s):find("lua:") or tostring(s):find("attempt") then io.stderr:write("TERMERR: " .. tostring(s) .. "\n") end end }, { __index = function() return function() end end })
colors = setmetatable({}, { __index = function() return 1 end })
keys = { backspace = 14, enter = 28, numPadEnter = 156 }
table.unpack = table.unpack or unpack
local runtimeError = false
local real = print
DBGW = function(s) io.stderr:write(s .. "\n") end
print = function(s) runtimeError = true io.stderr:write("PRINT: " .. tostring(s) .. "\n") end
local REMOTE = { mission = { mode = MODE, x = TX, y = TY, z = TZ, climb = tonumber(os.getenv("CLIMB") or 50) }, state = {},
  report = function() return false end, plan = function() end, launchIn = function() return 0 end,
  result = function(r, sp, t) RESULT = { r = r, sp = sp, t = t } end }
local ok, err = pcall(loadfile(arg[1]), REMOTE)
local sp = S.splash
if os.getenv("DBG") and sp then real(string.format("SPLASH tip %.1f %.1f center %.1f %.1f", sp.px, sp.pz, sp.cx, sp.cz)) end
real(string.format("%s %s ok=%s%s end_t=%.1f rel_t=%s splash_t=%s miss=%s splash_v=%s path_angle=%s crash=%s maxV=%.0f maxSpin=%.2f result_miss=%s",
  MODE, arg[1], tostring(ok), ok and "" or (" " .. tostring(err)), T, S.relT and string.format("%.1f", S.relT) or "-",
  sp and string.format("%.1f", sp.t) or "-", sp and string.format("%.1f", sp.miss) or "-", sp and string.format("%.0f", sp.v) or "-",
  sp and string.format("%.0f", sp.ang) or "-", S.crash and string.format("(%.0f,%.0f)@%.1f", S.crash.x, S.crash.z, S.crash.t) or "-",
  S.maxV, S.maxSpin, RESULT and string.format("%.1f L%+.1f R%+.1f", RESULT.r.total, RESULT.r.long, RESULT.r.right) or "-"))

if os.getenv("CHECK_FLIGHT") then
  local maxMiss = tonumber(os.getenv("MAX_MISS") or 20)
  if not ok or runtimeError or not RESULT or not S.splash or S.crash or S.splash.miss > maxMiss then os.exit(1) end
end
