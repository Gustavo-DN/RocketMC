local TX, TZ, TY = tonumber(os.getenv("TX") or 300), tonumber(os.getenv("TZ") or 0), tonumber(os.getenv("TY") or 0)
local MODE = os.getenv("MODE") or "WATER"
local GROUND = tonumber(os.getenv("GROUND") or 0)
local SY = tonumber(os.getenv("SY") or 300)
local SV = tonumber(os.getenv("SV") or 40)
local SACC = tonumber(os.getenv("SACC") or 15)
local REL = tonumber(os.getenv("REL") or 2)
local FREE_CARRIER = os.getenv("COAST")
local MASS = 5.5
local TACC = tonumber(os.getenv("TACC") or 77.6)
local AX = tonumber(os.getenv("AX") or 3.37)
local AY = tonumber(os.getenv("AY") or 1.95)
local CPL = tonumber(os.getenv("CPL") or 0.94)
local DRAGK = tonumber(os.getenv("DRAG") or 0.1)
local KAERO = tonumber(os.getenv("KAERO") or 0)
local TQMAX = tonumber(os.getenv("TQMAX") or 1e9)
local TMAX = tonumber(os.getenv("TMAX") or 300)
local NOSE = 3
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
S = { pos = { x = 0, y = shuttle and SY or GROUND, z = 0 }, vel = { x = 0, y = shuttle and SV or 0, z = 0 }, om = { x = 0, y = 0, z = 0 },
  q = { w = math.cos(h0 / 2), x = 0, y = math.sin(h0 / 2), z = 0 }, attached = shuttle, maxTilt = 0, maxSpin = 0, maxV = 0 }
local Eng = { thr = 0, tx = 0, ty = 0, gx = 0, gy = 0 }
local function q15(x) return math.floor(x * 15 + 0.5) / 15 end
local function phys(dt)
  local nose = qrot(S.q, { x = 0, y = 1, z = 0 })
  if S.attached then
    if T >= REL then
      S.attached = false
      S.relT = T
    else
      local a = FREE_CARRIER and -10.5 or SACC
      S.vel.y = S.vel.y + a * dt
      S.pos.y = S.pos.y + S.vel.y * dt
      return
    end
  end
  local a = math.min(1, dt / 0.23)
  Eng.gx = Eng.gx + (q15(Eng.tx) - Eng.gx) * a
  Eng.gy = Eng.gy + (q15(Eng.ty) - Eng.gy) * a
  local tq = math.floor(Eng.thr * 15 + 1e-9) / 15
  local k = tq / (2 / 15)
  local bv = qrot({ w = S.q.w, x = -S.q.x, y = -S.q.y, z = -S.q.z }, S.vel)
  local tx, tz = KAERO * bv.z, -KAERO * bv.x
  local tm = math.sqrt(tx * tx + tz * tz)
  if tm > TQMAX then tx, tz = tx * TQMAX / tm, tz * TQMAX / tm end
  local ax = -AY * Eng.gy * k + tx
  local az = AX * Eng.gx * k - CPL * Eng.gy * k + tz
  S.om.x, S.om.z = S.om.x + ax * dt, S.om.z + az * dt
  S.om.x, S.om.y, S.om.z = S.om.x * (1 - 0.2 * dt), S.om.y, S.om.z * (1 - 0.2 * dt)
  local d = qmul(S.q, { w = 1, x = 0.5 * S.om.x * dt, y = 0.5 * S.om.y * dt, z = 0.5 * S.om.z * dt })
  local n = math.sqrt(d.w^2 + d.x^2 + d.y^2 + d.z^2)
  S.q = { w = d.w / n, x = d.x / n, y = d.y / n, z = d.z / n }
  local f = qrot(S.q, { x = 0, y = TACC * tq, z = 0 })
  local pool = math.abs(S.pos.x - TX) < 40 and math.abs(S.pos.z - TZ) < 40
  local inWater = pool and S.pos.y < TY
  local dk = inWater and 4 or DRAGK
  S.vel.x = S.vel.x + (f.x - dk * S.vel.x) * dt
  S.vel.y = S.vel.y + (f.y - 10.5 - dk * S.vel.y) * dt
  S.vel.z = S.vel.z + (f.z - dk * S.vel.z) * dt
  local tip0 = { x = S.pos.x + nose.x * NOSE, y = S.pos.y + nose.y * NOSE, z = S.pos.z + nose.z * NOSE }
  local tipY0 = tip0.y
  S.pos.x, S.pos.y, S.pos.z = S.pos.x + S.vel.x * dt, S.pos.y + S.vel.y * dt, S.pos.z + S.vel.z * dt
  nose = qrot(S.q, { x = 0, y = 1, z = 0 })
  local tip = { x = S.pos.x + nose.x * NOSE, y = S.pos.y + nose.y * NOSE, z = S.pos.z + nose.z * NOSE }
  if not S.splash and tipY0 > TY and tip.y <= TY and T > 1 then
    local f = (tip0.y - TY) / (tip0.y - tip.y)
    local hx, hz = tip0.x + (tip.x - tip0.x) * f, tip0.z + (tip.z - tip0.z) * f
    S.splash = { px = hx, pz = hz, cx = S.pos.x, cz = S.pos.z, miss = math.sqrt((hx - TX)^2 + (hz - TZ)^2), v = math.sqrt(S.vel.x^2 + S.vel.y^2 + S.vel.z^2), t = T,
      ang = math.deg(math.asin(math.max(-1, math.min(1, -S.vel.y / math.max(1e-6, math.sqrt(S.vel.x^2 + S.vel.y^2 + S.vel.z^2)))))) }
  end
  local floor = pool and TY - 6 or GROUND
  if S.pos.y < floor then
    S.pos.y = floor
    if not S.splash and T > 1 then S.crash = S.crash or { x = S.pos.x, z = S.pos.z, t = T } end
    S.vel = { x = 0, y = 0, z = 0 }
    S.om = { x = 0, y = 0, z = 0 }
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
local engine = { setThrustNormalized = function(v) push("thr", v) end, setVector = function(x, y) push("tx", x) push("ty", y) end, pullFluid = function() end }
peripheral = { wrap = function(s) if s == "bottom" then return engine end end,
  call = function() return { { amount = 1000 } } end }
sublevel = {
  isInPlotGrid = function() return true end,
  getLogicalPose = function() return { position = { x = S.pos.x, y = S.pos.y, z = S.pos.z }, orientation = { w = S.q.w, x = S.q.x, y = S.q.y, z = S.q.z } } end,
  getLinearVelocity = function() return { x = S.vel.x, y = S.vel.y, z = S.vel.z } end,
  getAngularVelocity = function() return { x = S.om.x, y = S.om.y, z = S.om.z } end,
  getMass = function() return (S.attached and not os.getenv("NOMASS")) and 60 or MASS end,
}
redstone = { getSides = function() return { "top", "bottom", "left", "right", "front", "back" } end, getInput = function() return false end }
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
local real = print
DBGW = function(s) io.stderr:write(s .. "\n") end
print = function(s) io.stderr:write("PRINT: " .. tostring(s) .. "\n") end
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
