local PER_SIDE = "front"
local MON_SIDE = "top"
local SCROLL_TIME = 1.5

local mon = peripheral.wrap(MON_SIDE)
if not mon or not peripheral.hasType(MON_SIDE, "monitor") then
  error("No monitor on " .. MON_SIDE, 0)
end
mon.setTextScale(0.5)
local W, H = mon.getSize()

local function wrap(s, w)
  local out = {}
  while #s > w do
    out[#out + 1] = s:sub(1, w)
    s = s:sub(w + 1)
  end
  out[#out + 1] = s
  return out
end

local function scan()
  if not peripheral.isPresent(PER_SIDE) then return nil end
  local methods = peripheral.getMethods(PER_SIDE) or {}
  table.sort(methods)
  return { types = { peripheral.getType(PER_SIDE) }, methods = methods }
end

local function buildLines(p)
  local lines = {}
  for _, m in ipairs(p.methods) do
    for _, l in ipairs(wrap(m, W)) do lines[#lines + 1] = l end
  end
  return lines
end

local function put(y, text, col)
  mon.setCursorPos(1, y)
  mon.setTextColor(col)
  mon.write(text:sub(1, W))
end

local function draw(p, lines, scroll)
  mon.setBackgroundColor(colors.black)
  mon.clear()
  if not p then
    put(1, "NO PERIPHERAL", colors.red)
    put(3, "Nothing on", colors.lightGray)
    put(4, PER_SIDE, colors.lightGray)
    return
  end
  put(1, "CONNECTED", colors.lime)
  put(2, table.concat(p.types, ","), colors.yellow)
  put(3, #p.methods .. " methods", colors.lightGray)
  for i = 1, H - 3 do
    local l = lines[scroll + i]
    if l then put(3 + i, l, colors.white) end
  end
end

local function report(p)
  term.clear()
  term.setCursorPos(1, 1)
  if not p then
    print("No peripheral on " .. PER_SIDE)
    return
  end
  print("Peripheral on " .. PER_SIDE .. ": " .. table.concat(p.types, ", "))
  print(#p.methods .. " methods:")
  textutils.tabulate(p.methods)
end

local p, lines, scroll = nil, {}, 0

local function refresh()
  p = scan()
  lines = p and buildLines(p) or {}
  scroll = 0
  report(p)
  draw(p, lines, scroll)
end

refresh()
local timer = os.startTimer(SCROLL_TIME)

while true do
  local e, a = os.pullEvent()
  if (e == "peripheral" or e == "peripheral_detach") and a == PER_SIDE then
    refresh()
  elseif e == "monitor_touch" then
    refresh()
  elseif e == "monitor_resize" then
    W, H = mon.getSize()
    refresh()
  elseif e == "timer" and a == timer then
    local rows = H - 3
    if #lines > rows then
      scroll = scroll + 1
      if scroll > #lines - rows then scroll = 0 end
      draw(p, lines, scroll)
    end
    timer = os.startTimer(SCROLL_TIME)
  end
end
