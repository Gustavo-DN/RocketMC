local THR = 0.15
local ON_T = 3
local OFF_T = 2

local names = {}
for _, n in ipairs(peripheral.getNames()) do
  if peripheral.hasType(n, "liquid_vector_thruster") then names[#names + 1] = n end
end
table.sort(names)
if #names == 0 then
  print("No thrusters found on the network.")
  return
end

local eng = {}
for i, n in ipairs(names) do eng[i] = peripheral.wrap(n) end

local function allOff()
  for _, e in ipairs(eng) do
    pcall(e.setThrustNormalized, 0)
    pcall(e.setVector, 0, 0)
  end
end

local ok, err = pcall(function()
  allOff()
  term.clear()
  term.setCursorPos(1, 1)
  print("THRUSTER ID: fires one engine at a time at low power.")
  print("Watch which engine lights up. Ctrl+T stops.")
  print("")
  for i, n in ipairs(names) do
    print(string.format("%d/%d  %s  ON", i, #names, n))
    eng[i].setThrustNormalized(THR)
    sleep(ON_T)
    eng[i].setThrustNormalized(0)
    sleep(OFF_T)
  end
  print("")
  print("Done. Tell Claude which position each name was:")
  print("front, front-right, under the computer, right.")
end)
allOff()
if not ok then print(tostring(err)) end
