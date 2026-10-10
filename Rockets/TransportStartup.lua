local pull = os.pullEvent
os.pullEvent = os.pullEventRaw
settings.set("shell.allow_disk_startup", false)
settings.save()
while true do
  if fs.exists(".transport.exit") then fs.delete(".transport.exit") end
  if not fs.exists("Transport.lua") then
    print("Transport.lua is not installed.")
    break
  end
  shell.run("Transport.lua")
  if fs.exists(".transport.exit") then
    fs.delete(".transport.exit")
    break
  end
  sleep(1)
end
os.pullEvent = pull
