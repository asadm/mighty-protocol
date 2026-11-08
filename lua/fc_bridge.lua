-- Bidirectional FC link. Mounting calibration is saved by /calib-airframe.
-- UART remains owned by this script. Change the profile for PX4/Betaflight.
local fc = MAVLink.new { profile = "ardupilot", mount = "saved",
                        system_id = 1, component_id = 197 }
local last_pose, last_request = 0, -1000
local last_start_check, retry_after = -1000, 0
local start_request, startup_done = nil, false
local interval = math.floor(1000 / fc:profile().rate_hz)
local function update_startup(now)
  if start_request then
    local ok, result = Mighty.command_result(start_request)
    if ok ~= nil then
      start_request = nil
      startup_done = ok
      retry_after = now + 5000
      if not ok then print("VIO startup: " .. (result.error or result.message or "failed")) end
    end
    -- Never retry an unresolved command: it may still start VIO.
    return
  end
  if startup_done or now - last_start_check < 1000 then return end
  last_start_check = now
  local state = Mighty.app_state()
  if not state.valid then return end
  if state.vio_state ~= "off" then
    startup_done = true -- Adopt existing VIO without resetting its local frame.
  elseif now >= retry_after and not state.recording and fc:mount_ready() then
    start_request = Mighty.command_async("start_vio")
    retry_after = now + 5000
  end
end
-- Nonblocking status indication; these patterns describe Mighty, not FC readiness.
local led_patterns = {
  idle = { {true, 100}, {false, 900} },
  recording = { {true, 120}, {false, 120}, {true, 120}, {false, 120}, {true, 120}, {false, 400} },
  starting = { {true, 100}, {false, 100} },
  tracking = { {true, 1000} },
  degraded = { {true, 50}, {false, 150}, {true, 50}, {false, 750} },
  lost = { {true, 200}, {false, 200} },
}
local led_states = { off = "idle", tracking = "tracking", degraded = "degraded",
                     low_light = "degraded", lost = "lost", recovering = "starting" }
local led_mode, led_step, led_since = nil, 1, 0
local last_led_poll = -1000
local function update_led(now)
  -- LED patterns need at most 50 Hz; UART and odometry keep their own cadence.
  if now - last_led_poll < 20 then return end
  last_led_poll = now
  local state = Mighty.app_state()
  local mode = "starting"
  if state.valid then
    -- Concurrent/automatic recording must not hide VIO health.
    if state.recording and state.preview_mode then mode = "recording"
    elseif state.preview_mode then mode = "idle"
    elseif state.initializing then mode = "starting"
    else mode = led_states[state.vio_state] or "starting" end
  end
  if mode ~= led_mode then
    led_mode, led_step, led_since = mode, 1, now
    LED.write(led_patterns[mode][1][1])
  elseif now - led_since >= led_patterns[mode][led_step][2] then
    led_step = led_step % #led_patterns[mode] + 1
    led_since = now
    LED.write(led_patterns[mode][led_step][1])
  end
end
function setup()
  LED.begin()
  LED.off()
  Serial.begin(230400)
  Mighty.configure_odometry { rate_hz = fc:profile().rate_hz }
  -- Start once when ready, even while disarmed. Never stop on FC link/arm changes.
end
function loop()
  local available = math.min(Serial.available(), 256)
  if available > 0 then fc:ingest(Serial.readBytes(available)) end
  local now = millis()
  if now - last_request >= 1000 then
    last_request = now
    Serial.tryWriteRaw(fc:request_telemetry())
  end
  update_led(now)
  update_startup(now) -- Asynchronous: UART reception continues during startup.
  if now - last_pose >= interval then
    last_pose = now
    local packet = fc:odometry(Mighty.odometry())
    if packet then Serial.tryWriteRaw(packet) end
  end
end
