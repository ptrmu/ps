
local PLANE_MODE_MANUAL        = 0
local PLANE_MODE_CIRCLE        = 1
local PLANE_MODE_STABILIZE     = 2
local PLANE_MODE_TRAINING      = 3
local PLANE_MODE_ACRO          = 4
local PLANE_MODE_FLY_BY_WIRE_A = 5
local PLANE_MODE_FLY_BY_WIRE_B = 6
local PLANE_MODE_CRUISE        = 7
local PLANE_MODE_AUTOTUNE      = 8
local PLANE_MODE_AUTO          = 10
local PLANE_MODE_RTL           = 11
local PLANE_MODE_LOITER        = 12
local PLANE_MODE_TAKEOFF       = 13
local PLANE_MODE_AVOID_ADSB    = 14
local PLANE_MODE_GUIDED        = 15
local PLANE_MODE_INITIALISING  = 16
local PLANE_MODE_THERMAL       = 24

local wp_radius = 2
local target_alt_above_home = 10
local copter_guided_mode_num = 4
local copter_land_mode_num = 9
local sent_target = false

local pushed_mode = 0

-- the main update function that performs a simplified version of RTL
function update()
  if not arming:is_armed() then -- reset state when disarmed
    sent_target = false
  else
    pwm7 = rc:get_pwm(7)
    gcs:send_text(0, string.format("rc7 pwm %d", pwm7))
    if pwm7 and pwm7 > 1800 then                        -- check if RC input 7 has moved high
      local mode = vehicle:get_mode()                   -- get current mode
      if not sent_target then                           -- if we haven't sent the target yet
        if not (mode == PLANE_MODE_GUIDED) then    -- change to guided mode
          vehicle:set_mode(PLANE_MODE_GUIDED)
        else
          local above_home = ahrs:get_home()            -- get home location
          if above_home then
            above_home:alt(above_home:alt() + (target_alt_above_home * 100))
            sent_target = vehicle:set_target_location(above_home)   -- set target above home
          end
        end
      end
    end
  end

  return update, 1000
end

function guided()
  gcs:send_text(0, string.format("state guided"))
  local pwm7 = rc:get_pwm(7)
  if pwm7 < 1800 then                        -- check if RC input 7 has moved high
    return set_other()
  end
  return guided, 1000
end

function other()
  gcs:send_text(0, string.format("state other"))
  local pwm7 = rc:get_pwm(7)
  if pwm7 and pwm7 > 1800 then                        -- check if RC input 7 has moved high
    return set_guided()
  end
  return other, 1000
end

function set_guided()
  gcs:send_text(0, string.format("set guided mode", pushed_mode))
  pushed_mode = vehicle:get_mode()
  vehicle:set_mode(PLANE_MODE_GUIDED)
  local above_home = ahrs:get_home()            -- get home location
  if above_home then
    above_home:alt(above_home:alt() + (target_alt_above_home * 100))
    sent_target = vehicle:set_target_location(above_home)   -- set target above home
  end
  return guided()
end

function set_other()
  gcs:send_text(0, string.format("set other mode %d", pushed_mode))
  vehicle:set_mode(pushed_mode)
  return other()
end

gcs:send_text(0, string.format("Loaded set_target_location.lua"))

return other()
