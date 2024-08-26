

local gcs_send = require("gcs_send_funcfactory")("GRC")
local wrap_angle = require("wrap_angle_obj")
local switch_exec_updatefactory = require("switch_exec_updatefactory")


local GUIDING_TIME_MS = 300

local PLANE_MODE_GUIDED        = 15

local MAV_CMD_GUIDED_CHANGE_SPEED = 43000
local MAV_CMD_GUIDED_CHANGE_ALTITUDE = 43001
local MAV_CMD_GUIDED_CHANGE_HEADING = 43002

local HEADING_TYPE_COURSE_OVER_GROUND = 0
local HEADING_TYPE_HEADING = 1
local SPEED_TYPE_AIRSPEED = 0
local MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
local MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

local function find_channel(selector, attribute)
    local slider = rc:find_channel_for_option(selector)
    if not slider then
        gcs_send(string.format("Error: no RC channel for selector %i to control %s", selector, attribute))
    end
    return slider
end

local speed_slider = find_channel(301, "speed")
local altitude_slider = find_channel(302, "altitude")
local curvature_slider = find_channel(301, "curvature")
if not speed_slider or not altitude_slider or not curvature_slider then
    return
end

local roll_limit_deg = Parameter("ROLL_LIMIT_DEG"):get()
if not roll_limit_deg then
    gcs_send("Cound not find parameter ROLL_LIMIT_DEG")
    return
end
-- Calculate a max curvature that is reasonable for a speed of 25 mps
local lateral_acceleration_max = 9.8 * math.atan(math.rad(roll_limit_deg))

local function StateCurrent(state_base)

    local loc_cur = ahrs:get_location()
    local vel_cur_vmps = ahrs:get_velocity_NED()
    if not loc_cur or not vel_cur_vmps then
        gcs_send("Error: cannot get location.")
        return nil
    end
    loc_cur:change_alt_frame(0)

    local vel_cur_2mps = Vector2f()
    vel_cur_2mps:x(vel_cur_vmps:x())
    vel_cur_2mps:y(vel_cur_vmps:y())

    local time_cur = millis():tofloat() * 0.001

    return {
        vel_bearing = function() return vel_cur_2mps:angle() end,
        speed = function() return vel_cur_2mps:length() end,
        time = function() return time_cur end,
        loc = function() return loc_cur:copy() end,
        duration = function() return time_cur - state_base.time() end,
    }
end


local function Guider()

    local state_start = StateCurrent()
    if not state_start then
        return nil
    end

    -- All the failure modes have passed so we can enable guiding. A question is
    -- should we have set a target wp before enabling guiding. For now no but this needs checking
    local saved_mode = vehicle:get_mode()
    vehicle:set_mode(PLANE_MODE_GUIDED)

    local function finish()
        vehicle:set_mode(saved_mode)
    end

    local state_last = state_start

    return function(abort)

        if abort then
            finish()
            return false
        end

        if vehicle:get_mode() ~= PLANE_MODE_GUIDED then
            gcs_send("Terminating because mode ~= GUIDE")
            return false
        end

        local state_now = StateCurrent(state_start)
        if not state_now then
            return false
        end

        -- Set speed
        local speed_desired = ({22, 25, 28})[speed_slider:get_aux_switch_pos()+1]
        speed_desired = 22

        -- p1 = type (SPEED_TYPE_AIRSPEED)
        -- p2 = airspeed
        -- p3 = max airspeed accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_SPEED, {
            p1 = SPEED_TYPE_AIRSPEED,
            p2 = speed_desired,
            p3 = 20,
            })

        -- Set altitude
        local altitude_desired = ({80, 100, 120})[altitude_slider:get_aux_switch_pos()+1]
        altitude_desired = 120

        -- frame = type (MAV_FRAME_GLOBAL_RELATIVE_ALT)
        -- z = altitude 
        -- p3 = max accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_ALTITUDE, {
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT,
            z = altitude_desired,
            p3 = 100,
            })

        -- Set curvature
        local speed2 = state_now.speed() * state_now.speed()
        local curvature_max = lateral_acceleration_max / speed2

        local curvature_desired  = curvature_slider:norm_input()
        local curvature_direction = 1   -- clockwise
        if curvature_desired < 0 then
            curvature_direction = -1    -- counter clockwise
            curvature_desired = - curvature_desired
        end
        curvature_desired = curvature_desired * curvature_desired  -- add expo
        curvature_desired = curvature_desired * curvature_max
        local lateral_acceleration_desired = curvature_desired * speed2

        local bearing_new = 90 * curvature_direction
        bearing_new = wrap_angle.deg_360(math.deg(state_now.vel_bearing()) + bearing_new)

        -- p1 = type (GUIDED_HEADING_NONE=0, GUIDED_HEADING_COG=1, GUIDED_HEADING_HEADING=2)
        -- p2 = heading in degrees
        -- p3 = max accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_HEADING, {
            p1 = HEADING_TYPE_COURSE_OVER_GROUND,
            p2 = bearing_new,
            p3 = lateral_acceleration_desired,
            })

        gcs_send(string.format("spddes %.1f, altdes %.0f, curdes %.3f", 
            speed_desired, altitude_desired, curvature_desired))

        return true
    end
end




return (function()
    local r, d = switch_exec_updatefactory("guide_sah_driver", Guider, GUIDING_TIME_MS, 300, gcs_send)

    gcs_send("Loaded guide_sah_driver.lua")

    return r, d
end)()