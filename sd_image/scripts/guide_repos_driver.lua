local gcs_send     = require("gcs_send_funcfactory")("TTR")
local wrap_angle   = require("wrap_angle_obj")
local stuf         = require("switch_trigger_update_function")(gcs_send)
local track        = require("track_obj")(gcs_send, wrap_angle)
local StateCurrent = require("ahrs_state")(gcs_send, wrap_angle).StateCurrent



local GUIDING_TIME_MS                   = 200

local PLANE_MODE_GUIDED                 = 15

local MAV_CMD_DO_REPOSITION             = 192
local MAV_CMD_GUIDED_CHANGE_SPEED       = 43000
local MAV_CMD_GUIDED_CHANGE_ALTITUDE    = 43001
local MAV_CMD_GUIDED_CHANGE_HEADING     = 43002

local HEADING_TYPE_COURSE_OVER_GROUND   = 0
local HEADING_TYPE_HEADING              = 1
local SPEED_TYPE_AIRSPEED               = 0
local MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
local MAV_FRAME_GLOBAL_RELATIVE_ALT     = 3


local function find_channel(selector, attribute)
    local slider = rc:find_channel_for_option(selector)
    if not slider then
        gcs_send(string.format("Error: no RC channel for selector %i to control %s", selector, attribute))
    end
    return slider
end

local speed_slider = find_channel(303, "speed")
local altitude_slider = find_channel(302, "altitude")
local curvature_slider = find_channel(301, "curvature")
if not speed_slider or not altitude_slider or not curvature_slider then
    return nil, 0
end

local roll_limit_deg = Parameter("ROLL_LIMIT_DEG"):get()
if not roll_limit_deg then
    gcs_send("Cound not find parameter ROLL_LIMIT_DEG")
    return nil, 0
end

-- Calculate a max curvature that is reasonable for a speed of 25 mps
local lateral_acceleration_max = 9.8 * math.atan(math.rad(roll_limit_deg))

local function Guider()
    local state_start = StateCurrent()
    local state_last = StateCurrent(state_start)

    if not state_start or not state_last then
        gcs_send("StateCurrent() failed")
        return nil
    end

    local saved_mode = vehicle:get_mode()
    vehicle:set_mode(PLANE_MODE_GUIDED)

    local function finish()
        vehicle:set_mode(saved_mode)
    end

    local count = 0


    return function(abort)
        if abort then
            finish()
            return false
        end

        if vehicle:get_mode() ~= PLANE_MODE_GUIDED then
            gcs_send("Terminating because mode ~= GUIDE")
            return false
        end

        local state_now = StateCurrent(state_start, state_last)
        if not state_now then
            gcs_send("Error Couldn't get state_now")
            return false
        end

        -- Set speed
        local speed_desired       = ({ 20, 23, 26 })[speed_slider:get_aux_switch_pos() + 1]

        -- -- p1 = type (SPEED_TYPE_AIRSPEED)
        -- -- p2 = airspeed
        -- -- p3 = max airspeed accel
        -- gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_SPEED, {
        --     p1 = SPEED_TYPE_AIRSPEED,
        --     p2 = speed_desired,
        --     p3 = 20,
        -- })

        -- Set altitude
        local altitude_desired    = ({ 80, 100, 120 })[altitude_slider:get_aux_switch_pos() + 1]

        -- -- frame = type (MAV_FRAME_GLOBAL_RELATIVE_ALT)
        -- -- z = altitude
        -- -- p3 = max accel
        -- gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_ALTITUDE, {
        --     frame = MAV_FRAME_GLOBAL_RELATIVE_ALT,
        --     z = altitude_desired,
        --     p3 = 100,
        -- })

        -- Set curvature
        local speed_sq            = state_now:speed() * state_now:speed()
        local curvature_max       = lateral_acceleration_max / speed_sq

        local curvature_input     = -curvature_slider:norm_input()                  -- Get direction correct
        local curvature_direction = 1                                               -- clockwise
        local p4                  = 0                                               -- not ccw
        if curvature_input < 0 then
            curvature_direction = -1                                                -- counter clockwise
            p4 = 1                                                                  -- ccw
        end
        local curvature_desired = curvature_input * curvature_input * curvature_max -- add expo

        -- lateral acceleration is positive because curvature_desired is positive
        -- 0.75 is an empirical adjustment factor
        -- local lateral_acceleration_desired = curvature_desired * speed_sq / 0.75
        -- curvature_desired = curvature_desired * curvature_direction

        local bearing_new = 90 * curvature_direction
        bearing_new = wrap_angle.deg_360(math.deg(state_now:vel_bearing()) + bearing_new)

        -- -- p1 = type (GUIDED_HEADING_NONE=0, GUIDED_HEADING_COG=1, GUIDED_HEADING_HEADING=2)
        -- -- p2 = heading in degrees
        -- -- p3 = max accel
        -- gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_HEADING, {
        --     p1 = HEADING_TYPE_HEADING,
        --     p2 = bearing_new,
        --     p3 = lateral_acceleration_desired,
        -- })


        -- Try to generate a curve by specifying the center of curvature and radius
        if curvature_desired < 0.0001 then
            curvature_desired = 0.0001
        end
        local radius = 1 / curvature_desired
        local center_bearing = curvature_direction * math.pi / 2
        center_bearing = wrap_angle.rad_2pi(state_now:vel_bearing() + center_bearing)

        local center = state_now:loc():copy()
        center:offset(radius * math.cos(center_bearing), radius * math.sin(center_bearing))

        local center_NE = state_start:loc():get_distance_NE(center)

        gcs:run_command_int(MAV_CMD_DO_REPOSITION, {
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT,
            p3 = radius,
            p4 = p4,
            x = center:lat(),
            y = center:lng(),
            z = 100
        })


        count = count + 1
        gcs_send(string.format("%03i, crv(i:%.2f, r:%.0f, p4:%.0f, d:%.4f, a:%.4f), n:%.0f, e:%.0f",
            count, curvature_input, radius, p4, curvature_desired, state_now:curvature(),
            center_NE:x(), center_NE:y()))
        -- gcs_send(string.format("%03i, spd(d:%.1f, a:%.1f) alt(d:%.1f, a:%.1f), crv(i:%.2f, d:%.4f, a:%.4f)",
        --     count, speed_desired, state_now:speed(), altitude_desired, state_now:alt(),
        --     curvature_input, curvature_desired, state_now:curvature()))


        ---@diagnostic disable: param-type-mismatch
        logger.write("GSAH", "SpdD,SpdA,AktD,AltA,CrvD,CrvA", "ffffff",
            speed_desired, state_now:speed(), altitude_desired,
            state_now:alt(), curvature_desired, state_now:curvature())
        ---@diagnostic enable: param-type-mismatch

        state_last = state_now
        -- Have to release the reference to state_last. Otherwise none of the state objects
        -- are freed and their memmory collected.
        state_last:clear_last()

        return true
    end
end

gcs_send("Loaded guide_repos_driver.lua")

return stuf.SwitchTriggerUpdateFunction("guide_repos_driver", Guider, GUIDING_TIME_MS, 300)
