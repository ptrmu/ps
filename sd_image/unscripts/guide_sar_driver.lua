local gcs_send     = require("gcs_send_funcfactory")("GSAR")
local wrap_angle   = require("wrap_angle_obj")
local stuf         = require("switch_trigger_update_function")("guide_sar_driver", gcs_send)
local track        = require("track_obj")(gcs_send, wrap_angle)
local StateCurrent = require("ahrs_state")(gcs_send, wrap_angle).StateCurrent


local GUIDING_TIME_MS                   = 300

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

local speed_slider = find_channel(301, "speed")
local altitude_slider = find_channel(302, "altitude")
local curvature_slider = find_channel(303, "curvature")
if not speed_slider or not altitude_slider or not curvature_slider then
    return stuf.UpdateNothing()
end

local count = 0

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
        local speed_input   = speed_slider:norm_input()
        local speed_min     = 12
        local speed_max     = 20
        local speed_desired = (speed_input + 1) * (speed_max - speed_min) / 2 + speed_min

        -- -- p1 = type (SPEED_TYPE_AIRSPEED)
        -- -- p2 = airspeed
        -- -- p3 = max airspeed accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_SPEED, {
            p1 = SPEED_TYPE_AIRSPEED,
            p2 = speed_desired,
            p3 = 20,
        })

        -- Set altitude
        local altitude_desired = ({ 80, 90, 100 })[altitude_slider:get_aux_switch_pos() + 1]

        -- -- frame = type (MAV_FRAME_GLOBAL_RELATIVE_ALT)
        -- -- z = altitude
        -- -- p3 = max accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_ALTITUDE, {
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT,
            z = altitude_desired,
            p3 = 100,
        })


        local radius_min          = 40
        local radius_max          = 1000
        local curvature_min       = 1 / radius_max
        local curvature_max       = 1 / radius_min

        local curvature_input     = -curvature_slider:norm_input() -- Get direction correct
        local curvature_direction = 1                              -- clockwise
        local p4                  = 0
        if curvature_input < 0 then
            curvature_direction = -1 -- counter clockwise
            p4 = 1
        end

        -- Try to generate a curve by specifying the center of curvature and radius
        local curvature_desired = curvature_input * curvature_input * curvature_max -- add expo

        if curvature_desired < curvature_min then
            curvature_desired = curvature_min
        end
        local center_bearing = curvature_direction * math.pi / 2
        center_bearing = wrap_angle.rad_2pi(state_now:vel_bearing() + center_bearing)

        local radius = 1 / curvature_desired
        local radius_scaled = radius / ahrs:get_EAS2TAS() ^ 2
        local center = state_now:loc():copy()
        center:offset_bearing(math.deg(center_bearing), radius)

        gcs:run_command_int(MAV_CMD_DO_REPOSITION, {
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT,
            p3 = radius_scaled,
            p4 = p4,
            x = center:lat(),
            y = center:lng(),
            z = 100
        })


        local center_NE = center:get_distance_NE(state_start:loc())
        local actual_dist = center:get_distance(state_now:loc())
        count = count + 1
        gcs_send(string.format("%03i, crv(i:%.2f, r:%.0f, rs:%.0f, p4:%.0f, d:%.4f, a:%.4f)",
            count, curvature_input, radius, radius_scaled, p4, curvature_desired * curvature_direction,
            state_now:curvature()))
        -- gcs_send(string.format("%03i, spd(d:%.1f, a:%.1f) alt(d:%.1f, a:%.1f), crv(i:%.2f, d:%.4f, a:%.4f)",
        --     count, speed_desired, state_now:speed(), altitude_desired, state_now:alt(),
        --     curvature_input, curvature_desired, state_now:curvature()))


        ---@diagnostic disable: param-type-mismatch
        logger.write("GSAR", "SpdD,SpdA,AktD,AltA,CrvD,CrvA", "ffffff",
            speed_desired, state_now:speed(), altitude_desired,
            state_now:alt(), curvature_desired * curvature_direction, state_now:curvature())
        ---@diagnostic enable: param-type-mismatch

        state_last = state_now
        -- Have to release the reference to state_last. Otherwise none of the state objects
        -- are freed and their memmory collected.
        state_last:clear_last()

        return true
    end
end

return stuf.SwitchTriggerUpdateFunction(Guider, GUIDING_TIME_MS, 300)
