


local gcs_send = require("gcs_send_funcfactory")("GRC")
local wrap_angle = require("wrap_angle_obj")
local switch_exec_updatefactory = require("switch_exec_updatefactory")


local GUIDING_TIME_MS = 300

local PLANE_MODE_GUIDED        = 15

local guiding_dist_m = 1000.0
local guiding_dist_frac = 0.995

local MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

local MAV_CMD_DO_SET_MODE = 176
local MAV_CMD_DO_REPOSITION = 192

local MAV_CMD_GUIDED_CHANGE_SPEED = 43000
local MAV_CMD_GUIDED_CHANGE_ALTITUDE = 43001
local MAV_CMD_GUIDED_CHANGE_HEADING = 43002

local HEADING_TYPE_COURSE_OVER_GROUND = 0
local HEADING_TYPE_HEADING = 1
local SPEED_TYPE_AIRSPEED = 0
local MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

local speed_switch = rc:find_channel_for_option(302)
local turn_switch = rc:find_channel_for_option(301)
local altitude_switch = rc:find_channel_for_option(304)

if not speed_switch or not turn_switch or not altitude_switch then
    gcs_send("Could not find channels")
    return
end

local function Guider()

    local loc_start = ahrs:get_location()
    local vel_start_vmps_temp = ahrs:get_velocity_NED()
    if not loc_start or not vel_start_vmps_temp then
        gcs_send("Error: cannot get start location.")
        return nil
    end

    loc_start:change_alt_frame(0)
    local vel_start_2mps = Vector2f()
    vel_start_2mps:x(vel_start_vmps_temp:x())
    vel_start_2mps:y(vel_start_vmps_temp:y())
    local vel_start_bearing = vel_start_2mps:angle()
    local vel_start_speed_mps = vel_start_2mps:length()
    local vel_start_2norm = vel_start_2mps:copy()
    vel_start_2norm:normalize()


    -- All the failure modes have passed so we can enable guiding. A question is
    -- should we have set a target wp before enabling guiding. For now no but this needs checking
    local saved_mode = vehicle:get_mode()
    vehicle:set_mode(PLANE_MODE_GUIDED)

    local function finish()
        vehicle:set_mode(saved_mode)
    end

    local time_start = millis():tofloat() * 0.001

    return function(abort)

        if abort then
            finish()
            return false
        end

        local loc_now = ahrs:get_location()
        local vel_now_vmps = ahrs:get_velocity_NED()
        local wp_next = vehicle:get_target_location()
        if not loc_now or not vel_now_vmps or not wp_next then
            gcs_send("Aborting due to nil object")
            return false
        end

        loc_now:change_alt_frame(0)
        local vel_now_2mps = Vector2f()
        vel_now_2mps:x(vel_now_vmps:x())
        vel_now_2mps:y(vel_now_vmps:y())
        local vel_now_bearing = vel_now_2mps:angle()
        local vel_now_speed_mps = vel_now_2mps:length()
        local vel_now_2norm = vel_now_2mps:copy()
        vel_now_2norm:normalize()
    
        -- test for getting close to ground
        local loc_ground = loc_now:copy()
        if loc_ground:change_alt_frame(3) and loc_ground:alt() < 1000 then
            gcs_send("Aborting guiding because too close to ground")
            return false
        end


        local speed_desired = ({20, 24, 28})[speed_switch:get_aux_switch_pos()+1]
        local turn_desired = ({0, 5, 10})[turn_switch:get_aux_switch_pos()+1]
        local altitude_desired = ({80, 100, 120})[altitude_switch:get_aux_switch_pos()+1]

        local new_bearing = math.deg(wrap_angle.rad_2pi(vel_now_bearing + math.pi/2))

        -- p1 = type (SPEED_TYPE_AIRSPEED)
        -- p2 = airspeed
        -- p3 = max airspeed accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_SPEED, {
            p1 = SPEED_TYPE_AIRSPEED,
            p2 = speed_desired,
            p3 = 20,
            })

        -- frame = type (MAV_FRAME_GLOBAL_RELATIVE_ALT)
        -- z = altitude 
        -- p3 = max accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_ALTITUDE, {
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT,
            z = altitude_desired,
            p3 = 100,
            })

        -- p1 = type (GUIDED_HEADING_NONE=0, GUIDED_HEADING_COG=1, GUIDED_HEADING_HEADING=2)
        -- p2 = heading in degrees
        -- p3 = max accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_HEADING, {
            p1 = HEADING_TYPE_COURSE_OVER_GROUND,
            p2 = new_bearing,
            p3 = turn_desired,
            })

        gcs_send(string.format("Desired (speed %.0f, turn %.3f, altitude %.0f)", 
        speed_desired, turn_desired, altitude_desired))

        return true
    end
end


return (function()
    local r, d = switch_exec_updatefactory("Guider Run Cmd Int Test", Guider, GUIDING_TIME_MS, 300, gcs_send)

    gcs_send(string.format("Loaded guide_run_cmd_int_test.lua %i, %i", 1, 1))

    return r, d
end)()
