
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


local angle_slider = rc:find_channel_for_option(301)


local function Guider()

    local MAX_GUIDE_ANGLE_D = 50.0

    if not angle_slider then
        gcs_send("Error: no RC channels set up enabling or selecting angle")
        return nil
    end


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

    local time_last = time_start
    local vel_last_bearing = vel_start_bearing

    local acc_speed_mps = 0.0
    local acc_rotrate_dps = 0.0
    local acc_curvature_pm = 0.0
    local num_measurements = 0
    local time_acc_start_s = millis():tofloat() * 0.001
    local TIME_ACC_MAX_S = 2.0

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


        -- Find duration since last.
        local time_now = millis():tofloat() * 0.001
        local time_delta = time_now - time_last
        time_last = time_last

        -- Find the change in angle since last
        local vel_delta_bearing = wrap_angle.rad_pi(vel_now_bearing - vel_last_bearing)
        vel_last_bearing = vel_now_bearing

        -- Calculate the rate of direction change
        local rot_rate_c_dps = math.deg(vel_delta_bearing) / time_delta

        -- Determine the parameteers.
        local acc = angle_slider:norm_input() * 10
        local new_bearing = 90
        if acc < 0 then
            new_bearing = -new_bearing
        end
        new_bearing = wrap_angle.deg_360(math.deg(vel_now_bearing) + new_bearing)

        -- p1 = type (GUIDED_HEADING_NONE=0, GUIDED_HEADING_COG=1, GUIDED_HEADING_HEADING=2)
        -- p2 = heading in degrees
        -- p3 = max accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_HEADING, {
            p1 = HEADING_TYPE_COURSE_OVER_GROUND,
            p2 = new_bearing,
            p3 = math.abs(acc),
            })

        gcs_send(string.format("now_bearing %.1f, dem_bearing %.1f", math.deg(vel_now_bearing), new_bearing))

        -- gcs_send("*", string.format("angle: %6.2f rate: %6.2f", alpha_g_c_d, alpha_rate_c_dps))

        -- acc_speed_mps = acc_speed_mps + speed_cur_mps
        -- acc_rotrate_dps = acc_rotrate_dps + alpha_rate_c_dps
        -- acc_curvature_pm = acc_curvature_pm + curvature
        -- num_measurements = num_measurements + 1

        -- local time_acc_cur_s = millis():tofloat() * 0.001
        -- if (time_acc_cur_s - time_acc_start_s) > TIME_ACC_MAX_S then

        --     gcs_send(string.format("RESULT deg:%6.2f, mps:%6.2f, dps:%6.2f, curv: %8.4f",
        --         alpha_g_c_d, acc_speed_mps/num_measurements, acc_rotrate_dps/num_measurements, acc_curvature_pm/num_measurements))

        --     acc_speed_mps = 0.0
        --     acc_rotrate_dps = 0.0
        --     acc_curvature_pm = 0.0
        --     num_measurements = 0
        --     time_acc_start_s = time_acc_cur_s
        -- end

        -- ---@diagnostic disable-next-line: param-type-mismatch
        -- logger.write("GATD", "Dt,SetA,Speed,RateA", "ffff", t_delta_s, alpha_g_c_d, speed_cur_mps, alpha_rate_c_dps)

        return true
    end
end


return (function()
    local r, d = switch_exec_updatefactory("guide_cmd_int_heading_test", Guider, GUIDING_TIME_MS, 300, gcs_send)

    gcs_send("Loaded guide_cmd_int_heading_test.lua")

    return r, d
end)()