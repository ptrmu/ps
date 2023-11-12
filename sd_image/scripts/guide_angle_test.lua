-- 

local gcs_send = require("gcs_send_funcfact")("GAT")
local wrap_angle = require("wrap_angle_obj")
local switch_exec_updatefact = require("switch_exec_updatefact")


local GUIDING_TIME_MS = 300

local PLANE_MODE_GUIDED        = 15

local angle_slider = rc:find_channel_for_option(301)

---@type nil|function(boolean)
local guider_func = nil

local target_alt_above_home = 25

local function Guider()

    local MAX_GUIDE_ANGLE_D = 90.0

    if not angle_slider then
        gcs_send("Error: no RC channels set up enabling or selecting angle")
        return nil
    end

    local first_loc = ahrs:get_location()
    local first_vel_vmps = ahrs:get_velocity_NED()
    if not first_loc or not first_vel_vmps then
        gcs_send("Error: cannot get start location.")
        return nil
    end
    first_loc:change_alt_frame(0)
    local first_vel_angle_d = math.deg(math.atan(first_vel_vmps:y(), first_vel_vmps:x()))

    -- All the failure modes have passed so we can enable guiding. A question is
    -- should we have set a target wp before enabling guiding. For now no but this needs checking
    local saved_mode = vehicle:get_mode()
    vehicle:set_mode(PLANE_MODE_GUIDED)

    local function finish()
        vehicle:set_mode(saved_mode)
    end

    local t_start_s = millis():tofloat() * 0.001
    local t_last_s = -GUIDING_TIME_MS

    local alpha_l_n_d = first_vel_angle_d



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

        local loc_c = ahrs:get_location()
        local vel_cur_vmps = ahrs:get_velocity_NED()
        local wp_next = vehicle:get_target_location()
        if not loc_c or not vel_cur_vmps or not wp_next then
            gcs_send("Aborting due to nil object")
            return false
        end
        local speed_cur_mps = vel_cur_vmps:length()
        loc_c:change_alt_frame(0)

        -- test for getting close to ground
        local loc_g = loc_c:copy()
        if loc_g:change_alt_frame(3) and loc_g:alt() < 1000 then
            gcs_send("Aborting guiding because too close to ground")
            return false
        end

        -- Find duration since last.
        local t_cur_s = millis():tofloat() * 0.001 - t_start_s
        local t_delta_s = t_cur_s - t_last_s
        t_last_s = t_cur_s

        -- Find the change in angle since last
        local alpha_c_n_d = math.deg(math.atan(vel_cur_vmps:y(), vel_cur_vmps:x()))
        local alpha_c_l_d = wrap_angle.deg_180(alpha_c_n_d - alpha_l_n_d)
        alpha_l_n_d = alpha_c_n_d

        -- Calculate the rate of direction change
        local alpha_rate_c_dps = alpha_c_l_d / t_delta_s

        -- Determine the direction from here for the guide point.
        local alpha_g_c_d = math.floor(((1 + angle_slider:norm_input()) * MAX_GUIDE_ANGLE_D + 5.0)/10.0) * 10.0 - MAX_GUIDE_ANGLE_D
        local alpha_g_n_d = alpha_g_c_d + alpha_c_n_d

        local curvature = math.rad(alpha_rate_c_dps) / speed_cur_mps

        local target_loc = loc_c:copy()
        target_loc:offset_bearing(alpha_g_n_d, 40 * speed_cur_mps * GUIDING_TIME_MS * 0.001)
        target_loc:alt(loc_c:alt())
        target_loc:change_alt_frame(0)

        vehicle:update_target_location(wp_next, target_loc)

        -- gcs_send("*", string.format("angle: %6.2f rate: %6.2f", alpha_g_c_d, alpha_rate_c_dps))

        acc_speed_mps = acc_speed_mps + speed_cur_mps
        acc_rotrate_dps = acc_rotrate_dps + alpha_rate_c_dps
        acc_curvature_pm = acc_curvature_pm + curvature
        num_measurements = num_measurements + 1

        local time_acc_cur_s = millis():tofloat() * 0.001
        if (time_acc_cur_s - time_acc_start_s) > TIME_ACC_MAX_S then

            gcs_send(string.format("RESULT deg:%6.2f, mps:%6.2f, dps:%6.2f, curv: %8.4f",
                alpha_g_c_d, acc_speed_mps/num_measurements, acc_rotrate_dps/num_measurements, acc_curvature_pm/num_measurements))

            acc_speed_mps = 0.0
            acc_rotrate_dps = 0.0
            acc_curvature_pm = 0.0
            num_measurements = 0
            time_acc_start_s = time_acc_cur_s
        end

        ---@diagnostic disable-next-line: param-type-mismatch
        logger.write("GATD", "Dt,SetA,Speed,RateA", "ffff", t_delta_s, alpha_g_c_d, speed_cur_mps, alpha_rate_c_dps)

        return true
    end
end


return (function()
    local r, d = switch_exec_updatefact("Guider Test", Guider, GUIDING_TIME_MS, 300, gcs_send)

    gcs_send("Loaded guide_angle_test.lua")

    return r, d
end)()