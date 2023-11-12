-- 

local gcs_send = require("gcs_send_funcfact")("GAT")
local range2_nextfact = require("range2_nextfact")
local switch_exec_updatefact = require("switch_exec_updatefact")


local GUIDING_TIME_MS = 300
local WAITING_TIME_MS = 250

local PLANE_MODE_GUIDED        = 15

local enable_switch = rc:find_channel_for_option(300)
local angle_slider = rc:find_channel_for_option(301)

---@type nil|function(boolean)
local guider_func = nil

local target_alt_above_home = 25


local function wrap_360(angle)
    local res = math.fmod(angle, 360.0)
     if res < 0 then
         res = res + 360.0
     end
     return res
 end

local function wrap_180(angle)
     local res = wrap_360(angle)
     if res > 180 then
        res = res - 360
     end
     return res
 end

-- airspeed 15 20 25 30
-- angles -90 to 90 by 10
-- wait 5 sec, average for 2 sec

local function Guider()

    local MAX_GUIDE_ANGLE_D = 90.0

    if not enable_switch or not angle_slider then
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
        local alpha_c_l_d = wrap_180(alpha_c_n_d - alpha_l_n_d)
        alpha_l_n_d = alpha_c_n_d

        -- Calculate the rate of direction change
        local alpha_rate_c_dps = alpha_c_l_d / t_delta_s

        -- Determine the direction from here for the guide point.
        local alpha_g_c_d = angle_slider:norm_input() * MAX_GUIDE_ANGLE_D 
        local alpha_g_n_d = alpha_g_c_d + alpha_c_n_d

        local curvature = math.rad(alpha_rate_c_dps) / speed_cur_mps

        local target_loc = loc_c:copy()
        target_loc:offset_bearing(alpha_g_n_d, 40 * speed_cur_mps * GUIDING_TIME_MS * 0.001)
        target_loc:alt(loc_c:alt())
        target_loc:change_alt_frame(0)

        vehicle:update_target_location(wp_next, target_loc)

        -- local range_d_b_m = beg_loc:get_distance(loc_d)
        -- local alpha_d_b_d = math.deg(beg_loc:get_bearing(loc_d))
        -- local range_c_e_m = end_loc:get_distance(loc_c)
        -- gcs_send_trim("track", string.format("a_d_b %.2f, r_d_b %.2f, r_c_d %.2f, a_dc_d %.2f, e_ct %.2f, a_t_n %.2f, r_c_e %.2f, t %.2f",
        -- alpha_d_b_d, range_d_b_m, range_c_d_m, alpha_dc_d_d, error_ct_m, alpha_t_n_d, range_c_e_m, t))

        gcs_send("*", string.format("angle: %6.2f rate: %6.2f, curv: %8.4f", alpha_g_c_d, alpha_rate_c_dps, curvature))

        ---@diagnostic disable-next-line: param-type-mismatch
        logger.write("GATD", "Dt,SetA,Speed,RateA", "ffff", t_delta_s, alpha_g_c_d, speed_cur_mps, alpha_rate_c_dps)

        return true
    end
end



local function test_next_funcfact()
    local func, state = range2_nextfact(15, 30, 5, -90, 90, 10)

    local function test_iteration(abort)
        if abort then
            return false
        end

        local speed, angle = func(state)
        if not speed then
            return false
        end
        gcs_send(string.format("%3.0f, %3.0f", speed, angle))
        return true
    end

    return test_iteration
end

local function test_func_funcfact(speed_mps, angle_d)

    local first_loc = ahrs:get_location()
    local first_vel_vmps = ahrs:get_velocity_NED()
    if not first_loc or not first_vel_vmps then
        gcs_send("Aborting because cannot get start location.")
        return nil
    end

    -- Set the target airspeed.
    Parameter("TRIM_ARSPD_CM"):set(speed_mps * 100.0)

    first_loc:change_alt_frame(0)
    local first_vel_angle_d = math.deg(math.atan(first_vel_vmps:y(), first_vel_vmps:x()))

    local t_start_s = millis():tofloat() * 0.001
    local t_last_s = t_start_s

    local alpha_l_n_d = first_vel_angle_d

    local acc_speed_mps = 0.0
    local acc_rotrate_dps = 0.0
    local acc_curvature_pm = 0.0
    local num_measurements = 0

    local function test_func(accumulate_stats, dump_stats)

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
        -- if this is the first time, just continue 
        local t_cur_s = millis():tofloat() * 0.001 - t_start_s
        local t_delta_s = t_cur_s - t_last_s
        t_last_s = t_cur_s

        -- Find the change in angle since last
        local alpha_c_n_d = math.deg(math.atan(vel_cur_vmps:y(), vel_cur_vmps:x()))
        local alpha_c_l_d = wrap_180(alpha_c_n_d - alpha_l_n_d)
        alpha_l_n_d = alpha_c_n_d

        -- Calculate the rate of direction change
        local alpha_rate_c_dps = alpha_c_l_d / t_delta_s

        -- Determine the direction from here for the guide point.
        local alpha_g_c_d = angle_d
        local alpha_g_n_d = alpha_g_c_d + alpha_c_n_d

        local curvature = math.rad(alpha_rate_c_dps) / speed_cur_mps

        local target_loc = loc_c:copy()
        target_loc:offset_bearing(alpha_g_n_d, 40 * speed_cur_mps * GUIDING_TIME_MS * 0.001)
        target_loc:alt(loc_c:alt())
        target_loc:change_alt_frame(0)

        vehicle:update_target_location(wp_next, target_loc)

        -- local range_d_b_m = beg_loc:get_distance(loc_d)
        -- local alpha_d_b_d = math.deg(beg_loc:get_bearing(loc_d))
        -- local range_c_e_m = end_loc:get_distance(loc_c)
        -- gcs_send_trim("track", string.format("a_d_b %.2f, r_d_b %.2f, r_c_d %.2f, a_dc_d %.2f, e_ct %.2f, a_t_n %.2f, r_c_e %.2f, t %.2f",
        -- alpha_d_b_d, range_d_b_m, range_c_d_m, alpha_dc_d_d, error_ct_m, alpha_t_n_d, range_c_e_m, t))

        -- gcs_send("*", string.format("mps d:%6.2f c:%6.2f, deg d:%6.2f, dps:%6.2f, curv: %8.4f", speed_mps, speed_cur_mps, alpha_g_c_d, alpha_rate_c_dps, curvature))


        if accumulate_stats then
            acc_speed_mps = acc_speed_mps + speed_cur_mps
            acc_rotrate_dps = acc_rotrate_dps + alpha_rate_c_dps
            acc_curvature_pm = acc_curvature_pm + curvature
            num_measurements = num_measurements + 1

            if dump_stats then
                gcs_send(string.format("RESULT deg:%6.2f, mps:%6.2f, dps:%6.2f, curv: %8.4f", 
                angle_d, acc_speed_mps/num_measurements, acc_rotrate_dps/num_measurements, acc_curvature_pm/num_measurements))
            end
        end

        ---@diagnostic disable-next-line: param-type-mismatch
        logger.write("GATD", "Dt,SetA,Speed,RateA", "ffff", t_delta_s, alpha_g_c_d, speed_cur_mps, alpha_rate_c_dps)

        return true
    end

    return test_func
end

local function timed_test_funcfact(speed_mps, angle_d, settle_time_s, average_time_s)
    local test_func = test_func_funcfact(speed_mps, angle_d)
    if not test_func then
        return test_func
    end

    local t_start_s = millis():tofloat() * 0.001

    local function timed_test_func()
        local t_cur_s = millis():tofloat() * 0.001

        local accumulate_stats = (t_cur_s - t_start_s) > settle_time_s
        local dump_stats = (t_cur_s - t_start_s) > settle_time_s + average_time_s

        if not test_func(accumulate_stats, dump_stats) or dump_stats then
            return false
        end
        return true
    end

    return timed_test_func
end

local function guide_angle_test_funcfact()

    local next_func = range2_nextfact(15, 30, 5, -40, 40, 20)

    local test_func = nil

    -- Switch to guided mode
    local saved_mode = vehicle:get_mode()
    vehicle:set_mode(PLANE_MODE_GUIDED)

    local function finish()
        vehicle:set_mode(saved_mode)
    end

    local param_wp_radius = Parameter("WP_RADIUS"):get()
    local param_wp_loiter_rad = Parameter("WP_LOITER_RAD"):get()
    local param_trim_arspd_cm = Parameter("TRIM_ARSPD_CM"):get()
    gcs_send(string.format("WP_RADIUS:%.2f, WP_LOITER_RAD:%.2f, TRIM_ARSPD_CM:%.2f", param_wp_radius, param_wp_loiter_rad, param_trim_arspd_cm))

    local function exec(abort)
        if abort then
            finish()
            return false
        end

        repeat
            if not test_func then
                local speed_mps, angle_d = next_func()
                if not speed_mps then
                    return false -- end of loop
                end
                test_func = timed_test_funcfact(speed_mps, angle_d, 5.0, 2.0)
                if not test_func then
                    return false -- couldn't initialize
                end
            end

            if not test_func() then
                test_func = nil
            end
        until test_func

        return true
    end

    return exec
end

return (function()
    local r, d = switch_exec_updatefact("TestExec", guide_angle_test_funcfact, GUIDING_TIME_MS, 300, gcs_send)

    gcs_send(string.format("Loaded guide_angle_test.lua"))

    return r, d
end)()