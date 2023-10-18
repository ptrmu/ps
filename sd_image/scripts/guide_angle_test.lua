-- 

local TEST_NAME = "GAT"

--#region GCS messaging

local function gcs_send_funcfact(test_name)

    ---@param str string
    return function(str)
        gcs:send_text(6, string.format("%s: %s", test_name, str))
    end
end

local gcs_send_trim_funcfact = function(gcs_send, eat_messages_period_s)
    local gcs_send_times = {}
    local gcs_eaten_count = {}

    ---@param str1 string
    ---@param str2 string|nil
    return function(str1, str2)
        if not str1 then return end

        local time_curr_s = millis():tofloat() / 1000.0
        local time_first_s = gcs_send_times[str1]
        if (time_first_s) then
            local dur_since_first = time_curr_s - time_first_s
            if dur_since_first < eat_messages_period_s then
                if not gcs_eaten_count[str1] then
                    gcs_eaten_count[str1] = 0
                end
                gcs_eaten_count[str1] = gcs_eaten_count[str1] + 1
                return
            end
        end

        local send_str = nil
        local eaten_count = gcs_eaten_count[str1]
        if eaten_count then
            gcs_eaten_count[str1] = nil
            if str2 then
                send_str = string.format("%s %s (+%i)", str1, str2, eaten_count)
            else
                send_str = string.format("%s (+%i)", str1, eaten_count)
            end
        else
            if str2 then
                send_str = string.format("%s %s", str1, str2)
            else
                send_str = string.format("%s", str1)
            end
        end

        gcs_send(send_str)
        gcs_send_times[str1] = time_curr_s
    end
end

local gcs_send = gcs_send_funcfact(TEST_NAME)
local gcs_send_trim = gcs_send_trim_funcfact(gcs_send, 1.0)


--#endregion


local GUIDING_TIME_MS = 200
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



local function Guider()

    local MAX_GUIDE_ANGLE_D = 90.0

    if not enable_switch or not angle_slider then
        gcs_send_trim("Error: no RC channels set up enabling or selecting angle")
        return nil
    end

    local first_loc = ahrs:get_location()
    local first_vel_vmps = ahrs:get_velocity_NED()
    if not first_loc or not first_vel_vmps then
        gcs_send_trim("Error: cannot get start location.")
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

        gcs_send_trim("*", string.format("angle: %6.2f rate: %6.2f", alpha_g_c_d, alpha_rate_c_dps))

        ---@diagnostic disable-next-line: param-type-mismatch
        logger.write("GATD", "Dt,SetA,Speed,RateA", "ffff", t_delta_s, alpha_g_c_d, speed_cur_mps, alpha_rate_c_dps)

        return true
    end
end



-- forward declarations of local functions
local state_not_ready

local function test_go(switch_true)
    if not arming:is_armed() then
        return false
    end
    return (enable_switch:get_aux_switch_pos() == 2) == switch_true
end

local function goto_not_ready()
    return state_not_ready, 0
end

local function goto_complete()
    gcs:send_text(0, string.format("Finish guiding"))
    if guider_func then
        guider_func(true)
        guider_func = nil
    end
    return goto_not_ready()
end

 local function state_guiding()
    if test_go(false) then
        return goto_complete()
    end
    if not guider_func or not guider_func(false) then
        return goto_complete()
    end
    return state_guiding, GUIDING_TIME_MS
end

local function goto_guiding()
    guider_func = Guider()
    if not guider_func then
        gcs:send_text(0, string.format("Start guiding failed: no guiding object"))
        return goto_not_ready()
    end
    gcs:send_text(0, string.format("Start guiding"))
    return state_guiding, 0
end

 local function state_ready()
    -- Wait until the trigger switch is on before starting guiding.
    if test_go(true) then
        return goto_guiding()
    end
    return state_ready, WAITING_TIME_MS
end

state_not_ready = function()
    -- Ensure the trigger switch is off. This prevents guiding if reboot with switch on.
    if test_go(false) then
        return state_ready, 0
    end
    return state_not_ready, WAITING_TIME_MS
end

gcs:send_text(0, string.format("Loaded guide_angle_test.lua"))

return state_not_ready()