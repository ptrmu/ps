-- 

local TEST_NAME = "GL"

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
    ---@param str2 string
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




-- Function
--  time domain: 0.0 .. 1.0
--  loc_func(time): location, bearing_deg
--
--  loc_funcfact(start_loc, start_bearing_deg): loc_func
--
--  line_loc_funcfactfact(): loc_funcfact
--  arc_loc_funcfactfact(radius) 

local function line_loc_funcfactfact(d_m)

    local function loc_funcfact(loc_s, alpha_s_n_d)

         local function loc_func(t)
            local loc_start = loc_s:copy()
            loc_start:offset_bearing(alpha_s_n_d, d_m * t)
            return loc_start, alpha_s_n_d
        end

        return loc_func
    end

    return loc_funcfact
end


local function Guider()

    local BEG_LOC_BEARING_D = 0.0
    local BEG_LOC_DISTANCE_M = 100.0
    local END_LOC_BEARING_D = 90 -- from BEG_LOC
    local END_LOC_DISTANCE_M = 250.0

    local DUR_LINE_S = 15.0

    local ALPHA_T_P_MAX_D = 75.0
    local ERROR_CT_SCALE = 0.025

    local GIDED_PATH_LENGTH_M = 500.0

    local saved_mode = vehicle:get_mode()
    vehicle:set_mode(PLANE_MODE_GUIDED)

    local function finish()
        vehicle:set_mode(saved_mode)
    end


    local first_loc = ahrs:get_location()
    local first_vel_vmps = ahrs:get_velocity_NED()
    if not first_loc or not first_vel_vmps then
        gcs_send_trim("Error: cannot get start location.", "")
        return nil
    end
    first_loc:change_alt_frame(0)
    local first_vel_angle_d = math.deg(math.atan(first_vel_vmps:y(), first_vel_vmps:x()))

    local beg_loc = first_loc:copy()
    -- beg_loc:offset_bearing(BEG_LOC_BEARING_D, BEG_LOC_DISTANCE_M)
    beg_loc:offset_bearing(first_vel_angle_d + BEG_LOC_BEARING_D, BEG_LOC_DISTANCE_M)
    beg_loc:alt(first_loc:alt())

     -- Call the vector from beg to end p.
    -- The path to follow is path(lambra) = beg + lambda * p.
    -- The angle of p to north in degrees.
    local alpha_p_n_d = first_vel_angle_d + END_LOC_BEARING_D


    local line_loc_funcfact = line_loc_funcfactfact(END_LOC_DISTANCE_M)
    local line_loc_func = line_loc_funcfact(beg_loc, alpha_p_n_d)


    local end_loc = line_loc_func(1.0)

    local t_start_s = millis():tofloat() * 0.001

    gcs_send_trim("start", string.format("a_first %.2f, a_p_n %.2f", first_vel_angle_d, alpha_p_n_d))


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

        -- test for getting close to ground
        local loc_g = loc_c:copy()
        if loc_g:change_alt_frame(3) and loc_g:alt() < 1000 then
            gcs_send("Aborting guiding because too close to ground")
            return false
        end

        -- Test for time complete
        local t_cur_s = millis():tofloat() * 0.001 - t_start_s
        if t_cur_s > DUR_LINE_S then
            local d_c_e_m = loc_c:get_distance(end_loc)
            gcs_send(string.format("Done time up. To end: %.2fm", d_c_e_m))
            return false
        end

        -- Get the desired location and direction for this time
        local t = t_cur_s / DUR_LINE_S
        local loc_d, alpha_d_n_d = line_loc_func(t)

        -- Find the distance between loc_c and line line defined by loc_d, alpha_d_n_d.
        -- This is the cross track error.
        -- The vector from loc_d to loc_c is dc.
        -- The angle of dc to north in degrees.
        local alpha_dc_n_d = math.deg(loc_d:get_bearing(loc_c))
        -- The angle between d and dc in degrees.
        local alpha_dc_d_d = alpha_dc_n_d - alpha_d_n_d
        -- The distance between loc_c and loc_d in meters.
        local range_c_d_m = loc_d:get_distance(loc_c)
        -- And the cross track error in meters.
        local error_ct_m = range_c_d_m * math.sin(math.rad(alpha_dc_d_d))

        -- The target angle will be along the vector between the current and desired loc
        local alpha_t_n_d = math.deg(loc_c:get_bearing(loc_d))


        local target_loc = loc_c:copy()
        target_loc:offset_bearing(alpha_t_n_d, 40 * speed_cur_mps * GUIDING_TIME_MS * 0.001)
        target_loc:alt(end_loc:alt())
        target_loc:change_alt_frame(0)

        vehicle:update_target_location(wp_next, target_loc)

        local range_d_b_m = beg_loc:get_distance(loc_d)
        local alpha_d_b_d = math.deg(beg_loc:get_bearing(loc_d))
        local range_c_e_m = end_loc:get_distance(loc_c)
        gcs_send_trim("track", string.format("a_d_b %.2f, r_d_b %.2f, r_c_d %.2f, a_dc_d %.2f, e_ct %.2f, a_t_n %.2f, r_c_e %.2f, t %.2f",
        alpha_d_b_d, range_d_b_m, range_c_d_m, alpha_dc_d_d, error_ct_m, alpha_t_n_d, range_c_e_m, t))

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

gcs:send_text(0, string.format("Loaded guide_line_4.lua"))

return state_not_ready()