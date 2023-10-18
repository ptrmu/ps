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


local function Guider()

    local BEG_LOC_BEARING_D = 90.0
    local BEG_LOC_DISTANCE_M = 50.0
    local END_LOC_BEARING_D = 0 -- from BEG_LOC
    local END_LOC_DISTANCE_M = 250.0

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

    local end_loc = beg_loc:copy()

     -- Call the vector from beg to end p.
    -- The path to follow is path(lambra) = beg + lambda * p.
    -- The angle of p to north in degrees.
    local alpha_p_n_d = first_vel_angle_d + END_LOC_BEARING_D

    end_loc:offset_bearing(alpha_p_n_d, END_LOC_DISTANCE_M)
    end_loc:alt(beg_loc:alt())


    return function(abort)

        if abort then
            finish()
            return false
        end

        local cur_loc = ahrs:get_location()
        local cur_vel_vmps = ahrs:get_velocity_NED()
        local next_wp = vehicle:get_target_location()
        if not cur_loc or not cur_vel_vmps or not next_wp then
            gcs_send("aborting due to nil object")
            return false
        end

        -- test for getting close to ground
        local gnd_loc = cur_loc:copy()
        if gnd_loc:change_alt_frame(3) and gnd_loc:alt() < 1000 then
            gcs_send("Aborting guiding because too close to ground")
            return false
        end


        -- The vector from beg to cur is c
        -- The angle of c to north in degrees
        local alpha_c_n_d = math.deg(beg_loc:get_bearing(cur_loc))
        -- The distance from beg to cur in meters
        local range_c_b_m = beg_loc:get_distance(cur_loc)

        -- The angle of c to p in degrees
        local alpha_c_p_d = alpha_c_n_d - alpha_p_n_d

        -- The distance c is from the path line - the cross track error.
        local error_ct_m = range_c_b_m * math.sin(math.rad(alpha_c_p_d))

        -- The angle between the restoring vector t and p
        local alpha_t_p_d = -ALPHA_T_P_MAX_D * 2 * math.atan(ERROR_CT_SCALE * error_ct_m) / math.pi

        -- The angle between the restoring vector t and north
        local alpha_t_n_d = alpha_t_p_d + alpha_p_n_d

        local cur_speed_mps = cur_vel_vmps:length()
        local cur_vel_angle_d = math.deg(math.atan(cur_vel_vmps:y(), cur_vel_vmps:x()))

        local target_loc = cur_loc:copy()
        target_loc:offset_bearing(alpha_t_n_d, 40 * cur_speed_mps * GUIDING_TIME_MS * 0.001)
        target_loc:alt(end_loc:alt())
        target_loc:change_alt_frame(0)

        vehicle:update_target_location(next_wp, target_loc)

        -- Figure out if we will have passed the waypoint by the next itereation.
        local nxt_loc = cur_loc:copy()
        nxt_loc:offset_bearing(alpha_t_n_d, cur_speed_mps * GUIDING_TIME_MS * 0.001)

        -- The angle between the end location and the next location relative to north
        local alpha_x_n_d = math.deg(end_loc:get_bearing(nxt_loc))

        -- The angle of end-nxt vector to p
        local alpha_x_p_d = alpha_x_n_d - alpha_p_n_d

        local alpha_x_p_d_wrapped = wrap_180(alpha_x_p_d)

        local range_x_e_m = end_loc:get_distance(nxt_loc)
        gcs_send_trim("track", string.format("a_first %.2f, a_p_n %.2f, r_c_b %.2f, a_c_p %.2f, e_ct %.2f, a_t_p %.2f, a_t_n %.2f, a_x_p %.2f, r_x_e %.2f",
        first_vel_angle_d, alpha_p_n_d, range_c_b_m, alpha_c_p_d, error_ct_m, alpha_t_p_d, alpha_t_n_d, alpha_x_p_d_wrapped, range_x_e_m))

            -- Convert to an angle between -pi and pi.
        -- If the angle is between -90 and 90, then the next point
        -- is in front of the end point and this loop should terminate.
        return math.abs(alpha_x_p_d_wrapped) > 90
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