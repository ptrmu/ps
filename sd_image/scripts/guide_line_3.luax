-- 

local TEST_NAME = "GL"

--#region GCS messaging

---@param str1 string
local function gcs_send(str1)
    gcs:send_text(6, string.format("%s: %s", TEST_NAME, str1))
end

local gcs_send_trim = (function()
    local gcs_send_times = {}
    local gcs_eaten_count = {}
    local GCS_SEND_PERIOD_S = 1.0

    ---@param str1 string
    ---@param str2 string
    local function func(str1, str2)
        if not str1 then return end

        local time_curr_s = millis():tofloat() / 1000.0
        local time_first_s = gcs_send_times[str1]
        if (time_first_s) then
            local dur_since_first = time_curr_s - time_first_s
            if dur_since_first < GCS_SEND_PERIOD_S then
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

    return func
end)()


--#endregion


local GUIDING_TIME_MS = 200
local WAITING_TIME_MS = 250

local PLANE_MODE_GUIDED        = 15

local enable_switch = rc:find_channel_for_option(300)

---@type nil|function(boolean)
local guider_func = nil

local target_alt_above_home = 25

local function Equirect_projection(ref_loc)
    local obj = {}

    obj.EARTH_RADIUS_M = 6371000.0

    obj.ref_lat_r = math.rad(ref_loc:lat()/10000000.0)
    obj.ref_lng_r = math.rad(ref_loc:lng()/10000000.0)
    obj.ref_cos_lat = math.cos(obj.ref_lat_r)

    function obj.latlng_to_xy(self, lat, lng)
        local delta_lat_r = math.rad(lat/10000000.0) - self.ref_lat_r
        local delta_lng_r = math.rad(lng/10000000.0) - self.ref_lng_r
        return delta_lat_r * self.EARTH_RADIUS_M, delta_lng_r * self.EARTH_RADIUS_M * self.ref_cos_lat
    end

    function obj.xy_to_latlng(self, x, y)
        local delta_lat_r = x / self.EARTH_RADIUS_M
        local delta_lng_r = y / self.EARTH_RADIUS_M / self.ref_cos_lat
        return math.deg(delta_lat_r + self.ref_lat_r) * 10000000.0, math.deg(delta_lng_r + self.ref_lng_r) * 10000000.0
    end

    function obj.loc_to_vm(self, loc)
        local vm = Vector2f()
        local x, y = self:latlng_to_xy(loc:lat(), loc:lng())
        vm:x(x)
        vm:y(y)
        return vm
    end

    function obj.vm_in_loc(self, vm, loc)
        local lat, lng = self:xy_to_latlng(vm:x(), vm:y())
        loc:lat(lat)
        loc:lng(lng)
    end

    return obj
end

local function Line_calc_targets(erp, end_coords_vm, end_time_ms, delta_time_ms, target_extension_m)

    local beg_time_ms = millis():tofloat()
    assert(end_time_ms - beg_time_ms > 1, "Can only calc a new target into the future")

    local beg_coords_vm = erp:loc_to_vm(ahrs:get_location())

    local desired_speed_vmps = Vector2f()
    local tot_time_s = (end_time_ms - beg_time_ms) / 1000.0
    desired_speed_vmps:x((end_coords_vm:x() - beg_coords_vm:x()) / tot_time_s)
    desired_speed_vmps:y((end_coords_vm:y() - beg_coords_vm:y()) / tot_time_s)

    local function calc_next_target()
        local cur_time_ms = millis():tofloat()
        if (cur_time_ms >= end_time_ms) then
            return nil
        end
 
        local delta_nxt_time_s = (cur_time_ms - beg_time_ms + delta_time_ms) / 1000.0
        local nxt_coords_vm = Vector2f()
        nxt_coords_vm:x(desired_speed_vmps:x() * delta_nxt_time_s)
        nxt_coords_vm:y(desired_speed_vmps:y() * delta_nxt_time_s)
        nxt_coords_vm = nxt_coords_vm + beg_coords_vm

        local wrk_loc = ahrs:get_location():copy()
        local cur_coords_vm = erp:loc_to_vm(wrk_loc)
        local cur_nxt_vm = nxt_coords_vm - cur_coords_vm
        local cur_nxt_length = cur_nxt_vm:length()
        cur_nxt_vm:x(cur_nxt_vm:x()/cur_nxt_length * target_extension_m)
        cur_nxt_vm:y(cur_nxt_vm:y()/cur_nxt_length * target_extension_m)

        local tar_coords_vm = nxt_coords_vm + cur_nxt_vm
        erp:vm_in_loc(tar_coords_vm, wrk_loc)
        return wrk_loc
    end

    return calc_next_target
end


local function Guider()

    local GIDED_PATH_LENGTH_M = 500.0

    local saved_mode = vehicle:get_mode()
    vehicle:set_mode(PLANE_MODE_GUIDED)

    local function finish()
        vehicle:set_mode(saved_mode)
    end

    local beg_loc = ahrs:get_location()
    if not beg_loc then
        return nil
    end

    local erp = Equirect_projection(beg_loc)

    local end_loc = beg_loc:copy()
    end_loc:offset(0.0, GIDED_PATH_LENGTH_M)
    end_loc:alt(75000)
    end_loc:change_alt_frame(0)

    local end_vm = erp:loc_to_vm(end_loc)
    gcs_send_trim("Start", string.format(" %.2f, %.2f, %.0f", end_vm:x(), end_vm:y(), beg_loc:alt()))


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

        local cur_speed_mps = cur_vel_vmps:length()

        local distanceNE = cur_loc:get_distance_NE(end_loc)
        --local bearing_to_end = math.deg(math.atan(distanceNE:y(), distanceNE:x()))
        local bearing_to_end = cur_loc:get_bearing(end_loc)

        local distance_to_end = cur_loc:get_distance(end_loc)
        gcs_send_trim("pre", string.format(" %.2f, %.2f", math.deg(bearing_to_end), distance_to_end))
        local target_loc = cur_loc:copy()
        target_loc:offset_bearing(math.deg(bearing_to_end), 40 * cur_speed_mps * GUIDING_TIME_MS * 0.001)
        target_loc:alt(end_loc:alt())
        target_loc:change_alt_frame(0)

        local cur_vm = erp:loc_to_vm(cur_loc)
        gcs_send_trim("cur", string.format(" %.2f, %.2f, %.0f, %.2f, %i, %i", cur_vm:x(), cur_vm:y(), cur_loc:alt(), bearing_to_end, cur_loc:get_alt_frame(), target_loc:get_alt_frame()))

        local target_vm = erp:loc_to_vm(target_loc) 
        gcs_send_trim("Guide", string.format(" %.2f, %.2f, %.0f", target_vm:x(), target_vm:y(), target_loc:alt()))

        vehicle:update_target_location(next_wp, target_loc)
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

gcs:send_text(0, string.format("Loaded guide_line.lua"))

return state_not_ready()