-- 
local GUIDING_TIME = 200
local WAITING_TIME = 250

local PLANE_MODE_GUIDED        = 15

local enable_switch = rc:find_channel_for_option(300)

local guider = nil

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
    local self = {}

    local GIDED_PATH_LENGTH_M = 500.0

    local saved_mode = vehicle:get_mode()
    vehicle:set_mode(PLANE_MODE_GUIDED)

    local beg_loc = ahrs:get_location()
    local erp = Equirect_projection(beg_loc)

    local base_vel_vmps = ahrs:get_velocity_NED()
    if not base_vel_vmps then
        return nil
    end

    local base_speed_mps = base_vel_vmps:length()
    --base_speed_mps = 12.0
    base_vel_vmps:x(0)
    base_vel_vmps:y(base_speed_mps)
    local guided_path_vm = Vector2f();
    guided_path_vm:x(base_vel_vmps:x()/base_speed_mps * GIDED_PATH_LENGTH_M)
    guided_path_vm:y(base_vel_vmps:y()/base_speed_mps * GIDED_PATH_LENGTH_M)

    local end_coords_vm = erp:loc_to_vm(beg_loc)
    end_coords_vm = end_coords_vm + guided_path_vm
    local end_time_ms = millis():tofloat() + 1000.0 * GIDED_PATH_LENGTH_M / base_speed_mps

    local calc_targets = Line_calc_targets(erp, end_coords_vm, end_time_ms, GUIDING_TIME, 2500)

    function self.guide()
        local target_loc = calc_targets()
        if not target_loc then
            return false
        end

        local target_vm = erp:loc_to_vm(target_loc) 
        gcs:send_text(0, string.format("guide %.2f, %.2f, %.0f", target_vm:x(), target_vm:y(), target_loc:alt()))

        vehicle:set_target_location(target_loc)
         return true
     end

    function self.finish()
        vehicle:set_mode(saved_mode)
    end

    return self
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
    guider.finish()
    guider = nil
    return goto_not_ready()
end

 local function state_guiding()
    if test_go(false) then
        return goto_complete()
    end
    if not guider.guide() then
        return goto_complete()
    end
    return state_guiding, GUIDING_TIME
end

local function goto_guiding()
    guider = Guider()
    if not guider then
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
    return state_ready, WAITING_TIME
end

state_not_ready = function()
    -- Ensure the trigger switch is off. This prevents guiding if reboot with switch on.
    if test_go(false) then
        return state_ready, 0
    end
    return state_not_ready, WAITING_TIME
end

gcs:send_text(0, string.format("Loaded guide_line.lua"))

return state_not_ready()