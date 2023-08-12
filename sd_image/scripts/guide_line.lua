-- 
local GUIDING_TIME = 100
local WAITING_TIME = 250

local PLANE_MODE_GUIDED        = 15

local enable_switch = rc:find_channel_for_option(300)

local guider = nil

local target_alt_above_home = 25

local function Equirect_projection(loc)
    local self = {}

    local radius_m = 6371000.0

    local base_lat_r = math.rad(loc:lat()/10000000.0)
    local base_lng_r = math.rad(loc:lng()/10000000.0)
    local base_cos_lat = math.cos(base_lat_r)

    function self.latlng_to_xy(lat, lng)
        local delta_lat_r = math.rad(lat/10000000.0) - base_lat_r
        local delta_lng_r = math.rad(lng/10000000.0) - base_lng_r
        return delta_lat_r * radius_m, delta_lng_r * radius_m * base_cos_lat
    end

    function self.xy_to_latlng(x, y)
        local delta_lat_r = x / radius_m
        local delta_lng_r = y / radius_m / base_cos_lat
        return math.deg(delta_lat_r + base_lat_r) * 10000000.0, math.deg(delta_lng_r + base_lng_r) * 10000000.0
    end

    return self
end

local function Guider()
    local self = {}

    local saved_mode = vehicle:get_mode()
    vehicle:set_mode(PLANE_MODE_GUIDED)

    local erp = Equirect_projection(ahrs:get_location())
    local base_vel = ahrs:get_velocity_NED()
    local delta_x_m = GUIDING_TIME * base_vel:x() * 200.0 / 1000.0
    local delta_y_m = GUIDING_TIME * base_vel:y() * 200.0 / 1000.0
    local last_target_loc = nil

    function self.guide()
        local loc = ahrs:get_location()
        local x_m, y_m = erp.latlng_to_xy(loc:lat(), loc:lng())

        local new_x_m = x_m + delta_x_m
        local new_y_m = y_m + delta_y_m

        local new_lat, new_lng = erp.xy_to_latlng(new_x_m, new_y_m)

        local new_loc = loc:copy()
        new_loc:lat(new_lat)
        new_loc:lng(new_lng)

        vehicle:set_target_location(new_loc)
        ahrs:set_home(new_loc)
        gcs:send_text(0, string.format("guide %.2f, %.2f, %.2f, %.2f", loc:lat(), new_lat, loc:lng(), new_lng))
     end

    function self.finish()
        vehicle:set_mode(saved_mode)
    end

    return self
end

-- forward declarations of local functions
local goto_guiding
local goto_waiting
local state_guiding
local state_waiting

goto_guiding = function()
    guider = Guider()
    return state_guiding, 0
end

goto_waiting = function()
    guider.finish()
    guider = nil
    return state_waiting, 0
end

state_guiding = function()
    if enable_switch:get_aux_switch_pos() ~= 2 then
        return goto_waiting()
    end
    guider.guide()
    return state_guiding, GUIDING_TIME
end

state_waiting = function()
    if arming:is_armed() and enable_switch:get_aux_switch_pos() == 2 then
        return goto_guiding()
    end
    return state_waiting, WAITING_TIME
end

gcs:send_text(0, string.format("Loaded guide_line.lua"))

return state_waiting()