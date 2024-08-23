


local gcs_send = require("gcs_send_funcfactory")("GAT")
local wrap_angle = require("wrap_angle_obj")
local switch_exec_updatefactory = require("switch_exec_updatefactory")


local GUIDING_TIME_MS = 98

local PLANE_MODE_GUIDED        = 15

local guiding_dist_m = 1000.0
local guiding_dist_frac = 0.995

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

        local time_cur = millis():tofloat() * 0.001

        -- Figure out where the next target should be.
        local loc_target = loc_now:copy()
        loc_target:offset(vel_start_2norm:x()*guiding_dist_m, vel_start_2norm:y()*guiding_dist_m)

        vehicle:update_target_location(wp_next, loc_target)
        -- vehicle:set_target_location(loc_target)


        -- local t_del = time_cur - time_start
        -- local dist_target = loc_now:get_distance(loc_target)
        -- local dist_track = loc_now:get_distance(loc_track)
        -- gcs_send(string.format("p %.2f (%.0f, %.0f, %.0f), (%.0f, %.0f, %.0f)", t_del, 
        --     p.n(), p.e(), dist_track, px.n(), px.e(), dist_target))
        -- -- gcs_send(string.format("Do it %i", time_cur - time_last))
        local pos_now = loc_start:get_distance_NE(loc_now)
        local pos_target = loc_start:get_distance_NE(loc_target)
        gcs_send(string.format("d:%.2f, now(%.0f, %.0f), targ(%.0f, %.0f), b:%.2f",
            guiding_dist_m, pos_now:x(), pos_now:y(), pos_target:x(), pos_target:y(), 
            math.deg(vel_now_bearing)))

        guiding_dist_m = guiding_dist_m * guiding_dist_frac

        return true
    end
end


return (function()
    local r, d = switch_exec_updatefactory("Guider L1 Trajectory", Guider, GUIDING_TIME_MS, 300, gcs_send)

    gcs_send(string.format("Loaded guide_distance_test.lua %i, %i", 1, 1))

    return r, d
end)()
