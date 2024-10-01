local gcs_send                          = require("gcs_send_funcfactory")("TRR")
local wrap_angle                        = require("wrap_angle_obj")
local stuf                              = require("switch_trigger_update_function")("track_repos_v2", gcs_send)
local track                             = require("track_obj")(gcs_send, wrap_angle)
local ahrs_state                        = require("ahrs_state")(gcs_send, wrap_angle)
local StateCurrent                      = ahrs_state.StateCurrent

local GUIDING_TIME_MS                   = 100
local TRANSMITTER_SWITCH_CODE           = 300

local PLANE_MODE_GUIDED                 = 15

local MAV_CMD_DO_REPOSITION             = 192
local MAV_CMD_GUIDED_CHANGE_SPEED       = 43000
local MAV_CMD_GUIDED_CHANGE_ALTITUDE    = 43001
local MAV_CMD_GUIDED_CHANGE_HEADING     = 43002

local MODE_GUIDED                       = 15

local HEADING_TYPE_HEADING              = 1
local SPEED_TYPE_AIRSPEED               = 0
local MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
local MAV_FRAME_GLOBAL_RELATIVE_ALT     = 3

local params                            = {
    tim = {
        update_ms = GUIDING_TIME_MS,
        update_skip_max = 2,
    },
    spd = {
        min = 12,
        mid = 16,
        max = 20,
        pi_kP = .25,
        pi_kI = .01,
        pi_iMax = 4,
        pi_min = -4,
        pi_max = 4,
    },
    pos = {
        radius_min = 40,
        radius_max = 1000,
    },
    alt = {
        target = 80,
    },
}

local p_loc_n                           = function(loc) return 0 end
local p_loc_e                           = function(loc) return 0 end
local p_spot_n                          = function(spot) return 0 end
local p_spot_e                          = function(spot) return 0 end
local p_loc_lat                         = function(loc) return 0 end
local p_loc_lng                         = function(loc) return 0 end

local function p_setup(loc_origin)
    local lat_origin = loc_origin:lat()
    local lng_origin = loc_origin:lng()
    p_loc_n = function(loc) return loc_origin:get_distance_NE(loc):x() end
    p_loc_e = function(loc) return loc_origin:get_distance_NE(loc):y() end
    p_spot_n = function(spot) return spot:n() end
    p_spot_e = function(spot) return spot:e() end
    p_loc_lat = function(loc) return loc:lat() - lat_origin end
    p_loc_lng = function(loc) return loc:lng() - lng_origin end
end

-- constrain a value between limits
local function constrain(v, vmin, vmax)
    if v < vmin then
        v = vmin
    end
    if v > vmax then
        v = vmax
    end
    return v
end

-- a PI controller implemented as a Lua object
local function PI_controller(kP, kI, iMax, min, max)
    -- the new instance. You can put public variables inside this self
    -- declaration if you want to
    local self = {}

    -- private fields as locals
    local _kP = kP or 0.0
    local _kI = kI or 0.0
    local _iMax = iMax
    local _min = min
    local _max = max
    local _last_t = nil
    local _I = 0
    local _P = 0
    local _total = 0
    local _counter = 0
    local _target = 0
    local _current = 0

    -- update the controller.
    function self.update(target, current)
        local now = millis():tofloat() * 0.001
        if not _last_t then
            _last_t = now
        end
        local dt = now - _last_t
        _last_t = now
        local err = target - current
        _counter = _counter + 1

        local P = _kP * err
        if ((_total < _max and _total > _min) or (_total >= _max and err < 0) or (_total <= _min and err > 0)) then
            _I = _I + _kI * err * dt
        end
        if _iMax then
            _I = constrain(_I, -_iMax, iMax)
        end
        local I = _I
        local ret = P + I

        _target = target
        _current = current
        _P = P

        ret = constrain(ret, _min, _max)
        _total = ret
        return ret
    end

    -- reset integrator to an initial value
    function self.reset(integrator)
        _I = integrator
    end

    function self.set_I(I)
        _kI = I
    end

    function self.set_P(P)
        _kP = P
    end

    function self.set_Imax(Imax)
        _iMax = Imax
    end

    -- log the controller internals
    function self.log(name, add_total)
        -- allow for an external addition to total
        ---@diagnostic disable-next-line: param-type-mismatch
        logger.write(name, 'Targ,Curr,P,I,Total,Add', 'ffffff', _target, _current, _P, _I, _total, add_total)
    end

    -- return the instance
    return self
end

local function TrackSpot_from_StateCurrent(state)
    local ts = track.TrackSpot(state.distance_NE:x(), state.distance_NE:y(), state:vel_bearing())
    return ts
end

local function Location_from_TrackSpot(state, spot)
    local loc = state:loc_origin():copy()
    loc:offset(spot:n(), spot:e())
    return loc
end

local function distance_to_spot(state, spot)
    local dn = spot:n() - state.distance_NE:x()
    local de = spot:e() - state.distance_NE:y()
    return math.sqrt(dn * dn + de * de)
end

local function bearing_to_spot(state, spot)
    local dn = spot:n() - state.distance_NE:x()
    local de = spot:e() - state.distance_NE:y()
    return math.atan(de, dn)
end

local function PositionControlFactory(radius_min, radius_max)
    local curvature_min = 1 / radius_max
    local curvature_max = 1 / radius_min

    return function(t_path, t_path_last, state_now, arc_now, spot_now)
        local curvature           = arc_now:k()

        local curvature_direction = 1 -- clockwise
        local p4                  = 0
        if curvature < 0 then
            curvature_direction = -1 -- counter clockwise
            p4 = 1
            curvature = -curvature
        end

        if curvature > curvature_max then
            curvature = curvature_max
        elseif curvature < curvature_min then
            curvature = curvature_min
        end

        local center_bearing = curvature_direction * math.pi / 2
        center_bearing = wrap_angle.rad_2pi(spot_now:theta() + center_bearing)

        local radius = 0.785 / curvature
        local radius_scaled = radius / ahrs:get_EAS2TAS() ^ 2
        local loc_center = Location_from_TrackSpot(state_now, spot_now)
        loc_center:offset_bearing(math.deg(center_bearing), radius)

        gcs:run_command_int(MAV_CMD_DO_REPOSITION, {
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT,
            p3 = radius_scaled,
            p4 = p4,
            x = loc_center:lat(),
            y = loc_center:lng(),
            z = 100
        })

        -- gcs_send(string.format(
        --     "s:%.0f, k:%.4f, arc(%.1f, %.1f) b(%.1f, %.1f), c(%.0f, %.0f) spot(%.1f, %.1f) n(%.0f, %.0f)",
        --     arc_now:s(), arc_now:k(),
        --     p_spot_n(arc_now:start_spot()), p_spot_e(arc_now:start_spot()),
        --     math.deg(spot_now:theta()), math.deg(center_bearing),
        --     p_loc_n(loc_center), p_loc_e(loc_center),
        --     p_spot_n(spot_now), p_spot_e(spot_now),
        --     p_loc_n(state_now:loc()), p_loc_e(state_now:loc())))

        ---@diagnostic disable-next-line: param-type-mismatch
        logger.write("TRRP", 'RadD,RadS,CrvD,CrvA', 'ffff', radius, radius_scaled, curvature,
            state_now:curvature(t_path - t_path_last))
    end
end

local time_last_message = uint32_t(0)

local function SpeedControlFactory(kP, kI, iMax, spd_min, spd_mid, spd_max)
    local pi_controller = PI_controller(kP, kI, iMax, spd_min - spd_mid, spd_max - spd_mid)

    return function(t_path, state_now, arc_now, spot_now)
        local dist_to_spot = distance_to_spot(state_now, spot_now)
        local bear_to_spot = bearing_to_spot(state_now, spot_now)

        -- The angle between desired heading and vector to spot
        local alpha = wrap_angle.rad_pi(bear_to_spot - spot_now:theta())
        local el = dist_to_spot * math.cos(alpha)
        local et = dist_to_spot * math.sin(alpha)
        local u = pi_controller.update(0, el)

        -- speed_desired -
        local speed_desired = spd_mid - u

        -- p1 = type (SPEED_TYPE_AIRSPEED)
        -- p2 = airspeed
        -- p3 = max airspeed accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_SPEED, {
            p1 = SPEED_TYPE_AIRSPEED,
            p2 = speed_desired,
            p3 = 1000,
        })

        -- error longitudinal (el): positive -> hehind of spot, negative -> ahead spot
        -- error tangential (et): positive -> to right of vector to spot, negative -> to left
        -- gcs_send(string.format(
        --     "el:%.1f, et:%.1f, a:%.0f, spot(%.1f, %.1f) n(%.0f, %.0f), s:%.0f, k:%.3f, t:%.2f",
        --     el, et, math.deg(alpha),
        --     p_spot_n(spot_now), p_spot_e(spot_now),
        --     p_loc_n(state_now:loc()), p_loc_e(state_now:loc()),
        --     arc_now:s(), arc_now:k(), t_path))

        local time_this_message = millis()
        if time_this_message - time_last_message > 2000 then
            time_last_message = time_this_message
            gcs_send(string.format("el:%.1f, et:%.1f", el, et))
        end

        ---@diagnostic disable-next-line: param-type-mismatch
        logger.write("TRRS", 'El,U,SpdD,SpdA,Et,E,SptN,SptE,SptT,LocN,LocE,LocT', 'ffffffffffff', el, u, speed_desired,
            state_now:speed(), et, dist_to_spot,
            p_spot_n(spot_now), p_spot_e(spot_now), math.deg(spot_now:theta()),
            p_loc_n(state_now:loc()), p_loc_e(state_now:loc()), math.deg(state_now:vel_bearing()))
    end
end

local function AltitudeControlFactory(alt_target)
    return function()
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_ALTITUDE, {
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT,
            z = alt_target,
            p3 = 100,
        })
    end
end

local function PathTimeFactory()
    local time_start = millis():tofloat() * 0.001
    return function(reset_time_adjustment)
        local time_now = millis():tofloat() * 0.001
        if not reset_time_adjustment then
            return time_now - time_start
        end
        time_start = time_start - reset_time_adjustment
        if time_now - time_start > 0 then
            time_start = time_now
        end
        return time_now - time_start
    end
end

local function Guider()
    local loc_home = ahrs:get_home()
    if not loc_home then
        gcs_send("Guider: get_home() failed")
        return nil
    end
    p_setup(loc_home)

    local time_path = PathTimeFactory()

    local state_last = ahrs_state.StateLastFactory(loc_home)
    if not state_last then
        gcs_send("Guider: StateCurrent() failed")
        return nil
    end

    local option_switch = rc:find_channel_for_option(TRANSMITTER_SWITCH_CODE)
    if not option_switch then
        gcs_send("Guider: find_channel_for_option failed")
        return nil
    end


    local saved_mode = vehicle:get_mode()
    vehicle:set_mode(PLANE_MODE_GUIDED)

    local function finish()
        vehicle:set_mode(saved_mode)
    end

    local figure_8 = track.BuildFigureEightFactory(3)
    local this_track = track.Track(figure_8, figure_8, figure_8, figure_8)
    -- local track_line = track.Track({ { 1, 0 } })
    -- local this_track = track.Track(track_line, track_line, track_line, track_line)
    -- local track_circle = track.Track({ { 2 * math.pi, 2 } })
    -- local this_track = track.Track(track_circle, track_circle, track_circle, track_circle)
    -- local track_2circle = track.Track({ { math.pi, 2 }, { math.pi, -2 } })
    -- local this_track = track.Track(track_2circle, track_2circle, track_2circle, track_2circle)
    local spot_home = track.TrackSpot(0, 0, math.pi * 0.2)
    this_track:set_transform(spot_home, 100, params.spd.mid, 0)

    local position_control = PositionControlFactory(
        params.pos.radius_min, params.pos.radius_max)

    local speed_control = SpeedControlFactory(
        params.spd.pi_kP, params.spd.pi_kI, params.spd.pi_iMax,
        params.spd.min, params.spd.mid, params.spd.max)

    local altitude_control = AltitudeControlFactory(params.alt.target)

    local final_s = this_track:end_s()

    local arc_first = this_track:arc_along_track(0)
    local arc_now = arc_first
    local skip_update_count = 0
    local t_path_last = time_path()

    local function skip_update(t_path)
        local function return_noskip()
            skip_update_count = 0
            return false
        end
        local function return_test_skip()
            if skip_update_count > params.tim.update_skip_max then
                return return_noskip()
            end
            skip_update_count = skip_update_count + 1
            return true
        end

        local function process_first_arc_only()
            -- not at the end of current arc
            if t_path < arc_now:end_s() then
                if arc_now == arc_first then
                    return return_test_skip()
                end
                -- reset to first arc, reset time to now
                t_path_last = time_path(0)
                arc_now = this_track:arc_along_track(0)
                return return_noskip()
            end

            -- ready to move to next arc
            -- if already on first arc, don't advance, just reset time so t=0
            -- happens when the plane reaches the track start (negative times)
            if arc_now == arc_first then
                t_path_last = time_path(arc_now:s_back_one_period())
                return return_noskip()
            end

            -- low probability case: changed to first_arc_only mode at the exact
            -- instant when the plane transitions from non-first arc.
            -- reset to first arc, reset time to now
            t_path_last = time_path(0)
            arc_now = this_track:arc_along_track(0)
            return return_noskip()
        end

        local first_arc_only = option_switch:get_aux_switch_pos() == 1
        if first_arc_only then
            return process_first_arc_only()
        end

        -- process normal mode - not in first_arc_only mode.
        if t_path >= arc_now:end_s() then
            -- move to the next arc if not at the end of the track
            if t_path <= final_s then
                arc_now = this_track:arc_along_track(t_path)
                return return_noskip()
            end

            -- stay with the last arc if at the end of the track.
        end

        return return_test_skip()
    end


    return function(abort)
        if abort then
            finish()
            return false
        end

        local t_path = time_path()
        if skip_update(t_path) then
            return true
        end

        local state_now = ahrs_state.StateCurrentFactory(state_last)
        if not state_now then
            gcs_send("Error Couldn't get state_now")
            return false
        end

        local spot_now = arc_now:along_arc(t_path)

        position_control(t_path, t_path_last, state_now, arc_now, spot_now)
        speed_control(t_path, state_now, arc_now, spot_now)
        altitude_control()

        state_last = state_now
        t_path_last = t_path
        return true
    end
end


return stuf.SwitchTriggerUpdateFunction(Guider, GUIDING_TIME_MS, TRANSMITTER_SWITCH_CODE)
