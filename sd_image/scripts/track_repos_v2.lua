local gcs_send                          = require("gcs_send_funcfactory")("TTR")
local wrap_angle                        = require("wrap_angle_obj")
local stuf                              = require("switch_trigger_update_function")("track_repos_v2", gcs_send)
local track                             = require("track_obj")(gcs_send, wrap_angle)
local ahrs_state                        = require("ahrs_state")(gcs_send, wrap_angle)
local StateCurrent                      = ahrs_state.StateCurrent

local GUIDING_TIME_MS                   = 300

local PLANE_MODE_GUIDED                 = 15

local MAV_CMD_DO_REPOSITION             = 192
local MAV_CMD_GUIDED_CHANGE_SPEED       = 43000
local MAV_CMD_GUIDED_CHANGE_ALTITUDE    = 43001
local MAV_CMD_GUIDED_CHANGE_HEADING     = 43002

local MODE_GUIDED                       = 15

local MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

local HEADING_TYPE_HEADING              = 1
local SPEED_TYPE_AIRSPEED               = 0
local MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
local MAV_FRAME_GLOBAL_RELATIVE_ALT     = 3

local params = {
    spd = {
        min = 10,
        mid = 15,
        max = 20,
        pi_kP = .25,
        pi_kI = .025,
        pi_iMax = 4,
        pi_min = -4,
        pi_max = 4,
    },
    pos = {
        radius_min = 40,
        radius_max = 1000,
    },
}

local p_loc_n = function(loc) return 0 end
local p_loc_e = function(loc) return 0 end
local p_spot_n = function(spot) return 0 end
local p_spot_e = function(spot) return 0 end

local function p_setup(state_start, loc_origin)
    local function get_loc(spot)
        local loc = state_start:loc():copy()
        loc:offset(-spot:n(), -spot:e())
        return loc
    end
    p_loc_n = function(loc) return loc_origin:get_distance_NE(loc):x() end
    p_loc_e = function(loc) return loc_origin:get_distance_NE(loc):y() end
    p_spot_n = function(spot) return spot:n() end
    p_spot_e = function(spot) return spot:e() end
end

local function p_lat(lat)
    return lat - -353632620
end
local function p_lng(lng)
    return lng - 1491652372
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
    local dn = state.distance_NE:x() - spot:n()
    local de = state.distance_NE:y() - spot:e()
    return math.sqrt(dn * dn + de * de)
end

local function bearing_to_spot(state, spot)
    local dn = state.distance_NE:x() - spot:n()
    local de = state.distance_NE:y() - spot:e()
    return math.atan(de, dn)
end

local function PositionControlFactory(radius_min, radius_max)
    local curvature_min = 1 / radius_max
    local curvature_max = 1 / radius_min

    local function position_control(state_now, arc_now, spot_now)
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
        -- local radius_scaled = radius
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
    end

    return position_control
end

local function SpeedControlFactory(kP, kI, iMax, spd_min, spd_mid, spd_max)
    local pi_controller = PI_controller(kP, kI, iMax, spd_min - spd_mid, spd_max - spd_mid)

    local function speed_control(state_now, spot_now)
        local dist_to_spot = distance_to_spot(state_now, spot_now)
        local bear_to_spot = bearing_to_spot(state_now, spot_now)

        -- The angle between desired heading and vector to spot
        local alpha = bear_to_spot - spot_now:theta()
        local e = dist_to_spot * math.cos(alpha)
        local u = pi_controller.update(0, e)

        -- speed_desired -
        local speed_desired = spd_mid + u

        -- p1 = type (SPEED_TYPE_AIRSPEED)
        -- p2 = airspeed
        -- p3 = max airspeed accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_SPEED, {
            p1 = SPEED_TYPE_AIRSPEED,
            p2 = speed_desired,
            p3 = 1000,
        })

        gcs_send(string.format(
            "el:%.1f, et:%.1f, spot(%.1f, %.1f) n(%.0f, %.0f)",
            e, dist_to_spot * math.sin(alpha),
            p_spot_n(spot_now), p_spot_e(spot_now),
            p_loc_n(state_now:loc()), p_loc_e(state_now:loc())))
    end

    return speed_control
end


local function Guider()
    local loc_home = ahrs:get_home()
    if not loc_home then
        gcs_send("Guider: get_home() failed")
        return nil
    end
    local state_start = ahrs_state.StateStartFactory(loc_home)
    local state_last = ahrs_state.StateCurrentFactory(state_start)
    p_setup(state_start, loc_home)

    if not state_start or not state_last then
        gcs_send("Guider: StateCurrent() failed")
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
    -- local track_circle = track.Track({ { 2 * math.pi, 1 } })
    -- local this_track = track.Track(track_circle, track_circle, track_circle, track_circle)
    -- local track_2circle = track.Track({ { 2 * math.pi, 1 }, { 2 * math.pi, -1 } })
    -- local this_track = track.Track(track_2circle, track_2circle, track_2circle, track_2circle)
    local spot_home = track.TrackSpot(0, 0, math.pi * 0.2)
    this_track:set_transform(spot_home, 100, params.spd.mid, 0)

    local position_control = PositionControlFactory(
        params.pos.radius_min, params.pos.radius_max)

    local speed_control = SpeedControlFactory(
        params.spd.pi_kP, params.spd.pi_kI, params.spd.pi_iMax,
        params.spd.min, params.spd.mid, params.spd.max)

    gcs_send(string.format("Home:%i, %i", p_loc_n(loc_home), p_loc_e(loc_home)))

    return function(abort)
        if abort then
            finish()
            return false
        end

        local state_now = ahrs_state.StateCurrentFactory(state_last)
        if not state_now then
            gcs_send("Error Couldn't get state_now")
            return false
        end

        local t = state_now:time_total()
        local arc_now = this_track:arc_along_track(t)
        local spot_now = arc_now:along_arc(t)

        position_control(state_now, arc_now, spot_now)
        speed_control(state_now, spot_now)

        state_last = state_now
        return true
    end
end


return stuf.SwitchTriggerUpdateFunction(Guider, GUIDING_TIME_MS, 300)
