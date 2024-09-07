local gcs_send          = require("gcs_send_funcfactory")("TTR")
local wrap_angle        = require("wrap_angle_obj")
local stuf              = require("switch_trigger_update_function")(gcs_send)
local track             = require("track_obj")(gcs_send, wrap_angle)
local StateCurrent      = require("ahrs_state")(gcs_send, wrap_angle).StateCurrent

local GUIDING_TIME_MS   = 300

local PLANE_MODE_GUIDED = 15

local MAV_CMD_DO_SET_MODE = 176
local MAV_CMD_GUIDED_CHANGE_SPEED = 43000
local MAV_CMD_GUIDED_CHANGE_ALTITUDE = 43001
local MAV_CMD_GUIDED_CHANGE_HEADING = 43002

local MODE_GUIDED = 15

local MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

local HEADING_TYPE_HEADING = 1
local SPEED_TYPE_AIRSPEED = 0
local MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
local MAV_FRAME_GLOBAL_RELATIVE_ALT = 3


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


local function speed_controller(kP_param, kI_param, kFF_pitch_param, Imax, min, max)
    local self = {}
    local kFF_pitch = kFF_pitch_param
    local PI = PI_controller(kP_param:get(), kI_param:get(), Imax, min, max)
    local k_throttle = 70


    function self.update(target, anticipated_pitch_rad)
        local current_speed = ahrs:get_velocity_NED():length()
        local throttle = PI.update(target, current_speed)
        local FF = math.sin(anticipated_pitch_rad) * kFF_pitch:get()
        PI.log("AESP", FF)
        return throttle + FF
    end

    function self.reset()
        PI.reset(0)
        local temp_throttle = self.update(ahrs:get_velocity_NED():length(), 0)
        local current_throttle = SRV_Channels:get_output_scaled(k_throttle)
        PI.reset(current_throttle - temp_throttle)
    end

    return self
end

-- local speed_PI = speed_controller(SPD_P, SPD_I, THR_PIT_FF, 100.0, 0.0, 100.0)

local function speed_adjust_controller(kP_param, kI_param)
    local self = {}
    local spd_max = AEROM_TS_SPDMAX:get()
    local PI = PI_controller(kP_param:get(), kI_param:get(), spd_max, -spd_max, spd_max)

    function self.update(spd_error)
        local adjustment = PI.update(0, spd_error)
        PI.log("AESA", 0)
        return adjustment
    end

    function self.reset()
        PI.reset(0)
    end

    return self
end

-- local speed_adjustment_PI = speed_adjust_controller(AEROM_TS_P, AEROM_TS_I)


local function TrackSpot_from_StateCurrent(state)
    local ts = track.TrackSpot(state.distance_NE:x(), state.distance_NE:y(), state:vel_bearing())
    return ts
end

local function distance_from_spot(state, spot)
    local dn = state.distance_NE:x() - spot:n()
    local de = state.distance_NE:y() - spot:e()
    return math.sqrt(dn * dn + de * de)
end

local function Guider()

    local state_start = StateCurrent()
    local state_last = StateCurrent(state_start)

    if not state_start or not state_last then
        gcs_send("StateCurrent() failed")
        return nil
    end

    local saved_mode = vehicle:get_mode()
    vehicle:set_mode(PLANE_MODE_GUIDED)
    -- gcs:run_command_int(MAV_CMD_DO_SET_MODE, { p1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, p2 = MODE_GUIDED })

    local function finish()
        vehicle:set_mode(saved_mode)
    end

    local pi_controller = PI_controller(.25, .025, 4, -4, 4)

    local this_track = track.Track{{1, 0}}
    this_track:dump()
    local spot_start = TrackSpot_from_StateCurrent(state_start)
    this_track:set_transform(spot_start, 100, 25, 0)

    local test_heading = math.deg(wrap_angle.rad_2pi(state_start:vel_bearing()))
    gcs_send(string.format("ber:%0f", test_heading))
    
    return function(abort)

        if abort then
            finish()
            return false
        end

        local state_now = StateCurrent(state_start, state_last)
        if not state_now then
            gcs_send("Error Couldn't get state_now")
            return false
        end

        local t = state_now:time_total()
        local spot_now = this_track:along_track(t)
        -- dist_now - distance between target point and now point
        local dist_error = distance_from_spot(state_now, spot_now)

        -- calculate speed error based on position
        -- project to target location
        local dist_projected = 50
        local spot_projected = track.TrackSpot(
            dist_projected * math.cos(spot_now:theta()) + spot_now:n(),
            dist_projected * math.sin(spot_now:theta()) + spot_now:e(),
            spot_now:theta())
        -- dist_targ - distance between projected point and now point
        local dist_targ = distance_from_spot(state_now, spot_projected)

        -- e, u - the input and output to the PI controller
        local e = dist_targ - dist_projected
        local u = pi_controller.update(0, e)

        -- speed_desired - 
        local speed_desired = 25 - u
        -- dist_spot - distance from start to target spot
        local dist_spot = distance_from_spot(state_start, spot_now)
        -- dist_now - distance from start to current 
        local dist_now = state_now:distance_total()
        -- bearing_now_error - angle offset from start to now from track
        local bearing_now_error = wrap_angle.deg_180(math.deg(state_now:bearing_total()) - test_heading)

        -- Calculate Data to log.
        -- dist_now - distance between target point and now point

        -- p1 = type (SPEED_TYPE_AIRSPEED)
        -- p2 = airspeed
        -- p3 = max airspeed accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_SPEED, {
            p1 = SPEED_TYPE_AIRSPEED,
            p2 = speed_desired,
            p3 = 1000,
            })

        -- frame = type (MAV_FRAME_GLOBAL_RELATIVE_ALT)
        -- z = altitude 
        -- p3 = max accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_ALTITUDE, {
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT,
            z = 100,
            p3 = 100,
            })

        -- p1 = type (GUIDED_HEADING_NONE=0, GUIDED_HEADING_COG=1, GUIDED_HEADING_HEADING=2)
        -- p2 = heading in degrees
        -- p3 = max accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_HEADING, {
            p1 = HEADING_TYPE_HEADING,
            p2 = test_heading,
            p3 = 10,
            })


        gcs_send(string.format("t:%.2f, d:%.0f, n(%.0f, %.0f), pi(%.2f, %.2f), spd(d:%.1f, a:%.1f) ", t, dist_spot,
            dist_now, bearing_now_error, e, u, speed_desired, state_now:speed()))

        state_last = state_now
        -- Have to release the reference to state_last. Otherwise none of the state objects 
        -- are freed and memory is filled.
        state_last:clear_last()

        return true
    end
end

gcs_send("Loaded track_speed.lua")

return stuf.SwitchTriggerUpdateFunction("track_speed", Guider, GUIDING_TIME_MS, 300)
