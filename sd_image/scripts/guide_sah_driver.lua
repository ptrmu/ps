

local gcs_send = require("gcs_send_funcfactory")("GSAH")
local wrap_angle = require("wrap_angle_obj")
local switch_exec_updatefactory = require("switch_exec_updatefactory")


local GUIDING_TIME_MS = 200

local PLANE_MODE_GUIDED        = 15

local MAV_CMD_GUIDED_CHANGE_SPEED = 43000
local MAV_CMD_GUIDED_CHANGE_ALTITUDE = 43001
local MAV_CMD_GUIDED_CHANGE_HEADING = 43002

local HEADING_TYPE_COURSE_OVER_GROUND = 0
local HEADING_TYPE_HEADING = 1
local SPEED_TYPE_AIRSPEED = 0
local MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
local MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

local function find_channel(selector, attribute)
    local slider = rc:find_channel_for_option(selector)
    if not slider then
        gcs_send(string.format("Error: no RC channel for selector %i to control %s", selector, attribute))
    end
    return slider
end

local speed_slider = find_channel(301, "speed")
local altitude_slider = find_channel(302, "altitude")
local curvature_slider = find_channel(301, "curvature")
if not speed_slider or not altitude_slider or not curvature_slider then
    return nil, 0
end

local roll_limit_deg = Parameter("ROLL_LIMIT_DEG"):get()
if not roll_limit_deg then
    gcs_send("Cound not find parameter ROLL_LIMIT_DEG")
    return nil, 0
end
-- Calculate a max curvature that is reasonable for a speed of 25 mps
local lateral_acceleration_max = 9.8 * math.atan(math.rad(roll_limit_deg))

local function make_class()
    local cls = {}
    cls.__index = cls
    return cls
end
local _StateCurrentClass = make_class()

local function StateCurrent(state_start_arg, state_last_arg)

    if state_last_arg and state_last_arg:state_start() ~= state_start_arg then
        gcs_send("Error: state_last:state_start() ~= state_start.")
        return nil
    end

    -- local _self = {
    --     _state_start = state_start_arg,
    --     _state_last = state_last_arg,
    -- }

    local _self = setmetatable({
        _state_start = state_start_arg,
        _state_last = state_last_arg,
        }, _StateCurrentClass)

    _self.loc_cur = ahrs:get_location()
    _self.vel_cur_vmps = ahrs:get_velocity_NED()
    if not _self.loc_cur or not _self.vel_cur_vmps then
        gcs_send("Error: cannot get location.")
        return nil
    end
    _self.loc_cur:change_alt_frame(1)

    _self.vel_cur_2mps = Vector2f()
    _self.vel_cur_2mps:x(_self.vel_cur_vmps:x())
    _self.vel_cur_2mps:y(_self.vel_cur_vmps:y())

    _self.time_cur = millis():tofloat() * 0.001

    function _StateCurrentClass.vel_bearing(self)
        return self.vel_cur_2mps:angle()
    end

    function _StateCurrentClass.speed(self)
        return self.vel_cur_2mps:length()
    end

    function _StateCurrentClass.time(self)
        return self.time_cur
    end

    function _StateCurrentClass.time_delta(self)
        return self:time() - self._state_last:time()
    end

    function _StateCurrentClass.speed_avg(self)
        return (self:speed() + self._state_last:speed()) / 2
    end

    function _StateCurrentClass.curvature(self)
        local time_delta_tmp = self:time_delta()
        if time_delta_tmp == 0 then
            return 0
        end
        return wrap_angle.rad_pi(self:vel_bearing() - self._state_last:vel_bearing()) / time_delta_tmp / self:speed_avg()
    end

    function _StateCurrentClass.loc(self) return self.loc_cur:copy() end
    function _StateCurrentClass.alt(self) return self.loc_cur:alt() * 0.01 end
    function _StateCurrentClass.clear_last(self) self._state_last = nil end
    function _StateCurrentClass.state_start(self) return self._state_start end
    function _StateCurrentClass.time_total(self) return self.time_cur - self._state_start:time() end

    return _self
    -- return {
    --     vel_bearing = vel_bearing,
    --     speed = speed,
    --     time = time,
    --     loc = function() return self.loc_cur:copy() end,
    --     alt = function() return self.loc_cur:alt() * 0.01 end,

    --     time_delta = time_delta,
    --     speed_avg = speed_avg,
    --     curvature = curvature,
    --     clear_last = function() self.state_last = nil end,

    --     state_start = function() return self.state_start end,
    --     time_total = function() return self.time_cur - self.state_start.time() end,
    -- }
end


local function Guider()

    local state_start = StateCurrent()
    local state_last = StateCurrent(state_start)

    if not state_start then
        return nil
    end

    local saved_mode = vehicle:get_mode()
    vehicle:set_mode(PLANE_MODE_GUIDED)

    local function finish()
        vehicle:set_mode(saved_mode)
    end

    local count = 0

    return function(abort)

        if abort then
            finish()
            return false
        end

        if vehicle:get_mode() ~= PLANE_MODE_GUIDED then
            gcs_send("Terminating because mode ~= GUIDE")
            return false
        end

        local state_now = StateCurrent(state_start, state_last)
        if not state_now then
            return false
        end

        -- Set speed
        local speed_desired = ({22, 25, 28})[speed_slider:get_aux_switch_pos()+1]

        -- p1 = type (SPEED_TYPE_AIRSPEED)
        -- p2 = airspeed
        -- p3 = max airspeed accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_SPEED, {
            p1 = SPEED_TYPE_AIRSPEED,
            p2 = speed_desired,
            p3 = 20,
            })

        -- Set altitude
        local altitude_desired = ({80, 100, 120})[altitude_slider:get_aux_switch_pos()+1]

        -- frame = type (MAV_FRAME_GLOBAL_RELATIVE_ALT)
        -- z = altitude 
        -- p3 = max accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_ALTITUDE, {
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT,
            z = altitude_desired,
            p3 = 100,
            })

        -- Set curvature
        local speed2 = state_now:speed() * state_now:speed()
        local curvature_max = lateral_acceleration_max / speed2

        local curvature_input  = curvature_slider:norm_input()
        local curvature_direction = 1   -- clockwise
        if curvature_input < 0 then
            curvature_direction = -1    -- counter clockwise
        end
        local curvature_desired = curvature_input * curvature_input * curvature_max -- add expo

        -- lateral acceleration is positive because curvature_desired is positive
        -- 0.75 is an empirical adjustment factor
        local lateral_acceleration_desired = curvature_desired * speed2 / 0.75
        curvature_desired = curvature_desired * curvature_direction

        local bearing_new = 90 * curvature_direction
        bearing_new = wrap_angle.deg_360(math.deg(state_now:vel_bearing()) + bearing_new)

        -- p1 = type (GUIDED_HEADING_NONE=0, GUIDED_HEADING_COG=1, GUIDED_HEADING_HEADING=2)
        -- p2 = heading in degrees
        -- p3 = max accel
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_HEADING, {
            p1 = HEADING_TYPE_COURSE_OVER_GROUND,
            p2 = bearing_new,
            p3 = lateral_acceleration_desired,
            })


        count = count + 1
        gcs_send(string.format("%03i, spd(d:%.1f, a:%.1f) alt(d:%.1f, a:%.1f), crv(i:%.2f, d:%.4f, a:%.4f)",
            count, speed_desired, state_now:speed(), altitude_desired, state_now:alt(),
            curvature_input, curvature_desired, state_now:curvature()))

        ---@diagnostic disable-next-line: param-type-mismatch
        logger.write("GSAH", "SpdD,SpdA,AktD,AltA,CrvD,CrvA", "ffffff", speed_desired, state_now:speed(), altitude_desired, 
            state_now:alt(), curvature_desired, state_now:curvature())

        state_last = state_now
        -- Have to release the reference to state_last. Otherwise none of the state objects 
        -- are freed and their memmory collected.
        -- state_last.clear_last()

        return true
    end
end




return (function()
    local r, d = switch_exec_updatefactory("guide_sah_driver", Guider, GUIDING_TIME_MS, 300, gcs_send)

    gcs_send("Loaded guide_sah_driver.lua")

    return r, d
end)()