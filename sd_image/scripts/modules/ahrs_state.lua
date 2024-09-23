local function define_classes(gcs_send, wrap_angle)
    local function StateCurrentClass(_gcs_send)
        local gcs_send = _gcs_send

        local cls = {}
        cls.__index = cls

        local IDX_STATE_START = 1
        local IDX_STATE_LAST = 2

        local function new(state_start, state_last, loc_arg)
            if state_last and state_last:state_start() ~= state_start then
                gcs_send("Error: state_last:state_start() ~= state_start.")
                return nil
            end

            local self = setmetatable({
                state_start,
                state_last,
            }, cls)

            if not state_start then
                self[IDX_STATE_START] = self
            end
            if not state_last then
                self[IDX_STATE_LAST] = self
            else
                -- Prevent a chain of last objects from using up all of memory
                -- and thwarting the garbage collector.
                state_last:clear_last()
            end

            -- if loc_arg is passed in then use it as the loc for this StateCurrentClass
            self.loc_cur = loc_arg
            self.vel_cur_2mps = Vector2f()
            self.vel_cur_2mps:x(1)
            self.vel_cur_2mps:y(0)
            if not loc_arg then
                self.loc_cur = ahrs:get_location()
                local vel_cur_vmps = ahrs:get_velocity_NED()
                if not self.loc_cur or not vel_cur_vmps then
                    gcs_send("Error: cannot get location.")
                    return nil
                end
                self.vel_cur_2mps:x(vel_cur_vmps:x())
                self.vel_cur_2mps:y(vel_cur_vmps:y())
            end
            self.loc_cur:change_alt_frame(1)

            self.distance_NE = self[IDX_STATE_START]:loc():get_distance_NE(self.loc_cur)

            self.time_cur = millis():tofloat() * 0.001

            return self
        end

        function cls.vel_bearing(self) return self.vel_cur_2mps:angle() end

        function cls.speed(self) return self.vel_cur_2mps:length() end

        function cls.time(self) return self.time_cur end

        function cls.time_delta(self) return self:time() - self[IDX_STATE_LAST]:time() end

        function cls.speed_avg(self) return (self:speed() + self[IDX_STATE_LAST]:speed()) / 2 end

        function cls.curvature(self)
            local time_delta_tmp = self:time_delta()
            if time_delta_tmp == 0 then
                return 0
            end
            return wrap_angle.rad_pi(self:vel_bearing() - self[IDX_STATE_LAST]:vel_bearing()) / time_delta_tmp /
                self:speed_avg()
        end

        function cls.loc(self) return self.loc_cur end

        function cls.alt(self) return self.loc_cur:alt() * 0.01 end

        function cls.clear_last(self) self[IDX_STATE_LAST] = nil end

        function cls.state_start(self) return self[IDX_STATE_START] end

        function cls.time_total(self) return self.time_cur - self[IDX_STATE_START]:time() end

        function cls.distance_total(self) return self[IDX_STATE_START]:loc():get_distance(self.loc_cur) end

        function cls.bearing_total(self) return self[IDX_STATE_START]:loc():get_bearing(self.loc_cur) end

        return new
    end


    local function StateClass()
        local cls = {}
        cls.__index = cls

        local IDX_LOC = 1
        local IDX_LOC_ORIGIN = 2
        local IDX_STATE_START = 3
        local IDX_STATE_LAST = 4

        local function new(loc, velocity_NED, state_last, loc_origin)
            if state_last then
                loc_origin = state_last:loc_origin()
            end

            if not loc_origin or not loc or not velocity_NED then
                gcs_send("StateClass arg error.")
                return nil
            end

            local vel_cur_2mps = Vector2f()
            vel_cur_2mps:x(velocity_NED:x())
            vel_cur_2mps:y(velocity_NED:y())

            local self = setmetatable({
                loc:copy(),
                loc_origin,
                state_last,
                state_last,
                vel_cur_2mps = vel_cur_2mps,
                distance_NE = loc_origin:get_distance_NE(loc),
                time_cur = millis():tofloat() * 0.001,
            }, cls)

            if not state_last then
                self[IDX_STATE_START] = self
                self[IDX_STATE_LAST] = self
            else
                self[IDX_STATE_START] = state_last[IDX_STATE_START]
                -- Prevent a chain of last objects from using up all of memory
                -- and thwarting the garbage collector.
                state_last[IDX_STATE_LAST] = nil
            end

            self[IDX_LOC]:change_alt_frame(1)

            return self
        end

        cls.curvature = function(self)
            local time_delta_tmp = self:time_delta()
            if time_delta_tmp == 0 then
                return 0
            end
            return wrap_angle.rad_pi(self:vel_bearing() - self[IDX_STATE_LAST]:vel_bearing()) / time_delta_tmp /
                self:speed_avg()
        end

        cls.vel_bearing = function(self) return self.vel_cur_2mps:angle() end
        cls.speed = function(self) return self.vel_cur_2mps:length() end
        cls.time = function(self) return self.time_cur end
        cls.time_delta = function(self) return self:time() - self[IDX_STATE_LAST]:time() end
        cls.speed_avg = function(self) return (self:speed() + self[IDX_STATE_LAST]:speed()) / 2 end
        cls.loc = function(self) return self[IDX_LOC] end
        cls.loc_origin = function(self) return self[IDX_LOC_ORIGIN] end
        cls.time_total = function(self) return self.time_cur - self[IDX_STATE_START]:time() end
        cls.distance_total = function(self) return self[IDX_STATE_START]:loc():get_distance(self.loc) end
        cls.bearing_total = function(self) return self[IDX_STATE_START]:loc():get_bearing(self.loc) end

        return new
    end

    local StateClassFactory = StateClass()

    local StateCurrentFactory = function(state_last)
        if not state_last then
            gcs_send("StateCurrentFactory: arg error.")
            return nil
        end
        local loc = ahrs:get_location()
        local velocity_NED = ahrs:get_velocity_NED()
        if not loc or not velocity_NED then
            gcs_send("StateCurrentFactory: cannot get location.")
            return nil
        end
        return StateClassFactory(loc, velocity_NED, state_last)
    end

    local StateStartFactory = function(loc_origin)
        if not loc_origin then
            gcs_send("StateStartFactory arg error.")
            return nil
        end
        local loc = ahrs:get_location()
        local velocity_NED = ahrs:get_velocity_NED()
        if not loc or not velocity_NED then
            gcs_send("StateStartFactory: cannot get location.")
            return nil
        end
        return StateClassFactory(loc, velocity_NED, nil, loc_origin)
    end

    return {
        StateCurrent = StateCurrentClass(),
        StateCurrentFactory = StateCurrentFactory,
        StateStartFactory = StateStartFactory,
    }
end

return define_classes
