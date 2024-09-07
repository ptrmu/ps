

local function define_classes(gcs_send, wrap_angle)

    local function StateCurrentClass(_gcs_send)

        local gcs_send = _gcs_send

        local cls = {}
        cls.__index = cls

        local IDX_STATE_START = 1
        local IDX_STATE_LAST = 2

        local function new(state_start, state_last)

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
            end

            self.loc_cur = ahrs:get_location()
            self.vel_cur_vmps = ahrs:get_velocity_NED()
            if not self.loc_cur or not self.vel_cur_vmps then
                gcs_send("Error: cannot get location.")
                return nil
            end
            self.loc_cur:change_alt_frame(1)

            self.vel_cur_2mps = Vector2f()
            self.vel_cur_2mps:x(self.vel_cur_vmps:x())
            self.vel_cur_2mps:y(self.vel_cur_vmps:y())

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
            return wrap_angle.rad_pi(self:vel_bearing() - self[IDX_STATE_LAST]:vel_bearing()) / time_delta_tmp / self:speed_avg()
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

    return {
        StateCurrent = StateCurrentClass(),
    }
    end

return define_classes
