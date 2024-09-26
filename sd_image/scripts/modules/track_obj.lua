local function define_classes(gcs_send, wrap_angle)
    local function TrackSpotClass()
        local cls = {}
        cls.__index = cls

        local IDX_N = 1
        local IDX_E = 2
        local IDX_THETA = 3

        local function new(n, e, theta)
            return setmetatable({ n, e, theta }, cls)
        end

        function cls.n(self) return self[IDX_N] end

        function cls.e(self) return self[IDX_E] end

        function cls.theta(self) return self[IDX_THETA] end

        function cls.set_n(self, n) self[IDX_N] = n end

        function cls.set_e(self, e) self[IDX_E] = e end

        function cls.set_theta(self, theta) self[IDX_THETA] = theta end

        function cls.clone(self) return new(self[IDX_N], self[IDX_E], self[IDX_THETA]) end

        return new
    end

    local TrackSpot = TrackSpotClass()


    -- Using NE coordinates, 0 angle along N axis, clockwise angles are positive
    local function TrackArcClass()
        local cls = {}
        cls.__index = cls

        local IDX_S = 1
        local IDX_K = 2

        local function new(s, k)
            return setmetatable({
                s,
                k,
                p0 = TrackSpot(0, 0, 0),
                s0_scaled = 0,
                size_scale = 1,
                s_scale = 1,
            }, cls)
        end


        -- s - distance along arc
        -- k - curvature = 1/r
        local function along_arc_from_origin(s, k)
            -- N = sin(sk)/k
            -- E = (1-cos(sk))/k
            -- theta = sk
            local n = s
            local e = 0
            local sk = s * k

            -- if k == 0 then straight line and values are set already.
            if k ~= 0.0 then
                -- if k is small, use the series expansion to avoid the divide by zero
                if math.abs(k) < 1.0e-4 then
                    local s2k2 = sk * sk
                    n = s * (s2k2 * (s2k2 * (-s2k2 / 5040.0 + 1.0 / 120.0) - 1.0 / 6.0) + 1.0)
                    e = s * sk * (s2k2 * (s2k2 * (-s2k2 / 40320.0 + 1.0 / 720.0) - 1.0 / 24.0) + 1.0 / 2.0)

                    -- Use desired formulas
                else
                    -- Note Lua has low floating precision. For example sin(pi)=0.00000087 not 0.0
                    n = math.sin(sk) / k
                    e = (1. - math.cos(sk)) / k
                end
            end

            return TrackSpot(n, e, wrap_angle.rad_pi(sk))
        end

        local function scale_spot(self, p)
            local n_temp = self.p0:n() +
                (p:n() * math.cos(self.p0:theta()) - p:e() * math.sin(self.p0:theta())) * self.size_scale
            local e_temp = self.p0:e() +
                (p:n() * math.sin(self.p0:theta()) + p:e() * math.cos(self.p0:theta())) * self.size_scale
            p:set_n(n_temp)
            p:set_e(e_temp)
            p:set_theta(wrap_angle.rad_pi(self.p0:theta() + p:theta()))
            return p
        end


        function cls.along_arc(self, s)
            local s_from_origin = (s - self.s0_scaled) * self.s_scale / self.size_scale
            local k_from_origin = self[IDX_K]
            local p = along_arc_from_origin(s_from_origin, k_from_origin)
            -- gcs_send(string.format("n:%f, e:%f", p:n(), p:e()))
            return scale_spot(self, p)
        end

        function cls.set_transform(self, arc_start, size_scale, s_scale, s_start)
            self.p0 = arc_start:clone()
            self.s0_scaled = s_start
            self.size_scale = size_scale
            self.s_scale = s_scale
        end

        function cls.clone(self)
            local ta = new(self[IDX_S], self[IDX_K])
            ta.set_transform(self.p0, self.size_scale, self.s_scale, self.s0_scaled)
            return ta
        end

        cls.s = function(self) return self[IDX_S] * self.size_scale / self.s_scale end
        cls.k = function(self) return self[IDX_K] / self.size_scale end
        cls.s_raw = function(self) return self[IDX_S] end
        cls.k_raw = function(self) return self[IDX_K] end
        cls.start_s = function(self) return self.s0_scaled end
        cls.end_s = function(self) return self:s() + self.s0_scaled end
        cls.start_spot = function(self) return self:along_arc(self:start_s()) end
        cls.end_spot = function(self) return self:along_arc(self:end_s()) end

        return new
    end

    local TrackArc = TrackArcClass()


    local function TrackClass()
        local cls = {}
        cls.__index = cls


        local function link_arcs(self, track_start, size_scale, s_scale, s_start)
            local p = track_start
            local s = s_start
            for i, arc in ipairs(self) do
                arc:set_transform(p, size_scale, s_scale, s)
                p = arc:end_spot()
                s = arc:end_s()
            end
        end

        local function new(...)
            local self = {}

            for i, arcs in ipairs({ ... }) do
                for i1, arc in ipairs(arcs) do
                    table.insert(self, TrackArc(arc[1], arc[2]))
                end
            end

            if #self < 1 then
                gcs_send("TrackClass Error: Track with no arcs")
            end

            link_arcs(self, TrackSpot(0., 0., 0.), 1, 1, 0)

            self.arc_idx_cached = 1

            return setmetatable(self, cls)
        end

        function cls.set_transform(self, track_start, size_scale, s_scale, s_start)
            link_arcs(self, track_start, size_scale, s_scale, s_start)
        end

        function cls.along_track(self, s)
            self:update_arc_idx_caches(s)
            return self[self.arc_idx_cached]:along_arc(s)
        end

        function cls.arc_along_track(self, s)
            self:update_arc_idx_caches(s)
            return self[self.arc_idx_cached]
        end

        function cls.along_track_ext(self, s, sx)
            local p = self:along_track(s)
            local px = self[self.arc_idx_cached]:along_arc(s + sx)
            return p, px
        end

        function cls.update_arc_idx_caches(self, s)
            local arc_idx = self.arc_idx_cached
            if s < self[arc_idx]:start_s() then
                arc_idx = 1
            end
            while s >= self[arc_idx]:end_s() do
                if arc_idx >= #self then
                    break;
                end
                arc_idx = arc_idx + 1
            end
            self.arc_idx_cached = arc_idx
        end

        function cls.is_complete(self, s)
            self:update_arc_idx_caches(s)
            return self.arc_idx_cached == #self and
                s >= self[self.arc_idx_cached]:end_s()
        end

        cls.end_s = function(self)
            return self[#self]:end_s()
        end

        function cls.dump(self)
            for i, arc in ipairs(self) do
                gcs_send(string.format("i: %i, %.4f, %.4f, %.1f", i,
                    arc:end_spot():n(), arc:end_spot():e(), math.deg(arc:end_spot():theta())))
            end
        end

        return new
    end

    local TrackFactory = TrackClass()

    local function BuildFigureEightFactory()
        -- Create a figure eight that is composed of halfarc, straight, 2*halfarc, straight, halfarc.
        -- The start position/orientation is at start of halfarc, heading north, figure-eight
        -- extends to the east. THe end is the same position/orientation as the start.
        -- The radius of the arcs is 1/2. THe one parameter, Length, is the distance
        -- between the outer points of the curved portions of the figure eight. Length must be greater or equal to 2.
        local function new(length)
            if length < 2 then
                length = 2
            end
            local beta = math.acos(1 / (length - 1))
            local s1 = (math.pi - beta) / 2
            local k1 = 2
            local s2 = (length - 1) * math.sin(beta)
            local k2 = 0

            return TrackFactory({
                { s1,     k1 },
                { s2,     k2 },
                { 2 * s1, -k1 },
                { s2,     k2 },
                { s1,     k1 },
            })
        end

        return new
    end

    return {
        TrackSpot = TrackSpot,
        TrackArc = TrackArc,
        Track = TrackFactory,
        BuildFigureEightFactory = BuildFigureEightFactory(),
    }
end

return define_classes
