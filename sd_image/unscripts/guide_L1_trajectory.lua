

local gcs_send = require("gcs_send_funcfactory")("GAT")
local wrap_angle = require("wrap_angle_obj")
local switch_exec_updatefactory = require("switch_exec_updatefactory")


local GUIDING_TIME_MS = 98

local PLANE_MODE_GUIDED        = 15

local function TrackSpot(n_arg, e_arg, theta_arg)
    local n = n_arg
    local e = e_arg
    local theta = theta_arg

    return {
        n = function() return n end,
        e = function() return e end,
        theta = function() return theta end,
        set_n = function(n_) n = n_ end,
        set_e = function(e_) e = e_ end,
        set_theta = function(theta_) theta = theta_ end,
        copy = function() return TrackSpot(n, e, theta) end,
    }
end

-- Using NE coordinates, 0 angle along N axis, clockwise angles are positive
local function TrackArc(s_arg, k_arg)
    local self = {}

    local s_arc = s_arg
    local k_arc = k_arg

    local p0 = TrackSpot(0., 0., 0.)
    local s0_scaled = 0.

    local size_scale = 1.0
    local s_scale = 1.0


    -- s - distance along arc
    -- k - curvature = 1/r
    local function along_arc_from_origin(s_, k_)
        -- N = sin(sk)/k 
        -- E = (1-cos(sk))/k 
        -- theta = sk
        local n = s_
        local e = 0
        local sk = s_ * k_

        -- if k == 0 then straight line
        if k_ ~= 0.0 then

            -- if k is small, use the series expansion to avoid the divide by zero
            if math.abs(k_) < 1.0e-4 then
                local s2k2 = sk * sk
                n = s_ * (s2k2 * (s2k2 * (-s2k2/5040.0 + 1.0/120.0) - 1.0/6.0) + 1.0)
                e = s_ * sk * (s2k2 * (s2k2 * (-s2k2/40320.0 + 1.0/720.0) - 1.0/24.0) + 1.0/2.0)

            -- Use desired formulas
            else
                -- Note Lua has low floating precision. For example sin(pi)=0.00000087 not 0.0
                n = math.sin(sk) / k_
                e = (1. - math.cos(sk)) / k_
            end
        end

        -- gcs_send(string.format("%.9f, %.9f, %.9f", s_, k_, sk))
        -- gcs_send(string.format("%.9f, %.9f, %.9f", n, e, sk))
        return TrackSpot(n, e, wrap_angle.rad_pi(sk))
    end

    local function scale_spot(p)
        local n_temp = p0.n() + (p.n() * math.cos(p0.theta()) - p.e() * math.sin(p0.theta())) * size_scale
        local e_temp = p0.e() + (p.n() * math.sin(p0.theta()) + p.e() * math.cos(p0.theta())) * size_scale
        p.set_n(n_temp)
        p.set_e(e_temp)
        p.set_theta(wrap_angle.rad_pi(p0.theta() + p.theta()))
        return p
    end
 
    local function along_arc(s)
        local p = along_arc_from_origin((s - s0_scaled) / (size_scale * s_scale), k_arc)
        return scale_spot(p)
    end

    local function set_transform(arc_start_arg, size_scale_arg, s_scale_arg, s_start_arg)
        p0 = arc_start_arg.copy()
        s0_scaled = s_start_arg
        size_scale = size_scale_arg
        s_scale = s_scale_arg
    end

    local function copy()
        local ta = TrackArc(s_arc, k_arc)
        ta.set_transform(p0, size_scale, s_scale, s0_scaled)
        return ta
    end

    self[1] = s_arc
    self[2] = k_arc
    self.s_arc = function() return s_arc end
    self.k_arc = function() return k_arc end
    self.arc_start_s_scaled = function() return s0_scaled end
    self.arc_end_s_scaled = function() return s_arc * size_scale * s_scale + s0_scaled end
    self.arc_end_spot = function() return scale_spot(along_arc_from_origin(s_arc, k_arc)) end
    self.along_arc = along_arc
    self.set_transform = set_transform
    self.copy = copy

    return self
end

local function dump_table(o)
    if type(o) == 'table' then
        local s = '{ '
        for k,v in pairs(o) do
            if type(k) ~= 'number' then k = '"'..k..'"' end
            s = s .. '['..k..'] = ' .. dump_table(v) .. ','
        end
        return s .. '} '
    else
        return tostring(o)
    end
end

local function Track(...)
    local self = {}

    for i, arcs in ipairs({...}) do
        for i1, arc in ipairs(arcs) do
            table.insert(self, TrackArc(arc[1], arc[2]))
        end
    end

    if #self < 1 then
        gcs_send("Error: Track with no arcs")
    end

    local function link_arcs(track_start_arg, size_scale_arg, s_scale_arg, s_start_arg)
        local p = track_start_arg
        local s = s_start_arg
        for i, arc in ipairs(self) do
            arc.set_transform(p, size_scale_arg, s_scale_arg, s)
            p = arc.arc_end_spot()
            s = arc.arc_end_s_scaled()
        end
    end

    local function set_transform(track_start_arg, size_scale_arg, s_scale_arg, s_start_arg)
        link_arcs(track_start_arg, size_scale_arg, s_scale_arg, s_start_arg)
    end

    local arc_idx_cached = 1

    local function update_arc_idx_caches(s_)
        local arc_idx = arc_idx_cached
        if s_ < self[arc_idx].arc_start_s_scaled() then
            arc_idx = 1
        end
        while s_ >= self[arc_idx].arc_end_s_scaled() do
            if arc_idx >= #self then
                break;
            end
            arc_idx = arc_idx + 1
        end
        arc_idx_cached = arc_idx
    end

    local function along_track(s_)
        update_arc_idx_caches(s_)
        return self[arc_idx_cached].along_arc(s_)
    end

    local function along_track_ext(s_, sx_)
        local p = along_track(s_)
        local px = self[arc_idx_cached].along_arc(s_ + sx_)
        -- gcs_send(string.format("%f, %f", s_, sx_))
        return p, px
    end

    local function is_complete(s_)
        update_arc_idx_caches(s_)
        return arc_idx_cached == #self and
            s_ >= self[arc_idx_cached].arc_end_s_scaled()
    end

    local function dump()
        for i, arc in ipairs(self) do
            gcs_send(string.format("i: %i, %.2f, %.2f, %.2f", i, arc.arc_end_spot().n(), arc.arc_end_spot().e(), arc.arc_end_spot().theta()))
        end
    end

    link_arcs(TrackSpot(0., 0., 0.), 1, 1, 0)

    self.set_transform = set_transform
    self.along_track = along_track
    self.along_track_ext = along_track_ext
    self.is_complete = is_complete
    self.dump = dump

    return self
end


local function test_TA()
    local success = 1
    local eps = 1.0e-5
    local pi = math.pi
    local pi2 = 2 * pi
    local test_idx = 0

    -- Note: LUA has restricted floating precision. So use larger epsilon than normal
    local function compare(str_, e_, a_)
        test_idx = test_idx + 1
        if math.abs(e_.e() - a_.e()) > eps or
            math.abs(e_.n() - a_.n()) > eps or
            math.abs(wrap_angle.rad_2pi(e_.theta()) - wrap_angle.rad_2pi(a_.theta())) > eps then
            success = 0
            gcs_send(string.format( "%i %s, expected(%.4f, %.4f, %.4f), actual(%.4f, %.4f, %.4f)",
                test_idx, str_, e_.n(), e_.e(), e_.theta(), a_.n(), a_.e(), a_.theta()))
        end
    end

    local function one_test(s_arg, k_arg, expected)
        local ta = TrackArc(s_arg, k_arg)
        local actual = ta.along_arc(s_arg)
        local str = string.format("Test(s:%.2f, k:%.2f) ", s_arg, k_arg)
        compare(str, expected, actual)
    end

    local function x_test(ta_arg, arc_start_arg, size_scale_arg, s_scale_arg, s_start_arg, expected)
        ta_arg.set_transform(arc_start_arg, size_scale_arg, s_scale_arg, s_start_arg)
        local actual = ta_arg.along_arc(ta_arg.s_arc() * size_scale_arg * s_scale_arg + s_start_arg)
        local str = string.format("Test(s:%.2f, k:%.2f) ", ta_arg.s_arc(), ta_arg.k_arc())
        compare(str, expected, actual)
    end

    local ta0 = TrackArc(pi, 1.0)
    local ta1 = TrackArc(pi, -1.0)

    one_test(0, 1, TrackSpot(0, 0, 0))
    one_test(0, -1, TrackSpot(0, 0, 0))
    one_test(1, 0, TrackSpot(1, 0, 0))
    one_test(-1, 0, TrackSpot(-1, 0, 0))
    one_test(pi, 1, TrackSpot(0, 2, -pi))
    one_test(pi, -1.0, TrackSpot(0, -2, -pi))
    one_test(pi, .5, TrackSpot(2, 2, pi / 2))
    one_test(pi, -.5, TrackSpot(2, -2, -pi / 2))
    one_test(pi / 2, 1, TrackSpot(1, 1, pi / 2))
    one_test(pi / 2, -1, TrackSpot(1, -1, -pi / 2))

    x_test(ta0, TrackSpot(3, 4, 0), 1, 1, 0, TrackSpot(3, 6, pi))
    x_test(ta0, TrackSpot(3, 4, pi / 2), 1, 1, 0, TrackSpot(1, 4, -pi / 2))
    x_test(ta0, TrackSpot(3, 4, -pi / 2), 1, 1, 0, TrackSpot(5, 4, pi / 2))
    x_test(ta1, TrackSpot(3, 4, 0), 1, 1, 0, TrackSpot(3, 2, pi))
    x_test(ta1, TrackSpot(3, 4, pi / 2), 1, 1, 0, TrackSpot(5, 4, -pi / 2))
    x_test(ta1, TrackSpot(3, 4, -pi / 2), 1, 1, 0, TrackSpot(1, 4, pi / 2))

    x_test(ta0, TrackSpot(3, 4, 0), 2., 1, 0, TrackSpot(3, 8, -pi))
    x_test(ta0, TrackSpot(3, 4, pi/2), 2., 1, 0, TrackSpot(-1, 4, -pi/2))
    x_test(ta0, TrackSpot(3, 4, -pi/2), 2., 1, 0, TrackSpot(7, 4, pi/2))

    x_test(ta0, TrackSpot(3, 4, 0), 2., 6, 0, TrackSpot(3, 8, -pi))
    x_test(ta0, TrackSpot(3, 4, pi/2), 2., 6, 0, TrackSpot(-1, 4, -pi/2))
    x_test(ta0, TrackSpot(3, 4, -pi/2), 2., 6, 0, TrackSpot(7, 4, pi/2))

    x_test(ta0, TrackSpot(3, 4, 0), 2., 6, 5, TrackSpot(3, 8, -pi))
    x_test(ta0, TrackSpot(3, 4, pi/2), 2., 6, 5, TrackSpot(-1, 4, -pi/2))
    x_test(ta0, TrackSpot(3, 4, -pi/2), 2., 6, 5, TrackSpot(7, 4, pi/2))

    return success
end
local function test_T()
    local pi = math.pi

    local t = Track({
        { pi, 1 },
        { pi, -1 },
        TrackArc(pi, 1),
    })

    t.dump()
    return 0
end

local function Guider()

    local first_loc = ahrs:get_location()
    local first_vel_vmps = ahrs:get_velocity_NED()
    if not first_loc or not first_vel_vmps then
        gcs_send("Error: cannot get start location.")
        return nil
    end
    first_loc:change_alt_frame(0)
    local first_vel_angle_r = math.atan(first_vel_vmps:y(), first_vel_vmps:x())


    -- All the failure modes have passed so we can enable guiding. A question is
    -- should we have set a target wp before enabling guiding. For now no but this needs checking
    local saved_mode = vehicle:get_mode()
    vehicle:set_mode(PLANE_MODE_GUIDED)

    local function finish()
        vehicle:set_mode(saved_mode)
    end

    local time_start = millis():tofloat() * 0.001

    local track = Track({
        {2*math.pi, 1},
        {2*math.pi, -1}
    })

    local s_scale = 0.3/(2*math.pi)
    track.set_transform(TrackSpot(0, 0, 0), 200, s_scale, time_start)

    return function(abort)

        if abort then
            finish()
            return false
        end

        local loc_c = ahrs:get_location()
        local vel_cur_vmps = ahrs:get_velocity_NED()
        local wp_next = vehicle:get_target_location()
        if not loc_c or not vel_cur_vmps or not wp_next then
            gcs_send("Aborting due to nil object")
            return false
        end
        local speed_cur_mps = vel_cur_vmps:length()
        loc_c:change_alt_frame(0)

        -- test for getting close to ground
        local loc_g = loc_c:copy()
        if loc_g:change_alt_frame(3) and loc_g:alt() < 1000 then
            gcs_send("Aborting guiding because too close to ground")
            return false
        end

        local time_cur = millis():tofloat() * 0.001
        local p, px = track.along_track_ext(time_cur, 50. * s_scale)

        -- Figure out where the next target should be.
        local loc_target = first_loc:copy()
        loc_target:offset(px.n(), px.e())
        loc_target:alt(first_loc:alt())
        loc_target:change_alt_frame(0)

        local loc_track = first_loc:copy()
        loc_track:offset(p.n(), p.e())
        loc_track:alt(first_loc:alt())
        loc_track:change_alt_frame(0)

        vehicle:update_target_location(wp_next, loc_target)


        local t_del = time_cur - time_start
        local dist_target = loc_c:get_distance(loc_target)
        local dist_track = loc_c:get_distance(loc_track)
        gcs_send(string.format("p %.2f (%.0f, %.0f, %.0f), (%.0f, %.0f, %.0f)", t_del, 
            p.n(), p.e(), dist_track, px.n(), px.e(), dist_target))
        -- gcs_send(string.format("Do it %i", time_cur - time_last))


        return not track.is_complete(time_cur)
    end
end

return (function()
    local r, d = switch_exec_updatefactory("Guider L1 Trajectory", Guider, GUIDING_TIME_MS, 300, gcs_send)

    gcs_send(string.format("Loaded guide_L1_trajectory.lua %i, %i", 1, test_T()))

    return r, d
end)()