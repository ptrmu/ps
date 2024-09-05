local gcs_send = require("gcs_send_funcfactory")("TTR")
local wrap_angle = require("wrap_angle_obj")
local track = require("track_obj")(gcs_send, wrap_angle)

local TrackSpot = track.TrackSpot
local TrackArc = track.TrackArc
local Track = track.Track

local eps = 1.0e-5
local pi = math.pi
local pi2 = pi * 2
local pid2 = pi / 2
local pid4 = pi / 4
local sqrt2 = math.sqrt(2)

-- Note: LUA has restricted floating precision. So use larger epsilon than normal
local function compare(str, expected, actual)
    if math.abs(expected:e() - actual:e()) > eps or
        math.abs(expected:n() - actual:n()) > eps or
        math.abs(wrap_angle.rad_2pi(expected:theta()) - wrap_angle.rad_2pi(actual:theta())) > eps then
        gcs_send(string.format("%s, expected(%.4f, %.4f, %.4f), actual(%.4f, %.4f, %.4f)",
            str, expected:n(), expected:e(), expected:theta(), actual:n(), actual:e(), actual:theta()))
        return false
    end
    return true
end

local function test_TA()
    local success = 1
    local test_idx = 0

    local function one_test(s, k, expected)
        test_idx = test_idx + 1
        local ta = track.TrackArc(s, k)
        local actual = ta:along_arc(s)
        local str = string.format("%i Test(s:%.2f, k:%.2f) ", test_idx, s, k)
        if not compare(str, expected, actual) then
            success = 0
        end
    end

    local function x_test(ta, arc_start, size_scale, s_scale, s_start, expected)
        test_idx = test_idx + 1
        ta:set_transform(arc_start, size_scale, s_scale, s_start)
        local actual = ta:along_arc(ta:s() * size_scale / s_scale + s_start)
        local str = string.format("%i Test(s:%.2f, k:%.2f) ", test_idx, ta:s(), ta:k())
        if not compare(str, expected, actual) then
            success = 0
        end
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
    x_test(ta0, TrackSpot(3, 4, pi / 2), 2., 1, 0, TrackSpot(-1, 4, -pi / 2))
    x_test(ta0, TrackSpot(3, 4, -pi / 2), 2., 1, 0, TrackSpot(7, 4, pi / 2))

    x_test(ta0, TrackSpot(3, 4, 0), 2., 6, 0, TrackSpot(3, 8, -pi))
    x_test(ta0, TrackSpot(3, 4, pi / 2), 2., 6, 0, TrackSpot(-1, 4, -pi / 2))
    x_test(ta0, TrackSpot(3, 4, -pi / 2), 2., 6, 0, TrackSpot(7, 4, pi / 2))

    x_test(ta0, TrackSpot(3, 4, 0), 2., 6, 5, TrackSpot(3, 8, -pi))
    x_test(ta0, TrackSpot(3, 4, pi / 2), 2., 6, 5, TrackSpot(-1, 4, -pi / 2))
    x_test(ta0, TrackSpot(3, 4, -pi / 2), 2., 6, 5, TrackSpot(7, 4, pi / 2))

    return success
end

local function test_T()
    local success = 1
    local test_idx = 0

    local trk = Track(
        {{ pid4, 1 },
        { pid4, -1 },},
        {TrackArc(pi * 2, -0.5)},
        Track{TrackArc(sqrt2, 0)}
    )
    -- trk:dump()
    local end_n = 0
    local end_e = 2 * (1 - 1/sqrt2) - 4
    local end_theta = pi
    local end_s = pid4 + pid4 + pi * 2 + sqrt2

    local function trk_test(arc_start, size_scale, s_scale, s_start, s, expected)
        test_idx = test_idx + 1
        trk:set_transform(arc_start, size_scale, s_scale, s_start)
        local actual = trk:along_track(s)
        local str = string.format("trk_test(i:%i)", test_idx)
        if not compare(str, expected, actual) then
            success = 0
        end
    end

    trk_test(TrackSpot(0, 0, 0), 1, 1, 0, end_s, TrackSpot(end_n, end_e, end_theta))
    trk_test(TrackSpot(1, 0, 0), 1, 1, 0, end_s, TrackSpot(end_n + 1, end_e, end_theta))
    trk_test(TrackSpot(0, 1, 0), 1, 1, 0, end_s, TrackSpot(end_n, end_e + 1, end_theta))
    trk_test(TrackSpot(0, 0, pid2), 1, 1, 0, end_s, TrackSpot(-end_e, end_n, wrap_angle.rad_pi(end_theta + pid2)))
    trk_test(TrackSpot(0, 0, 0), 2, 1, 0, end_s * 2, TrackSpot(end_n * 2, end_e * 2, end_theta))
    trk_test(TrackSpot(0, 0, 0), 1, 25, 0, end_s / 25, TrackSpot(end_n, end_e, end_theta))
    trk_test(TrackSpot(0, 0, 0), 2, 25, 0, end_s / 25 * 2, TrackSpot(end_n * 2, end_e * 2, end_theta))
    trk_test(TrackSpot(0, 0, 0), 1, 1, 1, end_s + 1, TrackSpot(end_n, end_e, end_theta))

    return success
end

gcs_send(string.format("Loaded track_test.lua %i, %i", 1, 1))

local function do_nothing()
    return do_nothing, 1000
end
local function do_test_T()
    gcs_send(string.format("test_T() %i", test_T()))
    return do_nothing, 1000
end
local function do_test_TA()
    gcs_send(string.format("test_TA() %i", test_TA()))
    return do_test_T, 1000
end
return do_test_TA, 1000


