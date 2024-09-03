local gcs_send = require("gcs_send_funcfactory")("TTR")
local wrap_angle = require("wrap_angle_obj")
local track = require("track_obj")(gcs_send, wrap_angle)

local TrackSpot = track.TrackSpot
local TrackArc = track.TrackArc

local function test_TA()
    local success = 1
    local eps = 1.0e-5
    local pi = math.pi
    local pi2 = 2 * pi
    local test_idx = 0

    -- Note: LUA has restricted floating precision. So use larger epsilon than normal
    local function compare(str, expected, actual)
        test_idx = test_idx + 1
        if math.abs(expected:e() - actual:e()) > eps or
            math.abs(expected:n() - actual:n()) > eps or
            math.abs(wrap_angle.rad_2pi(expected:theta()) - wrap_angle.rad_2pi(actual:theta())) > eps then
            success = 0
            gcs_send(string.format("%i %s, expected(%.4f, %.4f, %.4f), actual(%.4f, %.4f, %.4f)",
                test_idx, str, expected:n(), expected:e(), expected:theta(), actual:n(), actual:e(), actual:theta()))
        end
    end

    local function one_test(s, k, expected)
        local ta = track.TrackArc(s, k)
        local actual = ta:along_arc(s)
        local str = string.format("Test(s:%.2f, k:%.2f) ", s, k)
        compare(str, expected, actual)
    end

    local function x_test(ta, arc_start, size_scale, s_scale, s_start, expected)
        ta:set_transform(arc_start, size_scale, s_scale, s_start)
        local actual = ta:along_arc(ta:s() * size_scale * s_scale + s_start)
        local str = string.format("Test(s:%.2f, k:%.2f) ", ta:s(), ta:k())
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
    local pi = math.pi

    local t = track.Track({
        { pi /2, 1 },
        { pi, -1 },
        track.TrackArc(pi, 1),
    })
    t:dump()
    return 0
end

gcs_send(string.format("Loaded track_test.lua %i, %i", test_TA(), 1))

local function do_nothing()
    return do_nothing, 1000
end
return do_nothing()

