

local gcs_send = require("gcs_send_funcfactory")("GAT")
local wrap_angle = require("wrap_angle_obj")
local switch_exec_updatefactory = require("switch_exec_updatefactory")


local GUIDING_TIME_MS = 500

local PLANE_MODE_GUIDED        = 15

local function State2d(n, e, theta)
    return {
        n = n,
        e = e,
        theta = theta
    }
end

-- Using NE coordinates, 0 angle along N axis, clockwise angles are positive
local function TrajectoryArc(arc_s, arc_k)
    local self = {}

    local p0 = State2d(0., 0., 0.)

    -- s - distance along arc
    -- k - curvature = 1/r
    local function along_arc_from_origin(s, k)
        -- N = sin(sk)/k 
        -- E = (1-cos(sk))/k 
        -- theta = sk
        local n = s
        local e = 0
        local sk = s * k

        -- if k == 0 then straight line
        if k ~= 0.0 then
    
            -- if k is small, use the series expansion to avoid the divide by zero
            if math.abs(k) < 1.0e-6 then
                local s2k2 = sk * sk
                n = s * (s2k2 * (s2k2 * (-s2k2/5040.0 + 1.0/120.0) - 1.0/6.0) + 1.0)
                e = s * sk * (s2k2 * (s2k2 * (-s2k2/40320.0 + 1.0/720.0) - 1.0/24.0) + 1.0/2.0)

            -- Use normal formulas
            else
                n = math.sin(sk) / k
                e = (1. - math.cos(sk)) / k
            end
        end

        return State2d(n, e, wrap_angle.rad_pi(sk))
    end

    self.along_arc = function(s)
        local p = along_arc_from_origin(s, arc_k)
        local n_temp = p0.n + p.n * math.cos(p0.theta) - p.e * math.sin(p0.theta)
        local e_temp = p0.e + p.n * math.sin(p0.theta) + p.e * math.cos(p0.theta)
        p.n = n_temp
        p.e = e_temp
        p.theta = wrap_angle.rad_pi(p0.theta + p.theta)
        return p
    end

    return self
end

local function test_TA()
    local ta = TrajectoryArc(math.pi, -1)
    local n = 9
    for i = 0, n do
        local s = i * math.pi / n
        local p = ta.along_arc(s)
        print(string.format("%6.2f, %6.2f, %6.2f", p.n, p.e, math.deg(p.theta)))
    end
end

test_TA()

local function Trajectory()
end

local function Guider()

    local time_last = millis():toint()

    return function(abort)

        if abort then
             return false
        end

        local time_cur = millis():tofloat()

        gcs_send(string.format("Do it %i", 0))
        -- gcs_send(string.format("Do it %i", time_cur - time_last))

        time_last = time_cur

        return true
    end
end

return (function()
    local r, d = switch_exec_updatefactory("Guider L1 Trajectory", Guider, GUIDING_TIME_MS, 300, gcs_send)

    gcs_send("Loaded guide_L1_trajectory.lua")

    return r, d
end)()