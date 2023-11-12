

local function deg_360(angle)
    local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

local function deg_180(angle)
    local res = deg_360(angle)
    if res > 180 then
        res = res - 360
    end
    return res
end


local function rad_2pi(angle)
    local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

local function rad_pi(angle)
    local res = deg_360(angle)
    if res > 180 then
        res = res - 360
    end
    return res
end


return {
    deg_360 = deg_360,
    deg_180 = deg_180,
    rad_2pi = rad_2pi,
    rad_pi = rad_pi,
 }