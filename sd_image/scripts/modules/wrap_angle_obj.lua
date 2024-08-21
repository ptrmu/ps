
local pi = math.pi
local pi2 = 2. * math.pi

local function deg_360(angle_deg)
    local res_deg = math.fmod(angle_deg, 360.0)
    if res_deg < 0 then
        res_deg = res_deg + 360.0
    end
    return res_deg
end

local function deg_180(angle_deg)
    local res_deg = deg_360(angle_deg)
    if res_deg > 180 then
        res_deg = res_deg - 360
    end
    return res_deg
end


local function rad_2pi(angle_rad)
    local res_rad = math.fmod(angle_rad, pi2)
    if res_rad < 0 then
        res_rad = res_rad + pi2
    end
    return res_rad
end

local function rad_pi(angle_rad)
    local res_rad = rad_2pi(angle_rad)
    if res_rad > pi then
        res_rad = res_rad - pi2
    end
    return res_rad
end


return {
    deg_360 = deg_360,
    deg_180 = deg_180,
    rad_2pi = rad_2pi,
    rad_pi = rad_pi,
 }