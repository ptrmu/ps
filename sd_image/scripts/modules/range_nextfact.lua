

local function range_nextfact(start, stop, step)
    if stop == nil and step == nil then
        stop = start
        start = 1
        step = 1
    elseif step == nil or step == 0 then
        step = 1
    end

    local i = nil

    local function next_func()
        if i == nil then
            i = start
        else
            i = i + step
        end
        if i > stop then
            i = nil
        end
        return i
    end

    return next_func
end

return range_nextfact