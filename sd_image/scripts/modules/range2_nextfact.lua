
local range_nextfact = require("range_nextfact")

local function range2_nextfact(start_o, stop_o, step_o, start_i, stop_i, step_i)
    local outer_next = range_nextfact(start_o, stop_o, step_o)
    local inner_next = range_nextfact(start_i, stop_i, step_i)
    local outer_i = nil
    local inner_i = nil

    local function next_func(_s)
        repeat
            if inner_i == nil then
                outer_i = outer_next()
                if outer_i == nil then
                    return nil -- end of loop
                end
            end
            inner_i = inner_next()
        until inner_i ~= nil

        return outer_i, inner_i
    end

    return next_func
end

return range2_nextfact