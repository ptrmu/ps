

local function switch_exec_updatefact(name, exec_funcfact, update_period_ms, switch_code, send_text_func)

    local go_switch = rc:find_channel_for_option(switch_code)
    if not go_switch then
        send_text_func(string.format("Error: cannot find channel for option %i", switch_code))
        return -- bad configuration, don't continue
    end

    -- forward declarations of local functions
    local state_not_ready

    local exec_func


    local function test_go(switch_true)
        if not arming:is_armed() then
            return false
        end
        return (go_switch:get_aux_switch_pos() == 2) == switch_true
    end

    local function goto_not_ready()
        return state_not_ready, 0
    end

    local function goto_complete()
        send_text_func(string.format("Finish %s", name))
        if exec_func then
            exec_func(true)
            exec_func = nil
        end
        return goto_not_ready()
    end

    local function state_exec()
        if not test_go(true) then
            return goto_complete()
        end
        if not exec_func or not exec_func(false) then
            return goto_complete()
        end
        return state_exec, update_period_ms
    end

    local function goto_exec()
        exec_func = exec_funcfact()
        if not exec_func then
            send_text_func(string.format("Start %s failed: no exec_func object", name))
            return goto_not_ready()
        end
        send_text_func(string.format("Start %s", name))
        return state_exec, 0
    end

    local function state_ready()
        -- Wait until the trigger switch is on before starting guiding.
        if test_go(true) then
            return goto_exec()
        end
        return state_ready, update_period_ms
    end

    state_not_ready = function()
        -- Ensure the trigger switch is off. This prevents guiding if reboot with switch on.
        if test_go(false) then
            return state_ready, 0
        end
        return state_not_ready, update_period_ms
    end

    return goto_not_ready()
end

return switch_exec_updatefact