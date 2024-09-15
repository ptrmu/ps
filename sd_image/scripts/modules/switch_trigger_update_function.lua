

local function define_classes(name, gcs_send)

    local function SwitchTriggerUpdateFunction(func_factory, update_period_ms, switch_code)

        local go_switch = rc:find_channel_for_option(switch_code)
        if not go_switch then
            gcs_send(string.format("Error: cannot find channel for option %i", switch_code))
            return -- bad configuration, don't continue
        end

        -- forward declarations of local functions
        local state_not_ready

        local func


        local function test_go(switch_true)
            if not arming:is_armed() then
                return false
            end
            return (go_switch:get_aux_switch_pos() > 0) == switch_true
        end

        local function goto_not_ready()
            return state_not_ready, 0
        end

        local function goto_complete()
            gcs_send(string.format("Finish %s", name))
            if func then
                func(true)
                func = nil
            end
            return goto_not_ready()
        end

        local function state_exec()
            if not test_go(true) then
                return goto_complete()
            end
            if not func or not func(false) then
                return goto_complete()
            end
            return state_exec, update_period_ms
        end

        local function goto_exec()
            func = func_factory()
            if not func then
                gcs_send(string.format("Start %s failed: no exec_func object", name))
                return goto_not_ready()
            end
            gcs_send(string.format("Start %s", name))
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

        gcs_send(string.format("Loaded %s.lua", name))
        return goto_not_ready()
    end

    local function UpdateNothing()
        gcs_send(string.format("Terminating script %s.lua. Script will no longer run.", name))
        local function update_nothing()
            return update_nothing, 1000
        end
        return update_nothing, 1000
    end

    return {
        SwitchTriggerUpdateFunction = SwitchTriggerUpdateFunction,
        UpdateNothing = UpdateNothing,
    }
end

return define_classes