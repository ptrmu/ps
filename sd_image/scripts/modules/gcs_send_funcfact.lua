

local gcs_send_funcfact = function(name, eat_messages_period_s, msg_severity)
    local gcs_send_times = {}
    local gcs_eaten_count = {}

    ---@param str1 string
    ---@param str2 string|nil
    return function(str1, str2)
        if not str1 or #str1 == 0 then return end
        if not msg_severity then msg_severity = 6 end
        if not eat_messages_period_s then eat_messages_period_s = 1.0 end
        
        local send_str = nil

        if not str2 or #str2 == 0 then
            send_str = string.format("%s: %s", name, str1)
        
        else
            local time_curr_s = millis():tofloat() / 1000.0
            local time_first_s = gcs_send_times[str1]
            if (time_first_s) then
                local dur_since_first = time_curr_s - time_first_s
                if dur_since_first < eat_messages_period_s then
                    if not gcs_eaten_count[str1] then
                        gcs_eaten_count[str1] = 0
                    end
                    gcs_eaten_count[str1] = gcs_eaten_count[str1] + 1
                    return
                end
            end

            local eaten_count = gcs_eaten_count[str1]
            if eaten_count then
                gcs_eaten_count[str1] = nil
                if str2 then
                    send_str = string.format("%s: %s %s (+%i)", name, str1, str2, eaten_count)
                else
                    send_str = string.format("%s: %s (+%i)", name, str1, eaten_count)
                end
            else
                if str2 then
                    send_str = string.format("%s: %s %s", name, str1, str2)
                else
                    send_str = string.format("%s: %s", name, str1)
                end
            end

            gcs_send_times[str1] = time_curr_s
        end

        gcs:send_text(msg_severity, send_str)
    end
end

return gcs_send_funcfact