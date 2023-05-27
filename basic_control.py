import time
from espresso import *

PRE_INF_DUR = 8
SHOT_DUR = 18
PRE_INF_LEVEL = 1/6
TARGET_FLOW_RATE = 2.0
BREW_TEMP = 92
STEAM_TEMP = 125

def pre_infuse():
    set_pump_level(PRE_INF_LEVEL)

def brew(flow, target):
    level = target / flow
    set_pump_level(level)
    return level

def temp_control(target, current):
    alpha = 1/20
    control = alpha * (target - current)
    if 0 <= control and control <= 2/6: control = 2/6
        
    set_heat_level(control)

def test():
    boot_screen()

    temp_targ = BREW_TEMP
    total_flow = 0.0
    start = time.ticks_ms()
    seconds = 0.0
    pump_level = PRE_INF_LEVEL

    while True:
        cur_temp = poll_temp()
        cur_pres = poll_pressure()

        temp_targ = STEAM_TEMP if not SWT_MODE.value() else BREW_TEMP
        if (not SWT_BREW.value()) and SWT_MODE.value():
            if not timing:
                timing = True
                start = time.ticks_ms()
                seconds = 0.0
                
            if timing and seconds <= PRE_INF_DUR + SHOT_DUR:
                seconds = time.ticks_diff(time.ticks_ms(), start) / 1000
                open_valve()
                
                if seconds <= PRE_INF_DUR:
                    pump_level = PRE_INF_LEVEL
                    pre_infuse()
                else:
                    flow = calc_flow(cur_pres, pump_level)
                    pump_level = brew(flow, flow_targ, pump_level)
                    total_flow += 0.1 * flow
                    
        else:
            timing = False
            pump_off()
            close_valve()

        set_temp = temp_targ
        if timing and seconds <= PRE_INF_DUR + SHOT_DUR: set_temp += 2
        temp_control(set_temp, cur_temp)

        #update_display(cur_temp, cur_pres, seconds, temp_targ, pres_targ, PRE_INF_DUR + SHOT_DUR)
        time.sleep(0.1)


test()
