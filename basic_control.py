import time
from espresso import *

PRE_INF_DUR = 8

def pre_infuse():
    set_pump_level(1/6)

def brew():
    set_pump_level(3/6)

def temp_control(target, current):
    alpha = 1/20
    control = alpha * (target - current)
    if 0 <= control and control <= 2/6: control = 2/6
        
    set_heat_level(control)

def test():
    boot_screen()

    temp_targ = 92.0
    pres_targ = 9.0
    sec_targ = 25.0
    start = time.ticks_ms()
    seconds = 0.0
    timing = False

    while True:
        cur_temp = poll_temp()
        cur_pres = poll_pressure()
        
        temp_targ = 125 if not SWT_MODE.value() else 92

        if (not SWT_BREW.value()) and SWT_MODE.value():
            if not timing:
                timing = True
                start = time.ticks_ms()
                
            if timing:
                seconds = time.ticks_diff(time.ticks_ms(), start) / 1000
                open_valve()
                
                if seconds <= PRE_INF_DUR:
                    pre_infuse()
                else:
                    brew()

        else:
            timing = False
            pump_off()
            close_valve()

        set_temp = temp_targ
        if timing and seconds <= PRE_INF_DUR: set_temp += 2
        temp_control(set_temp, cur_temp)

        update_display(cur_temp, cur_pres, seconds, temp_targ, pres_targ, sec_targ)
        time.sleep(0.1)


test()
