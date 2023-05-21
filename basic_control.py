import time
from espresso import *

PRE_INF_DUR = 8

def temp_control(target, current, elapsed):
    if elapsed >= PRE_INF_DUR:
        target += 2

    alpha = 1/20
    if current >= target:
        heat_off()
    else:
        control = alpha * (target - current)
        if control <= 2/6:
            control = 2/6

        set_heat_level(control)

def pressure_control(target, current, elapsed):
    if elapsed < PRE_INF_DUR:
        set_pump_level(1/6)
    else:
        set_pump_level(3/6)

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

        if not SWT_MODE.value():
            temp_targ = 125
        else:
            temp_targ = 92

        if not SWT_BREW.value():
            if not timing:
                timing = True
                start = time.ticks_ms()
        else:
            timing = False

        if timing:
            seconds = time.ticks_diff(time.ticks_ms(), start) / 1000
            temp_control(temp_targ, cur_temp, seconds)
        else:
            temp_control(temp_targ, cur_temp, 0)

        if not SWT_BREW.value():
            open_valve()
            pressure_control(pres_targ, cur_pres, seconds)
        else:
            pump_off()
            close_valve()

        update_display(cur_temp, cur_pres, seconds, temp_targ, pres_targ, sec_targ)
        time.sleep(0.1)


test()
