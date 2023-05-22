import time
from espresso import *

PRE_INF_DUR = 8

import time
from espresso import *

PRE_INF_DUR = 8

def pre_infuse():
    set_pump_level(1/6)

def brew():
    set_pump_level(3/6)

def temp_control(
    target,
    curr_temp,
    last_time=None,
    last_error=None,
    last_integral=0.,
):
    alpha = 1/10
    beta = 1/2000
    gamma = 1/20

    curr_time = time.ticks_ms()
    error = target - curr_temp

    proportional = error
    derivative = 0.
    integral = last_integral
    
    if not (last_time is None):
        sec_diff = time.ticks_diff(curr_time, last_time) / 1000.
        if sec_diff > 0:
            derivative = (error - last_error) / sec_diff
        integral += last_error * sec_diff

    control = alpha*proportional + beta*integral + gamma*derivative
    
    set_heat_level(control)
    return curr_time, error, integral

def test():
    boot_screen()

    temp_targ = 92.0
    pres_targ = 9.0
    sec_targ = 25.0
    start = time.ticks_ms()
    seconds = 0.0
    timing = False
    
    last_time = None
    last_error = None
    last_integral = 0.0

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
        if timing and seconds <= PRE_INF_DUR: set_temp += 1
        last_time, last_error, last_integral = temp_control(
            set_temp, cur_temp, last_time, last_error, last_integral
        )

        update_display(cur_temp, cur_pres, seconds, temp_targ, pres_targ, sec_targ)
        time.sleep(0.1)


test()
