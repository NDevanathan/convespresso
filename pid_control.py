import time
from espresso import *


PRE_INF_DUR = 8

def temp_control(
    target,
    last_time=None,
    last_error=None,
    last_integral=0.,
    elapsed=0.
):
    alpha = 1/20
    beta = 1/20
    gamma = 1/20

    if elapsed >= PRE_INF_DUR:
        target += 2

    curr_time, curr_temp = time.ticks_ms(), poll_temp()
    error = target - curr_temp

    proportional = error
    derivative = 0.
    integral = last_integral
    if not (last_time is None):
        sec_diff = time.ticks_diff(curr_time, last_time) / 1000.
        if sec_diff > 0:
            derivative = (error - last_error) / sec_diff
        integral += last_error * sec_diff

    control = alpha*proportional + beta*derivative + gamma*integral
    if control <= 0:
        heat_off()
    elif control <= 2/6:
        control = 2/6

    set_heat_level(control)

    return curr_time, error, integral


def pressure_control(target, current, elapsed):
    if elapsed < PRE_INF_DUR:
        set_pump_level(1/6)
    else:
        set_pump_level(4/6)


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
        else:
            seconds = 0.

        last_time, last_error, last_integral = temp_control(
            temp_targ, last_time, last_error, last_integral, seconds
        )

        if not SWT_MODE.value():
            if not temping:
                temping = True
                last_time, last_error, last_integral = temp_control(
                    temp_targ
                )
            else:
                last_time, last_error, last_integral = temp_control(
                    temp_targ, last_time, last_error, last_integral
                )
        else:
            temping = False
            heat_off()

        if not SWT_BREW.value():
            open_valve()
            pressure_control(pres_targ, cur_pres, seconds)
        else:
            pump_off()
            close_valve()

        update_display(cur_temp, cur_pres, seconds, temp_targ, pres_targ, sec_targ)
        time.sleep(0.1)


test()
