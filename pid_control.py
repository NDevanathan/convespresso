import time
from espresso import *


def temp_control(
    target,
    last_time=None,
    last_error=None,
    last_integral=0.
):
    alpha = 1/20
    beta = 1/20
    gamma = 1/20

    curr_time, curr_temp = time.time_ns(), poll_temp()
    error = target - curr_temp

    proportional = error
    derivative = 0.
    integral = last_integral
    if not (last_time is None):
        derivative = (error - last_error) / ((curr_time - last_time) / 1e+9)
        integral += last_error * (curr_time - last_time) / 1e+9

    control = alpha*proportional + beta*derivative + gamma*integral
    if control <= 0:
        heat_off()
    elif control <= 2/6:
        control = 2/6

    set_heat_level(control)

    return curr_time, error, integral


def pressure_control(target):
    set_pump_level(4/6)

def test():
    display.draw_bitmap("res/cvx.mono", WIDTH // 2 - 50, HEIGHT // 2 - 25, 100, 50, rotate=180)
    display.present()
    time.sleep(3)
    display.clear()

    temp_targ = 92.0
    pres_targ = 9.0
    time_targ = 30.0
    start = time.time_ns()
    seconds = 0.0
    timing = False
    temping = False

    while True:
        display.clear_buffers()
        display.draw_text(START_X,START_Y,"TEMP.",FONT,rotate=180)
        display.draw_text(START_X-39,START_Y,"PRES.",FONT,rotate=180)
        display.draw_text(START_X-78,START_Y,"TIMER",FONT,rotate=180)

        if not SWT_BREW.value():
            open_valve()
            pressure_control(pres_targ)

            if not timing:
                timing = True
                start = time.time_ns()
        else:
            pump_off()
            close_valve()
            timing = False

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

        if timing:
            seconds = (time.time_ns() - start) / 1.0e+9

        temp_str = "{0:.1f}".format(poll_temp())
        pres_str = "{0:.1f}".format(poll_pressure())
        sec_str = "{0:.1f}".format(seconds)

        display.draw_text(START_X-6*(5-len(temp_str)),START_Y-10,temp_str,FONT,rotate=180)
        display.draw_text(START_X-39-6*(5-len(pres_str)),START_Y-10,pres_str,FONT,rotate=180)
        display.draw_text(START_X-78-6*(5-len(sec_str)),START_Y-10,sec_str,FONT,rotate=180)

        display.draw_text(START_X-6,START_Y-20,"{0:.1f}".format(temp_targ),FONT,rotate=180)
        display.draw_text(START_X-39-12,START_Y-20,"{0:.1f}".format(pres_targ),FONT,rotate=180)
        display.draw_text(START_X-78-6,START_Y-20,"{0:.1f}".format(time_targ),FONT,rotate=180)

        display.present()
        time.sleep(0.1)


test()
