import time
from espresso import *

PRE_INF_DUR = 8
RAMP_DUR = 5
PRE_INF_LEVEL = 1/6
RAMP_LEVEL = 4/6
TARGET_FLOW_RATE = 2.0
TARGET_WEIGHT = 40
BREW_TEMP = 90
STEAM_TEMP = 125

def pre_infuse():
    set_pump_level(PRE_INF_LEVEL)

def ramp():
    set_pump_level(RAMP_LEVEL)

def brew(flow, target):
    level = target / flow
    set_pump_level(level)
    return level

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

    temp_targ = BREW_TEMP
    flow = 0.0
    total_flow = 0.0
    start = time.ticks_ms()
    seconds = 0.0
    pump_level = 0
    pres_targ = 9.0
    
    last_time = None
    last_error = None
    last_integral = 0.0

    while True:
        cur_temp = poll_temp()
        cur_pres = poll_pressure()

        temp_targ = STEAM_TEMP if not SWT_MODE.value() else BREW_TEMP
        if (not SWT_BREW.value()) and SWT_MODE.value():
            if not timing:
                timing = True
                start = time.ticks_ms()
                seconds = 0.0
                total_flow = 0.0
                
            if timing and total_flow <= TARGET_WEIGHT:
                seconds = time.ticks_diff(time.ticks_ms(), start) / 1000
                open_valve()
                
                if seconds <= PRE_INF_DUR:
                    pump_level = PRE_INF_LEVEL
                    flow = calc_flow(cur_pres, pump_level)
                    pre_infuse()
                elif seconds <= PRE_INF_DUR + RAMP_DUR:
                    pump_level = RAMP_LEVEL
                    flow = calc_flow(cur_pres, pump_level)
                    total_flow += 0.1 * flow
                    ramp()
                else:
                    flow = calc_flow(cur_pres, pump_level)
                    pump_level = brew(flow, TARGET_FLOW_RATE)
                    total_flow += 0.1 * flow
                    
            else:
                pump_off()
                close_valve()
                flow = 0.0
                    
        else:
            timing = False
            pump_off()
            close_valve()
            flow = 0.0

        set_temp = temp_targ
        if timing and seconds <= PRE_INF_DUR: set_temp += 1
        last_time, last_error, last_integral = temp_control(
            set_temp, cur_temp, last_time, last_error, last_integral
        )

        update_display(cur_temp, cur_pres, flow, temp_targ, pres_targ, total_flow, seconds)
        time.sleep(0.1)


test()
