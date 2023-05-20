import time
from espresso import *

def temp_control(target, elapsed):
    if elapsed >= 6:
        target += 2

    alpha = 1/20
    if poll_temp() >= target:
        heat_off()
    elif poll_temp() <= target:
        control = alpha * (target - poll_temp())
        if control <= 2/6:
            control = 2/6
            
        set_heat_level(control)
        
def pressure_control(target, elapsed):
    if elapsed < 6:
        set_pump_level(1/6)
    else:
        set_pump_level(4/6)

def test():
    display.draw_bitmap("res/cvx.mono", WIDTH // 2 - 50, HEIGHT // 2 - 25, 100, 50, rotate=180)
    display.present()
    time.sleep(3)
    display.clear()

    temp_targ = 92.0
    pres_targ = 9.0
    time_targ = 30.0
    start = time.time()
    seconds = 0.0
    timing = False
    
    while True:
        display.clear_buffers()
        display.draw_text(START_X,START_Y,"TEMP.",FONT,rotate=180)
        display.draw_text(START_X-39,START_Y,"PRES.",FONT,rotate=180)
        display.draw_text(START_X-78,START_Y,"TIMER",FONT,rotate=180)
        
        if not SWT_BREW.value():
            if not timing:
                timing = True
                start = time.time()
        else:
            timing = False
            
        if timing:
            seconds = time.time() - start
            
        
        if not SWT_BREW.value():
            open_valve()
            pressure_control(pres_targ, seconds)
        else:
            pump_off()
            close_valve()
    
        if not SWT_MODE.value():
            if timing:
                temp_control(temp_targ, seconds)
            else:
                temp_control(temp_targ, 0)
        else:
            heat_off()
        
            
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
