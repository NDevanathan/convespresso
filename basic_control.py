from time import sleep
from espresso import *


def test():
    display.draw_bitmap("res/cvx.mono", WIDTH // 2 - 50, HEIGHT // 2 - 25, 100, 50, rotate=180)
    display.present()
    sleep(3)
    
    display.clear()
    
    while True:
        display.clear_buffer()
        display.draw_text(WIDTH - 20,30,"TEMP.",FONT,rotate=180)
        display.draw_text(WIDTH - 60,30,"PRES.",FONT,rotate=180)
        display.draw_text(WIDTH - 20,20,"{0:.1f}".format(poll_temp()),FONT,rotate=180)
        display.draw_text(WIDTH - 60,20,"{0:.1f}".format(poll_pressure()),FONT,rotate=180)
        
        if not SWT_BREW.value():
            open_valve()
            pump_on()
        else:
            pump_off()
            close_valve()
    
        if not SWT_MODE.value():
            heat_on()
        else:
            heat_off()
        
        display.present()
        sleep(0.1)


test()
