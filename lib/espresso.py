import time
from machine import Pin, SPI, ADC, PWM
from ssd1309 import Display
from max6675 import MAX6675
from xglcd_font import XglcdFont


BTN_SAVE  = Pin(21, Pin.IN, Pin.PULL_UP)
BTN_BREW  = Pin(20, Pin.IN, Pin.PULL_UP)
SWT_MODE  = Pin(11, Pin.IN, Pin.PULL_UP)
SWT_BREW  = Pin(10, Pin.IN, Pin.PULL_UP)
POT1      = ADC(27)
POT2      = ADC(26)
POT1_POLL = Pin(17, Pin.OUT)
POT2_POLL = Pin(18, Pin.OUT)

SCR_SCK   = Pin(14)
SCR_MOSI  = Pin(15)
SCR_CS    = Pin(13)
SCR_DC    = Pin(12)
SCR_RST   = Pin(6) #Not connected (placeholder)

PRS_SENS  = ADC(28)
#HEAT_CTRL = Pin(9, Pin.OUT)
HEAT_PWM  = PWM(Pin(9, Pin.OUT), freq=10)
SOL_CTRL  = Pin(22, Pin.OUT)
TEMP_SCK  = Pin(2, Pin.OUT)
TEMP_CS   = Pin(1, Pin.OUT)
TEMP_SO   = Pin(0, Pin.OUT)
PUMP_ZC   = Pin(8, Pin.IN)
#PUMP_PSM  = Pin(7, Pin.OUT)
PUMP_PWM  = PWM(Pin(7, Pin.OUT), freq=10)

FONT    = XglcdFont('res/FixedFont5x8.c', 5, 8)
WIDTH   = 128
HEIGHT  = 64
START_X = WIDTH - 10
START_Y = 35

spi1 = SPI(1, baudrate=10000000, sck=SCR_SCK, mosi=SCR_MOSI)
display = Display(spi1, dc=SCR_DC, cs=SCR_CS, rst=SCR_RST)
temp_probe = MAX6675(sck=TEMP_SCK, cs=TEMP_CS, so=TEMP_SO) 


def poll_temp():
    return temp_probe.read()


def heat_on():
    HEAT_PWM.duty_u16(65535)


def heat_off():
    HEAT_PWM.duty_u16(0)


def set_heat_level(level: float):
    if level < 0:
        level = 0
    elif level > 1:
        level = 1
        
    HEAT_PWM.duty_u16(int(level*65535))


def poll_pressure():
    reading = PRS_SENS.read_u16()
    volts = (reading * 5) / 65536
    bars = (volts - 0.5) * 3
    return bars


def pump_on():
    PUMP_PWM.duty_u16(65535)


def pump_off():
    PUMP_PWM.duty_u16(0)


def set_pump_level(level: float):
    if level < 0:
        level = 0
    elif level > 1:
        level = 1
        
    PUMP_PWM.duty_u16(int(level*65535))


def open_valve():
    SOL_CTRL.on()


def close_valve():
    SOL_CTRL.off()


def boot_screen():
    display.draw_bitmap("res/cvx.mono", WIDTH // 2 - 50, HEIGHT // 2 - 25, 100, 50, rotate=180)
    display.present()
    time.sleep(3)
    display.clear()

def update_display(temp, pres, sec, temp_targ, pres_targ, sec_targ):
    display.clear_buffers()
    
    headers = ["TEMP.","PRES.","TIMER"]
    values = ["{0:.1f}".format(val) for val in [temp, pres, sec]]
    targets = ["{0:.1f}".format(val) for val in [temp_targ, pres_targ, sec_targ]]
    offset = 0
    
    for i in range(3):
        display.draw_text(START_X-offset,START_Y,headers[i],FONT,rotate=180)
        display.draw_text(START_X-offset-6*(5-len(values[i])),START_Y-10,values[i],FONT,rotate=180)
        display.draw_text(START_X-offset-6*(5-len(targets[i])),START_Y-20,targets[i],FONT,rotate=180)
        offset += 39

    display.present()
    
