from time import sleep
from machine import Pin, SPI, ADC
from ssd1309 import Display
from xglcd_font import XglcdFont


BTN_SAVE  = Pin(21, Pin.IN, Pin.PULL_UP)
BTN_BREW    = Pin(20, Pin.IN, Pin.PULL_UP)
SWT_MODE  = Pin(11, Pin.IN, Pin.PULL_UP)
SWT_BREW    = Pin(10, Pin.IN, Pin.PULL_UP)
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
HEAT_CTRL = Pin(9, Pin.OUT)
SOL_CTRL  = Pin(22, Pin.OUT)
TEMP_SCK  = Pin(2)
TEMP_CS   = Pin(1)
TEMP_MISO = Pin(0)
PUMP_ZC   = Pin(8, Pin.IN)
PUMP_PSM  = Pin(7, Pin.OUT)

FONT = XglcdFont('res/FixedFont5x8.c', 5, 8)
WIDTH = 128
HEIGHT = 64


def test():
    spi = SPI(1, baudrate=10000000, sck=SCR_SCK, mosi=SCR_MOSI)
    display = Display(spi, dc=SCR_DC, cs=SCR_CS, rst=SCR_RST)
    
    display.draw_bitmap("res/cvx.mono", WIDTH // 2 - 50, HEIGHT // 2 - 25, 100, 50, rotate=180)
    display.present()
    sleep(3)
    
    display.clear()
    
    while True:
        if not SWT_BREW.value():
            display.draw_text(WIDTH - 5,30,"VENT",FONT,rotate=180)
            SOL_CTRL.on()
            PUMP_PSM.on()
        else:
            display.draw_text(WIDTH - 5,30,"VENT",FONT,invert=True,rotate=180)
            SOL_CTRL.off()
            PUMP_PSM.off()
    
        display.present()
        sleep(0.1)


test()
