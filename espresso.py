from machine import Pin, SPI, ADC
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
HEAT_CTRL = Pin(9, Pin.OUT)
SOL_CTRL  = Pin(22, Pin.OUT)
TEMP_SCK  = Pin(2, Pin.OUT)
TEMP_CS   = Pin(1, Pin.OUT)
TEMP_SO   = Pin(0, Pin.OUT)
PUMP_ZC   = Pin(8, Pin.IN)
PUMP_PSM  = Pin(7, Pin.OUT)

FONT   = XglcdFont('res/FixedFont5x8.c', 5, 8)
WIDTH  = 128
HEIGHT = 64

spi1 = SPI(1, baudrate=10000000, sck=SCR_SCK, mosi=SCR_MOSI)
display = Display(spi1, dc=SCR_DC, cs=SCR_CS, rst=SCR_RST)
temp_probe = MAX6675(sck=TEMP_SCK, cs=TEMP_CS, so=TEMP_SO) 


def poll_temp():
    return temp_probe.read()

def heat_on():
    HEAT_CTRL.on()

def heat_off():
    HEAT_CTRL.off()
    
def set_heat_level():
    return
    
def poll_pressure():
    reading = PRS_SENS.read_u16()
    volts = (reading * 5) / 65536
    bars = (volts - 0.5) * 3
    return bars

def pump_on():
    PUMP_PSM.on()
    
def pump_off():
    PUMP_PSM.off()

def set_pump_level():
    return

def open_valve():
    SOL_CTRL.on()
    
def close_valve():
    SOL_CTRL.off()
