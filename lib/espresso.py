# Import necessary libraries and modules
import time
from machine import Pin, SPI, ADC, PWM
from ssd1309 import Display
from max6675 import MAX6675
from xglcd_font import XglcdFont

# Define pins for buttons and switches on the espresso machine
BTN_SAVE  = Pin(21, Pin.IN, Pin.PULL_UP)
BTN_BREW  = Pin(20, Pin.IN, Pin.PULL_UP)
SWT_MODE  = Pin(11, Pin.IN, Pin.PULL_UP)
SWT_BREW  = Pin(10, Pin.IN, Pin.PULL_UP)
POT1      = ADC(27)
POT2      = ADC(26)
POT1_POLL = Pin(17, Pin.OUT)
POT2_POLL = Pin(18, Pin.OUT)

# Define pins for the display
SCR_SCK   = Pin(14)
SCR_MOSI  = Pin(15)
SCR_CS    = Pin(13)
SCR_DC    = Pin(12)
SCR_RST   = Pin(6)  # Not connected (placeholder)

# Define pins for the sensors, heater, pump, and valve
PRS_SENS  = ADC(28)
HEAT_PWM  = PWM(Pin(9, Pin.OUT), freq=10)
SOL_CTRL  = Pin(22, Pin.OUT)
TEMP_SCK  = Pin(2, Pin.OUT)
TEMP_CS   = Pin(1, Pin.OUT)
TEMP_SO   = Pin(0, Pin.OUT)
PUMP_ZC   = Pin(8, Pin.IN)
PUMP_PWM  = PWM(Pin(7, Pin.OUT), freq=10)

# Define display settings
FONT    = XglcdFont('res/FixedFont5x8.c', 5, 8)
WIDTH   = 128
HEIGHT  = 64
START_X = WIDTH - 10
START_Y = 45

# Initialize the display
spi1 = SPI(1, baudrate=10000000, sck=SCR_SCK, mosi=SCR_MOSI)
display = Display(spi1, dc=SCR_DC, cs=SCR_CS, rst=SCR_RST)

# Initialize the temperature sensor (MAX6675)
temp_probe = MAX6675(sck=TEMP_SCK, cs=TEMP_CS, so=TEMP_SO)

# Coefficients for polynomial model of flow w.r.t. pressure
flow_coefs = [6.4634e+02, -7.0024e+01,  4.6624e+00, -1.9119e-01]

class BrewState:
    def __init__(self, 
        start:       int,
        seconds:     float,
        temperature: float,
        temp_targ:   float,
        pressure:    float,
        pres_targ:   float,
        flow:        float,
        flow_targ:   float,
        total_flow:  float,
        mass_targ:   float,
        pump_level:  float):
        
        self.start       = start
        self.seconds     = seconds
        self.temperature = temperature
        self.temp_targ   = temp_targ
        self.pressure    = pressure
        self.pres_targ   = pres_targ
        self.flow        = flow
        self.flow_targ   = flow_targ
        self.total_flow  = total_flow
        self.mass_targ   = mass_targ
        self.pump_level  = pump_level

def poll_temp():
    """
    Read the current temperature from the temperature probe in Celcius.
    """
    return temp_probe.read()


def heat_on():
    """
    Turn on the heating element.
    """
    HEAT_PWM.duty_u16(65535)


def heat_off():
    """
    Turn off the heating element.
    """
    HEAT_PWM.duty_u16(0)


def set_heat_level(level: float):
    """
    Set the heating element's power level (between 0 and 1).
    """
    if level < 0:
        level = 0
    elif level > 1:
        level = 1

    HEAT_PWM.duty_u16(int(level * 65535))


def poll_pressure():
    """
    Read the current pressure from the pressure sensor in bars.
    """
    reading = PRS_SENS.read_u16()
    volts = (reading * 5) / 65536
    bars = (volts - 0.5) * 3
    return bars


def calc_flow(pressure, power_level):
    """
    Calculates the flow (ml/s) based on the measured pressue and pump power level.
    """
    flow = sum([flow_coefs[i] * pressure**i for i in range(len(flow_coefs))]) / 60
    flow *= power_level
    return flow


def pump_on():
    """
    Turn on the water pump.
    """
    PUMP_PWM.duty_u16(65535)


def pump_off():
    """
    Turn off the water pump.
    """
    PUMP_PWM.duty_u16(0)


def set_pump_level(level: float):
    """
    Set the water pump's power level (between 0 and 1).
    """
    if level < 0:
        level = 0
    elif level > 1:
        level = 1

    PUMP_PWM.duty_u16(int(level * 65535))


def open_valve():
    """
    Open the solenoid valve.
    """
    SOL_CTRL.on()


def close_valve():
    """
    Close the solenoid valve.
    """
    SOL_CTRL.off()


def boot_screen():
    """
    Display boot screen on the display.
    """
    display.draw_bitmap("res/cvx.mono", WIDTH // 2 - 50, HEIGHT // 2 - 25, 100, 50, rotate=180)
    display.present()
    time.sleep(3)
    display.clear()

    
def boot_screen2():
    """
    Display special boot screen on the display.
    """
    display.draw_bitmap("res/zach.mono", 0, 0, 128, 64, rotate=180)
    display.present()
    time.sleep(5)
    display.clear()


def update_display(temp, pres, flow, temp_targ, pres_targ, flow_targ, sec, mode):
    """
    Update the display with the current temperature, pressure, flow, and timer values, as well as their respective target values.
    """
    display.clear_buffers()

    headers = ["TEMP.", "PRES.", "FLOW."]
    values = ["{0:.1f}".format(val) for val in [temp, pres, flow]]
    targets = ["{0:.1f}".format(val) for val in [temp_targ, pres_targ, flow_targ]]
    offset = 0

    for i in range(3):
        display.draw_text(START_X - offset, START_Y, headers[i], FONT, rotate=180)
        display.draw_text(START_X - offset - 6 * (5 - len(values[i])), START_Y - 10, values[i], FONT, rotate=180)
        display.draw_text(START_X - offset - 6 * (5 - len(targets[i])), START_Y - 20, targets[i], FONT, rotate=180)
        offset += 39

    sec_str = "{0:.1f}".format(sec)
    display.draw_text(START_X - 20, START_Y - 32, "TIME:", FONT, rotate=180)
    display.draw_text(START_X - 60 - 6 * (5 - len(sec_str)), START_Y - 32, sec_str, FONT, rotate=180)
    display.draw_text(WIDTH//2 + 3 * (len(mode)), START_Y - 42, mode, FONT, rotate=180)
    display.present()
