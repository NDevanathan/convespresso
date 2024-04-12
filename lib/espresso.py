# Import necessary libraries and modules
import time
import _thread
from machine import Pin, SPI, ADC, PWM
from ssd1309 import Display
from max6675 import MAX6675
from xglcd_font import XglcdFont

# Define pins for buttons and switches on the espresso machine
BTN_SAVE  = Pin(21, Pin.IN, Pin.PULL_UP)  # Save button
BTN_BREW  = Pin(20, Pin.IN, Pin.PULL_UP)  # Brew button
SWT_PINF  = Pin(11, Pin.IN, Pin.PULL_UP)  # Pre-infuse switch
SWT_BREW  = Pin(10, Pin.IN, Pin.PULL_UP)  # Brew switch
POT1      = ADC(27)                       # Potentiometer 1 (connects with microcontroller) 
POT2      = ADC(26)                       # Potentiometer 2 (connects with microcontroller)
POT1_POLL = Pin(17, Pin.OUT)              # Potentiometer 1 polling pin
POT2_POLL = Pin(18, Pin.OUT)              # Potentiometer 2 polling pin

# Define pins for the display
SCR_SCK   = Pin(14)                       # Display clock pin
SCR_MOSI  = Pin(15)                       # Display MOSI pin
SCR_CS    = Pin(13)                       # Display chip select pin
SCR_DC    = Pin(12)                       # Display data/command pin
SCR_RST   = Pin(6)                        # Display reset pin (Not connected, placeholder)

# Define pins for the sensors, heater, pump, and valve
PRS_SENS  = ADC(28)                        # Pressure sensor pin
HEAT_PWM  = PWM(Pin(9, Pin.OUT), freq=6000)  # Heating element pulse width modulation pin
SOL_CTRL  = Pin(22, Pin.OUT)               # Solenoid valve control pin
TEMP_SCK  = Pin(2, Pin.OUT)                # Temperature probe clock pin
TEMP_CS   = Pin(1, Pin.OUT)                # Temperature probe chip select pin
TEMP_SO   = Pin(0, Pin.OUT)                # Temperature probe serial output pin
PUMP_ZC   = Pin(8, Pin.IN)                 # Water pump zero crossing pin
PUMP_PWM  = PWM(Pin(7, Pin.OUT), freq=6000)  # Water pump pulse width modulation pin

# Define display settings
FONT    = XglcdFont('res/FixedFont5x8.c', 5, 8)  # Display font
WIDTH   = 128                                    # Display width
HEIGHT  = 64                                     # Display height
START_X = WIDTH - 10                             # Display starting x-coordinate
START_Y = 45                                     # Display starting y-coordinate

# Coefficients for polynomial model of flow with respect to pressure
flow_coefs = [6.4634e+02, -7.0024e+01,  4.6624e+00, -1.9119e-01]

class BrewState:
    def __init__(self,
        temperature: float = 25.,  # Default temperature (Celsius)
        pressure   : float =  1.,  # Default pressure (bars)
        flow       : float =  0.,  # Default flow (ml/s)
        pump_level : float =  0.,  # Default pump power level (0-1)
        heat_level : float =  0.): # Default heater power level (0-1)

        self.temperature = temperature
        self.pressure    = pressure
        self.flow        = flow
        self.pump_level  = pump_level
        self.heat_level  = heat_level

class EspressoMachine():
    """
    This class represents an espresso machine, with methods to control and monitor its functioning.
    """

    def __init__(self, state: BrewState = BrewState()):
        """
        Constructor - initializes the desired state for the espresso machine.
        """
        self.state = state
        self.spi1        = SPI(1, baudrate=10000000, sck=SCR_SCK, mosi=SCR_MOSI)
        self.display     = Display(self.spi1, dc=SCR_DC, cs=SCR_CS, rst=SCR_RST)
        self.temp_probe  = MAX6675(sck=TEMP_SCK, cs=TEMP_CS, so=TEMP_SO)

    def is_brewing(self) -> bool:
        return not SWT_BREW.value()
        
    def is_preinf(self) -> bool:
        return not SWT_PINF.value()
        
    def poll_temp(self) -> float:
        """
        Read the current temperature from the temperature probe in Celsius.

        Returns
        -------
        float
            The current temperature in Celsius.
        """
        self.state.temperature = self.temp_probe.read()
        return self.state.temperature

    def heat_on(self) -> int:
        """
        Turn on the heating element.

        Returns
        -------
        int
            The heater power level (1 for on).
        """
        self.state.heat_level = 1
        HEAT_PWM.duty_u16(65535)
        return 1

    def heat_off(self) -> int:
        """
        Turn off the heating element.

        Returns
        -------
        int
            The heater power level (0 for off).
        """
        self.state.heat_level = 0
        HEAT_PWM.duty_u16(0)
        return 0

    def set_heat_level(self, level: float) -> float:
        """
        Set the heating element's power level (between 0 and 1).

        Parameters
        ----------
        level : float
            Power level for the heating element (0-1).

        Returns
        -------
        float
            The actual set power level for the heating element.
        """
        if level < 0:
            level = 0
        elif level > 1:
            level = 1

        self.state.heat_level = level
        HEAT_PWM.duty_u16(int(level * 65535))
        return level

    def poll_pressure(self) -> float:
        """
        Read the current pressure from the pressure sensor in bars.

        Returns
        -------
        float
            Pressure reading in bars.
        """
        reading = PRS_SENS.read_u16()
        volts = (reading * 5) / 65536
        bars = (volts - 0.5) * 3
        self.state.pressure = bars
        return bars

    def calc_flow(self, pressure: float, power_level: float) -> float:
        """
        Calculates the flow (ml/s) based on the measured pressure and pump power level.

        Parameters
        ----------
        pressure : float
            Pressure in bars.
        power_level : float
            Water pump power level (0-1).

        Returns
        -------
        float
            Flow (ml/s) calculated based on pressure and pump power level.
        """
        flow = sum([flow_coefs[i] * pressure**i for i in range(len(flow_coefs))]) / 60
        flow *= power_level
        self.state.flow = flow
        return flow

    def pump_on(self) -> int:
        """
        Turn on the water pump.

        Returns
        -------
        int
            Water pump power level (1 for on).
        """
        self.open_valve() #safeguard
        self.state.pump_level = 1
        PUMP_PWM.duty_u16(65535)
        return 1

    def pump_off(self) -> int:
        """
        Turn off the water pump.

        Returns
        -------
        int
            Water pump power level (0 for off).
        """
        self.state.pump_level = 0
        PUMP_PWM.duty_u16(0)
        return 0

    def set_pump_level(self, level: float) -> float:
        """
        Set the water pump's power level (between 0 and 1).

        Parameters
        ----------
        level : float
            Water pump power level (0-1).

        Returns
        -------
        float
            The actual set power level for the water pump.
        """
        if level < 0:
            level = 0
        elif level > 1:
            level = 1

        if level > 0: self.open_valve() #safeguard
        self.state.pump_level = level
        PUMP_PWM.duty_u16(int(level * 65535))
        return level

    def open_valve(self):
        """
        Open the solenoid valve.
        """
        SOL_CTRL.on()

    def close_valve(self):
        """
        Close the solenoid valve.
        """
        self.pump_off()
        SOL_CTRL.off()

    def boot_screen(self):
        """
        Display boot screen on the display.
        """
        self.display.draw_bitmap("res/cvx.mono", WIDTH // 2 - 50, HEIGHT // 2 - 25, 100, 50, rotate=180)
        self.display.present()
        time.sleep(3)
        self.display.clear()

    def update_display(self, mode: str, temp_targ: float, pres_targ: float, mass_targ: float, sec: float):
        """
        Update the display with the current temperature, pressure, flow, and timer values, as well as their 
        respective target values.

        Parameters
        ----------
        mode : str 
            The espresso machine mode.
        temp_targ : float
            The target temperature (Celsius).
        pres_targ : float
            The target pressure (bars).
        mass_targ : float
            The target mass of the extraction (grams).
        sec : float
            The timer value (seconds).

        """
        
        def subroutine(d, m, temperature, pressure, flow, t_targ, p_targ, m_targ, s):
            # Clear display buffers
            d.clear_buffers()
            trackers = [temperature, pressure, flow]
            targets = [t_targ, p_targ, m_targ]

            # Set up header labels
            headers = ["TEMP.", "PRES.", "FLOW."]
            values = ["{0:.1f}".format(val) for val in trackers]
            targets = ["{0:.1f}".format(val) for val in targets]
            offset = 0

            # Display tracked values and targets
            for i in range(3):
                d.draw_text(START_X - offset, START_Y, headers[i], FONT, rotate=180)
                d.draw_text(START_X - offset - 6 * (5 - len(values[i])), 
                    START_Y - 10, values[i], FONT, rotate=180)
                d.draw_text(START_X - offset - 6 * (5 - len(targets[i])), 
                    START_Y - 20, targets[i], FONT, rotate=180)
                
                offset += 39

            # Display the timer and mode text
            sec_str = "{0:.1f}".format(s)
            d.draw_text(START_X - 20, START_Y - 32, "TIME:", FONT, rotate=180)
            d.draw_text(START_X - 60 - 6 * (5 - len(sec_str)), START_Y - 32, sec_str, FONT, rotate=180)
            d.draw_text(WIDTH//2 + 3 * (len(m)), START_Y - 42, m, FONT, rotate=180)
            d.present()
            
        # Create and start a new thread for updating the display
        display_thread = _thread.start_new_thread(
            subroutine,
            (
                self.display, mode,
                self.state.temperature,
                self.state.pressure,
                self.state.flow, temp_targ,
                pres_targ, mass_targ, sec
            )            
        )

