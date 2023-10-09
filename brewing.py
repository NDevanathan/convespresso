import time
import math
from espresso import EspressoMachine, BrewState

# Brew phase controls
PRE_INF_ON = 4.
PRE_INF_OFF = 2.

# Brew targets
TEMP_TARG = 95.
TEMP_AMB = 24.5
PRESS_TARG = 9.

# Loop frequency controls
PERIODS_PER_FRAME = 5
FREQ = 30
PERIOD = 1/FREQ
DELTA = 1000//FREQ


class LowPassFilter:
    def __init__(self, alpha_per_second):
        self.alpha_per_second = alpha_per_second
        self.state = None

    def apply(self, dt, value):
        if self.state is None:
            self.state = value
            return value
        else:
            alpha = 1 - math.exp(-self.alpha_per_second * dt)
            self.state = alpha * value + (1 - alpha) * self.state
            return self.state

class Brewer():
    def __init__(self):
        # cooling parameters
        self.k_amb = -0.005
        self.k_flow = -0.01
        
        # heating parameters
        self.alpha = 3
        self.beta = 6
        self.gamma = 0.25
        self.heat_effect = 0
        
        self.em = EspressoMachine()
        self.temp_filter = LowPassFilter(0.8)
        self.press_filter = LowPassFilter(0.8)
        self.total_flow = 0

    def get_state(self):
        """
        This function gets the current state of the EspressoMachine including
        temperature, pressure, heat level, and pump level.
        """
        return [
            self.em.poll_temp(),
            self.em.poll_pressure(),
            self.em.state.heat_level,
            self.em.state.pump_level
        ]

    def take_action(self, heat_level, pump_level):
        """
        This function sets the heat level and pump level of the EspressoMachine.

        Params:
        heat_level (int): The desired heat level for the EspressoMachine
        pump_level (int): The desired pump level for the EspressoMachine
        """
        self.em.set_heat_level(heat_level)
        self.em.set_pump_level(pump_level)

    def refresh_display(self, mode, temp_targ, pres_targ, mass_targ, sec):
        """
        This function updates the display values for the EspressoMachine.

        Params:
        mode (int): The display mode for the EspressoMachine (e.g. coffee making, idle)
        temp_targ (float): The desired temperature to display
        pres_targ (float): The desired pressure to display
        mass_targ (float): The desired mass to display
        sec (int): The desired time duration in seconds to display
        """
        self.em.update_display(mode, temp_targ, pres_targ, mass_targ, sec)
      
    def close_valve(self):
        """
        This function closes the valve of the EspressoMachine.
        """
        self.em.close_valve()

    def temp_control(self):
        t_targ = TEMP_TARG
        t_amb = TEMP_AMB
        t_curr = self.state[0]
        
        flow = self.em.calc_flow(self.state[1], self.state[3]) * PERIOD
        k = self.k_amb + self.k_flow * flow
        t_cool = (t_curr - t_amb) * math.exp(k) + t_amb 
        
        self.heat_effect = self.gamma * self.state[2] + (1 - self.gamma) * self.heat_effect
        diff = (t_targ - (t_cool + self.alpha * self.heat_effect))
        return diff / self.beta

    def press_control(self):
        return 1.

    def run(self):
        start = time.ticks_ms()
        next = start
        shot_time = 0
        i = 0
        brewing = False
        
        while True:
            self.state = self.get_state()
            self.state[0] = self.temp_filter.apply(dt=PERIOD, value=self.state[0])
            self.state[1] = self.press_filter.apply(dt=PERIOD, value=self.state[1])
            flow = self.em.calc_flow(self.state[1], self.state[3]) * PERIOD

            action = [0.0, 0.0]
            action[0] = self.temp_control()

            if self.em.is_brewing():
                PRESS_TARG = 9.
                if not brewing:
                    self.total_flow = 0
                
                if time.ticks_diff(next, start) / 1000 < PRE_INF_ON:
                    action[1] = 0.5
                elif time.ticks_diff(next, start) / 1000 < PRE_INF_ON + PRE_INF_OFF:
                    action[1] = 0.
                else:
                    action[1] = self.press_control()
                    self.total_flow += flow
                
                brewing = True
                shot_time = time.ticks_diff(next, start) / 1000
            else:
                self.em.close_valve()
                PRESS_TARG = 0
                start = next
                brewing = False

            # Act on decided policy
            self.take_action(action[0], action[1])
            if i % PERIODS_PER_FRAME == 0:
                self.refresh_display(
                    "ESPRESSO",
                    TEMP_TARG,
                    PRESS_TARG,
                    self.total_flow,
                    shot_time,
                )
            
            i += 1
            
            # Iterate at specified rate
            next = time.ticks_add(next, DELTA)
            while time.ticks_diff(next, time.ticks_ms()) > 0:
                time.sleep_us(10)
            

if __name__ == "__main__":
    brewer = Brewer()
    brewer.run()
