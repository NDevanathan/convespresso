import time
from espresso import EspressoMachine, BrewState

em = EspressoMachine(BrewState(
    start = time.ticks_ms(),
    seconds = 0.,
    temperature = poll_temp(),
    temp_targ = BREW_TEMP,
    pressure = poll_pressure(),
    pres_targ = 9.,
    flow = 0.,
    flow_targ = FLOW_RATE[0],
    total_flow = 0.,
    mass_targ = TOTAL_MASS[0],
    pump_level = 0.,
    heat_level = 0.
))

def get_state():
    print(em.poll_temp(), 
        em.poll_pressure(), 
        em.state.heat_level, 
        em.state.pump_level
    )


def take_action(heat_level, pump_level):
    em.set_heat_level(heat_level)
    em.set_pump_level(pump_level)


def refresh_display(mode):
    em.update_display(mode)
