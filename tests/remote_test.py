import time
from espresso import *

state = BrewState(
    start = time.ticks_ms(),
    seconds = 0.,
    temperature = poll_temp(),
    temp_targ = 95.,
    pressure = poll_pressure(),
    pres_targ = 9.,
    flow = 0.,
    flow_targ = 2.,
    total_flow = 0.,
    mass_targ = 30.,
    pump_level = 0.,
    heat_level = 0.
)

update_display(
    state.temperature,
    state.pressure,
    state.flow,
    state.temp_targ,
    state.pres_targ,
    state.total_flow,
    state.seconds,
    "TEST"
)

def adjust_temp(new_temp_targ):
    state.temp_targ = new_temp_targ
    update_display(
        state.temperature,
        state.pressure,
        state.flow,
        state.temp_targ,
        state.pres_targ,
        state.total_flow,
        state.seconds,
        "TEST"
    )
    
def get_temp():
    temp = poll_temp()
    print(temp)
    state.temperature = temp
    update_display(
        state.temperature,
        state.pressure,
        state.flow,
        state.temp_targ,
        state.pres_targ,
        state.total_flow,
        state.seconds,
        "TEST"
    )

