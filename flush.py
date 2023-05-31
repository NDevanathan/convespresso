import time
from espresso import *

HEAT_LEVEL = 1
DELTA = 0.1

VALS_TO_TRACK = [
    'seconds', 'pressure', 'temperature', 'pump_level', 'heat_level'
]
MAX_STORAGE = 10000
OVERWRITE = True # If false, then data are simply stopped being collected


def main_loop():
    boot_screen()

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
    gamma = 1

    while True:
        state.temperature = gamma * poll_temp() + (1 - gamma) * state.temperature
        state.pressure = gamma * poll_pressure() + (1 - gamma) * state.pressure

        if not SWT_BREW.value():
            # turn on heat
            open_valve()
            pump_on()
        else:
            # turn off heat
            close_valve()
            pump_off()

        update_display(
            state.temperature,
            state.pressure,
            state.flow,
            state.temp_targ,
            state.pres_targ,
            state.total_flow,
            state.seconds,
            "FLUSH"
        )

        time.sleep(DELTA)


main_loop()
