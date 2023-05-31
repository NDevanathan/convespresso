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

    hists = {}
    for val in VALS_TO_TRACK:
        hists[val] = []

    while True:
        state.temperature = gamma * poll_temp() + (1 - gamma) * state.temperature
        state.pressure = gamma * poll_pressure() + (1 - gamma) * state.pressure

        if not SWT_MODE.value():
            # start collecting data
            delta = (time.ticks_diff(time.ticks_ms(), state.start) / 1000) - state.seconds
            state.seconds += delta
            for name, hist in hists.items():
                if len(hist) < MAX_STORAGE:
                    hist.append(getattr(state, name))
                elif len(hist) == MAX_STORAGE and OVERWRITE:
                    hist.pop(0)
                    hist.append(getattr(state, name))
        else:
            # save data (if data > 0)
            state.seconds = 0.
            state.start = time.ticks_ms()
            for name, hist in hists.items():
                if len(hist) > 0:
                    file = open(f'logs/log_{name}_{time.localtime()}.txt', 'w')
                    for val in hist:
                        file.write(f'{str(val)}\n')
                    file.close()

        if not SWT_BREW.value():
            # turn on heat
            state.heat_level = set_heat_level(HEAT_LEVEL)
        else:
            # turn off heat
            state.heat_level = set_heat_level(0)

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

        time.sleep(DELTA)


main_loop()
