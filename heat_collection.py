import time
from espresso import *

HEAT_LEVEL = [1/6]
DELTA = 0.1

VALS_TO_TRACK = [
    'seconds', 'temperature', 'heat_level'
]
MAX_STORAGE = 500
OVERWRITE = False # If false, then data are written to a file and the cache is reset


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

    runtime = time.time_ns()
    data_runs = 0
    prev_swt_mode_value = 1

    while True:
        state.temperature = gamma * poll_temp() + (1 - gamma) * state.temperature
        state.pressure = gamma * poll_pressure() + (1 - gamma) * state.pressure

        if not SWT_MODE.value():
            # start collecting data
            prev_swt_mode_value = 0
            delta = (time.ticks_diff(time.ticks_ms(), state.start) / 1000) - state.seconds
            state.seconds += delta
            for name in VALS_TO_TRACK:
                # val = getattr(state, name)
                # file = open(f'logs/log_{name}_{runtime}.txt', 'a')
                # file.write(f'{str(val)}\n')
                # file.close()
                hist = hists[name]
                if len(hist) < MAX_STORAGE:
                    hist.append(getattr(state, name))
                elif len(hist) == MAX_STORAGE and OVERWRITE:
                    hist.pop(0)
                    hist.append(getattr(state, name))
                elif len(hist) == MAX_STORAGE:
                    file = open(f'logs/log_{name}_{runtime}.txt', 'a')
                    for val in hist:
                        file.write(f'{str(val)}\n')
                    file.close()
                    hists[name] = []
        elif prev_swt_mode_value == 0:
            prev_swt_mode_value = 1
            for name, hist in hists.items():
                if len(hist) > 0:
                    file = open(f'logs/log_{name}_{runtime}.txt', 'a')
                    for val in hist:
                        file.write(f'{str(val)}\n')
                    file.close()
                hists[name] = []
            runtime = time.time_ns()
            data_runs += 1
            state.seconds = 0.
            state.start = time.ticks_ms()
        else:
            # save data (if data > 0)
            state.seconds = 0.
            state.start = time.ticks_ms()

        if not SWT_BREW.value():
            # turn on heat
            set_heat_level(HEAT_LEVEL[data_runs])
            state.heat_level = HEAT_LEVEL[data_runs]
        else:
            # turn off heat
            heat_off()
            state.heat_level = 0.

        if not BTN_SAVE.value():
            open_valve()
            pump_on()
        else:
            pump_off()
            close_valve()

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
