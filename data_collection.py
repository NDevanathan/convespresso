import time
from espresso import *

PRE_INF_DUR = 8
RAMP_DUR = 5
PRE_INF_LEVEL = 1/4
RAMP_LEVEL = 1/2
TOTAL_MASS = [36, 36, 18]
FLOW_RATE = [1.5, 1.5, 0.75]
MODES = ["STEAM", "DATA OFF", "DATA ON"]
BREW_TEMP = [130, 90, 90]
DELTA = 0.01

VALS_TO_TRACK = [
    'seconds', 'pressure', 'temperature', 'pump_level', 'heat_level'
]
MAX_STORAGE = 50000
OVERWRITE = True # If false, then data are simply stopped being collected

last_time = None
last_error = None
last_integral = 0.
def temp_control(
    target,
    curr_temp#,
    #last_time=None,
    #last_error=None,
    #last_integral=0.,
):
    alpha = 1/5
    beta = 1/2000
    gamma = 1/20

    curr_time = time.ticks_ms()
    error = target - curr_temp

    proportional = error
    derivative = 0.
    integral = last_integral

    if not (last_time is None):
        sec_diff = time.ticks_diff(curr_time, last_time) / 1000.
        if sec_diff > 0:
            derivative = (error - last_error) / sec_diff
        integral += last_error * sec_diff

    control = alpha*proportional + beta*integral + gamma*derivative

    state.heat_level = set_heat_level(control)
    #return curr_time, error, integral

def pre_infuse(state):
    state.pump_level = PRE_INF_LEVEL
    state.flow = calc_flow(state.pressure, state.pump_level)
    state.seconds = time.ticks_diff(time.ticks_ms(), state.start) / 1000
    set_pump_level(PRE_INF_LEVEL)
    return state

def ramp_up(state):
    state.pump_level = RAMP_LEVEL
    state.flow = calc_flow(state.pressure, state.pump_level)
    delta = (time.ticks_diff(time.ticks_ms(), state.start) / 1000) - state.seconds
    state.seconds += delta
    state.total_flow += delta * state.flow
    set_pump_level(RAMP_LEVEL)
    return state

def hold_flow(state):
    #if state.total_flow < state.mass_targ:
    state.flow = calc_flow(state.pressure, state.pump_level)
    state.pump_level = (state.flow_targ * state.pump_level / state.flow) ** (0.5)

    if state.pump_level > RAMP_LEVEL: state.pump_level = RAMP_LEVEL
    delta = (time.ticks_diff(time.ticks_ms(), state.start) / 1000) - state.seconds
    state.seconds += delta
    state.total_flow += delta * state.flow
    set_pump_level(state.pump_level)

    #else:
    #    pump_off()
    #    state.flow = 0

    return state

brew_modes = [pre_infuse, ramp_up, hold_flow]
def brew(mode, state):
    temp_control(state.temp_targ, state.temperature)

    if mode < 0:
        pump_off()
        close_valve()
        state.flow = 0
        return state
    else:
        open_valve()
        return brew_modes[mode](state)

def main_loop():
    boot_screen()

    state = BrewState(
        start = time.ticks_ms(),
        seconds = 0.,
        temperature = poll_temp(),
        temp_targ = BREW_TEMP[0],
        pressure = poll_pressure(),
        pres_targ = 9.,
        flow = 0.,
        flow_targ = FLOW_RATE[0],
        total_flow = 0.,
        mass_targ = TOTAL_MASS[0],
        pump_level = 0.,
        heat_level = 0.
    )
    gamma = 1
    brew_mode = -1

    hists = {}
    for val in VALS_TO_TRACK:
        hists[val] = []

    while True:
        state.temperature = gamma * poll_temp() + (1 - gamma) * state.temperature
        state.pressure = gamma * poll_pressure() + (1 - gamma) * state.pressure

        # mode = SWT_MODE.value()
        # state.flow_targ = FLOW_RATE[mode]
        # state.mass_targ = TOTAL_MASS[mode]
        # state.temp_targ = BREW_TEMP[mode]

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
            state.heat_level = set_heat_level(1)
        else:
            # turn off heat
            state.heat_level = set_heat_level(0)

        # state = brew(brew_mode, state)

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
