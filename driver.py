from espresso import EspressoMachine, BrewState

em = EspressoMachine()

def get_state():
    print(
        em.poll_temp(),
        em.poll_pressure(), 
        em.state.heat_level, 
        em.state.pump_level
    )

def take_action(heat_level, pump_level):
    em.set_heat_level(heat_level)
    em.set_pump_level(pump_level)

def refresh_display(mode, temp_targ, pres_targ, mass_targ, sec):
    em.update_display(mode, temp_targ, pres_targ, mass_targ, sec)
