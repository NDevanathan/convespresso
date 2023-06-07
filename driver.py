from espresso import EspressoMachine, BrewState

# Initialize an EspressoMachine object
em = EspressoMachine()

def get_state():
    """
    This function gets the current state of the EspressoMachine including
    temperature, pressure, heat level, and pump level.
    """
    print(
        em.poll_temp(),
        em.poll_pressure(),
        em.state.heat_level,
        em.state.pump_level
    )

def take_action(heat_level, pump_level):
    """
    This function sets the heat level and pump level of the EspressoMachine.

    Params:
    heat_level (int): The desired heat level for the EspressoMachine
    pump_level (int): The desired pump level for the EspressoMachine
    """
    em.set_heat_level(heat_level)
    em.set_pump_level(pump_level)

def refresh_display(mode, temp_targ, pres_targ, mass_targ, sec):
    """
    This function updates the display values for the EspressoMachine.

    Params:
    mode (int): The display mode for the EspressoMachine (e.g. coffee making, idle)
    temp_targ (float): The desired temperature to display
    pres_targ (float): The desired pressure to display
    mass_targ (float): The desired mass to display
    sec (int): The desired time duration in seconds to display
    """
    em.update_display(mode, temp_targ, pres_targ, mass_targ, sec)
  
def close_valve():
    """
    This function closes the valve of the EspressoMachine.
    """
    em.close_valve()
