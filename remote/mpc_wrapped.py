from lib.serial_control import Controller
from lib.mpc import TPTrackerMPC
from lib.mhe import MHE


class TPTracker(Controller):
    
    def __init__(self, act_pipe, targ_pipe, state_queue, brew_event, A, B, c, c1, c2, target_p, target_T, C, x0, mhe_horizon, H=None, enable_input_constraints=True,):
        super().__init__(act_pipe, targ_pipe, state_queue, brew_event)

        self.mhe = MHE(A, B, c, C, x0, mhe_horizon)
        self.controller = TPTrackerMPC(A, B, c, c1, c2, target_p, target_T, H=H, enable_input_constraints=enable_input_constraints)

    def calc_flow(self):
        pass

    def temp_control(self, secs):
        pass

    def flow_control(self, secs):
        pass
    
    def run(self, secs):
        pass
