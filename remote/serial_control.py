import datetime as dt
from abc import ABCMeta, abstractmethod
from multiprocessing import Event, Pipe, Process, Queue

import cvxpy as cp
import numpy as np
import pause

from remote.lib.serial_courier import SerialCourier

FREQ   = 60 #Hz
PERIOD = 1/FREQ
DELTA = dt.timedelta(seconds=PERIOD) #seconds

class CommProcess(Process):
    def __init__(self, act_pipe, targ_pipe, state_queue, brew_event, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.act_pipe = act_pipe
        self.targ_pipe = targ_pipe
        self.state_queue = state_queue
        self.brew_event = brew_event
        self.comms = SerialCourier()
        self.action = [0.,0.]
        self.targets = [95., 9., 0.]

    def run(self):
        start = dt.datetime.now()
        next = start
        while True:
            self.state_queue.put(self.comms.get_state())
            if self.targ_pipe.poll():
                self.targets = self.targ_pipe.recv()
            if self.act_pipe.poll():
                self.action = self.act_pipe.recv()

            if not self.brew_event.is_set():
                start = next
                self.action[1] = 0.
                self.comms.close_valve()

            self.comms.take_action(self.action[0],self.action[1])
            self.comms.refresh_display(
                'ESPRESSO',
                self.targets[0],
                self.targets[1],
                self.targets[2],
                (next-start).total_seconds()
            )
            print(dt.datetime.now())
            next += DELTA
            pause.until(next)


class LowPassFilter:
    def __init__(self, alpha_per_second):
        self.alpha_per_second = alpha_per_second
        self.state = None

    def apply(self, dt, value):
        if self.state is None:
            self.state = value
            return value
        else:
            alpha = 1 - np.exp(-self.alpha_per_second * dt)
            self.state = alpha * value + (1 - alpha) * self.state
            return self.state


# class FilterProcess(Process):
#     def __init__(self, input_queue, output_queue, filter_obj, *args, **kwargs):
#         super().__init__(*args, **kwargs)
#         self.input_queue = input_queue
#         self.output_queue = output_queue
#         self.filter = filter_obj

#     def run(self):
#         while True:
#             time, raw_state = self.input_queue.get()
#             filtered_state = [self.filter.apply(time, s) for s in raw_state]
#             self.output_queue.put((time, filtered_state))


class Controller(Process, metaclass=ABCMeta):
    def __init__(self, act_pipe, targ_pipe, state_queue, brew_event, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.act_pipe = act_pipe
        self.targ_pipe = targ_pipe
        self.state_queue = state_queue
        self.brew_event = brew_event
        self.state = np.zeros(4)

        self.targets = [94.5, 9., 0.]
        self.flow_coefs = [6.4634e+02, -7.0024e+01,  4.6624e+00, -1.9119e-01]

        self.last_error = None
        self.last_integral = 0.

    @abstractmethod
    def calc_flow(self):
        pass

    @abstractmethod
    def temp_control(self, secs):
        pass

    @abstractmethod
    def flow_control(self, secs):
        pass

    @abstractmethod
    def run(self):
        pass


class PID(Controller):
    def calc_flow(self):
        full_flow = sum([self.flow_coefs[i] * self.state[1]**i for i in range(len(self.flow_coefs))]) / 60
        return full_flow * self.state[3]

    def temp_control(self, secs):
        target = self.targets[0]
        curr_temp = self.state[0]

        alpha = 1/30
        beta = 1/10000
        gamma = 1/50

        error = target - curr_temp

        proportional = error
        derivative = 0.
        integral = self.last_integral

        if not (self.last_error is None):
            derivative = (error - self.last_error) / PERIOD
            integral += self.last_error * PERIOD

        self.last_error = error
        return alpha*proportional + beta*integral + gamma*derivative

    def flow_control(self, secs):
        flow = self.calc_flow()
        pump_level = (2 * pump_level / flow) ** (0.5)

        if pump_level > 3/4: pump_level = 3/4
        self.targets[3] += flow * PERIOD
        return pump_level

    def run(self):
        start = dt.datetime.now()
        next = start
        while True:
            while not self.state_queue.empty():
                self.state = self.state_queue.get()

            action = [0., 0.]
            action[0] = self.temp_control(next - start)

            if self.brew_event.is_set():
                action[1] = self.flow_control(next - start)
            else:
                self.targets[2] = 0
                start = next

            self.act_pipe.send(action)
            self.targ_pipe.send(self.targets)
            next += DELTA
            pause.until(next)


class IndependentMRAC(Controller):
    def __init__(self, Am, Bm, cm, filter, kx, kr, gamma, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # We assume Am, Bm, cm, kx, kr, gamma are numpy arrays.
        # Each contains a hyperparameter for temperature first and
        # for pressure second.
        self.Am = Am
        self.Bm = Bm
        self.cm = cm

        self.kx = kx
        self.kr = kr
        self.gamma = gamma

        self.filter = filter
        self.r = None

    def temp_control(self, secs):
        # Implement MRAC temperature control method here
        return self.controls[0]

    def flow_control(self, secs):
        # Implement MRAC flow control method here
        return self.controls[1]

    def update_model(self):
        # Implement MRAC model update method here
        err = self.state - self.targets[:2]
        self.kx += -self.gamma * err * self.state * DELTA
        self.kr += -self.gamma * err * self.r * DELTA
        self.controls = self.kx * self.state + self.kr * self.r

    def compute_reference(self):
        r_cvx = cp.Variable()

    def run(self):
        start = dt.datetime.now()
        next = start
        while True:
            while not self.state_queue.empty():
                obs = self.state_queue.get()
                self.state = self.filter.apply(dt=DELTA, value=obs)

            # Compute a reference signal if necessary
            if self.r is None:
                self.compute_reference()

            # Update MRAC model parameters if necessary
            self.update_model()

            action = [0., 0.]
            action[0] = self.temp_control(next - start)

            if self.brew_event.is_set():
                action[1] = self.flow_control(next - start)
            else:
                self.targets[2] = 0
                start = next

            self.act_pipe.send(action)
            self.targ_pipe.send(self.targets)
            next += DELTA
            pause.until(next)


if __name__ == '__main__':
    act_pipe_cont, act_pipe_comm = Pipe()
    targ_pipe_cont, targ_pipe_comm = Pipe()
    state_queue = Queue()
    brew_event = Event()
    comm_proc = CommProcess(act_pipe_comm, targ_pipe_comm, state_queue, brew_event)
    cont_proc = IndependentMRAC(act_pipe_cont, targ_pipe_cont, state_queue, brew_event)
    comm_proc.start()
    cont_proc.start()
