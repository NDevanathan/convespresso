import datetime as dt
from abc import ABCMeta, abstractmethod
from multiprocessing import Event, Pipe, Process, Queue

import numpy as np
import pause

from lib.serial_courier import SerialCourier

FREQ   = 60 #Hz
PERIOD = 1/FREQ
DELTA = dt.timedelta(seconds=PERIOD) #seconds
PERIODS_PER_FRAME = 15

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
        i = 0
        start = dt.datetime.now()
        next = start
        brew_time = 0
        
        while True:
            self.state_queue.put(self.comms.get_state())
            while self.targ_pipe.poll():
                self.targets = self.targ_pipe.recv()
            while self.act_pipe.poll():
                self.action = self.act_pipe.recv()

            if not self.brew_event.is_set():
                start = next
                self.action[1] = 0.
                self.comms.close_valve()
            else:
                brew_time = (next-start).total_seconds()

            self.comms.take_action(self.action[0],self.action[1])
            if i % PERIODS_PER_FRAME == 0: self.comms.refresh_display(
                'ESPRESSO',
                self.targets[0],
                self.targets[1],
                self.targets[2],
                brew_time
            )
            
            i += 1
            next += DELTA
            pause.until(next)


class LowPassFilter:
    def __init__(self, alpha_per_second):
        self.alpha_per_second = alpha_per_second
        self.state = None
        self.last_time = None

    def apply(self, time, value):
        if self.state is None:
            self.state = value
            self.last_time = time
            return value
        else:
            dt = time - self.last_time
            self.last_time = time
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

        alpha = 1/15
        beta = 1/5000
        gamma = 1/20

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
        if secs <= 10:
            return 1/4
        elif secs <= 15:
            return 1/2
        
        flow = self.calc_flow()
        pump_level = (1.5 * self.state[3] / flow) ** (0.5)

        if pump_level > 3/4: pump_level = 3/4
        self.targets[2] += flow * PERIOD
        return pump_level

    def run(self):
        start = dt.datetime.now()
        next = start
        while True:
            while not self.state_queue.empty():
                self.state = self.state_queue.get()

            action = [0., 0.]
            action[0] = self.temp_control((next-start).total_seconds())

            if self.brew_event.is_set():
                action[1] = self.flow_control((next-start).total_seconds())
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
    cont_proc = PID(act_pipe_cont, targ_pipe_cont, state_queue, brew_event)
    comm_proc.start()
    cont_proc.start()
    input("Hit ENTER to start brewing.")
    brew_event.set()
    input("Hit ENTER to stop brewing.")
    brew_event.clear()
    # act_pipe_cont, act_pipe_comm = Pipe()
    # targ_pipe_cont, targ_pipe_comm = Pipe()
    # raw_state_queue = Queue()
    # filtered_state_queue = Queue()
    # brew_event = Event()

    # alpha_per_second = 0.8
    # filter_obj = LowPassFilter(alpha_per_second)

    # comm_proc = CommProcess(act_pipe_comm, targ_pipe_comm, raw_state_queue, brew_event)
    # filter_proc = FilterProcess(raw_state_queue, filtered_state_queue, filter_obj)
    # cont_proc = PID(act_pipe_cont, targ_pipe_cont, filtered_state_queue, brew_event)

    # comm_proc.start()
    # filter_proc.start()
    # cont_proc.start()
