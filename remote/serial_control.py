import pause
import numpy as np
import datetime as dt
from multiprocessing import Process, Pipe, Queue, Event

from lib.controller import Controller

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
        self.comms = Controller()
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
            
class ControlProcess(Process):
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
        
    def calc_flow(self):
        full_flow = sum([flow_coefs[i] * self.state[1]**i for i in range(len(flow_coefs))]) / 60
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
            while not state_queue.empty():
                self.state = state_queue.get()
        
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
    cont_proc = ControlProcess(act_pipe_cont, targ_pipe_cont, state_queue, brew_event)
    comm_proc.start()
    cont_proc.start()