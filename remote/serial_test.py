import pause
import datetime as dt
from multiprocessing import Process, Pipe, Queue, Event

from lib.controller import Controller

FREQ   = 60 #Hz
PERIOD = dt.timedelta(seconds=1/FREQ) #seconds

pico_comms = Controller()

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
            if self.brew_event:
                self.state_queue.put(self.comms.get_state())
                if self.act_pipe.poll():
                    self.action = self.act_pipe.recv()
                if self.targ_pipe.poll():
                    self.targets = self.targ_pipe.recv()
                self.comms.take_action(self.action[0],self.action[1])
            else:
                start = next
                
            self.comms.refresh_display(
                'ESPRESSO', 
                self.targets[0], 
                self.targets[1], 
                self.targets[2], 
                (next-start).total_seconds()
            )
            next += PERIOD
            pause.until(next)
            
class ControlProcess(Process):
    def __init__(self, act_pipe, targ_pipe, state_queue, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.act_pipe = act_pipe
        self.targ_pipe = targ_pipe
        self.state_queue = state_queue

if name == '__main__':
    act_pipe_cont, act_pipe_comm = Pipe()
    targ_pipe_cont, targ_pipe_comm = Pipe()
    state_queue = Queue()
    brew_event = Event()
    comm_proc = CommProcess(act_pipe_comm, targ_pipe_comm, state_queue, brew_event)
    #cont_proc = ControlProcess(act_pipe_cont, targ_pipe_cont, state_queue)
    comm_proc.start()
