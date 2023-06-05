import pause
import datetime as dt
from multiprocessing import Process, Queue

FREQ   = 100 #Hz
PERIOD = dt.timedelta(seconds=1/FREQ)

class CommProcess(Process):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.action = [0.,0.]
        
    def run(self):
        start = dt.datetime.now()
        for _ in range(1000):
            print(start)
            start += PERIOD
            pause.until(start)
            
            
'''class ControlProcess(Process):
    def __init__(self, act_pipe, state_queue, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.act_pipe = act_pipe
        self.state_queue = state_queue
        self.action = [0.,0.]
        
    def run(self):
        start = time.perf_counter()
        while condition:'''
             

if __name__ == '__main__':
    p = CommProcess()
    p.start()
    p.join(timeout=10)
    
