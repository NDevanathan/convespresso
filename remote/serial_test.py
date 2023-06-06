import pause
import numpy as np
import datetime as dt

from lib.controller import Controller

FREQ   = 60 #Hz
PERIOD = 1/FREQ
DELTA = dt.timedelta(seconds=PERIOD) #seconds

comms = Controller()
action = [0.,0.]
targets = [95., 9., 0.]
    
start = dt.datetime.now()
next = start
while True:
    comms.take_action(action[0], action[1])
    comms.refresh_display(
        'ESPRESSO', 
        targets[0], 
        targets[1], 
        targets[2], 
        (next-start).total_seconds()
    )
    state = comms.get_state()
    print(dt.datetime.now())
    next += DELTA
    pause.until(next)
            
