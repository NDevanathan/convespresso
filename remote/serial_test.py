import pause
import numpy as np
import datetime as dt

from lib.serial_courier import SerialCourier

FREQ   = 100 #Hz
PERIOD = 1/FREQ
DELTA = dt.timedelta(seconds=PERIOD) #seconds

comms = SerialCourier()
action = [0.,0.]
targets = [95., 9., 0.]

it = 0
start = dt.datetime.now()
next = start
while True:
    comms.take_action(action[0], action[1])
    if it % 10 == 0: comms.refresh_display(
        'ESPRESSO', 
        targets[0], 
        targets[1], 
        targets[2], 
        (next-start).total_seconds()
    )
    it += 1
    state = comms.get_state()
    next += DELTA
    pause.until(next)
    print(dt.datetime.now())
            
