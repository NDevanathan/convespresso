from multiprocessing import Process

from lib.controller import Controller

FREQ   = 60 #Hz
PERIOD = 1/FREQ #seconds

pico_comms = Controller()
