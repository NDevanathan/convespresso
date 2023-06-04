from lib.controller import Controller

pico_comms = Controller()
pico_comms.send("adjust_temp(83.)")
print(pico_comms.receive())
print(pico_comms.receive())
print(pico_comms.receive())
print(pico_comms.receive())
print(pico_comms.receive())
