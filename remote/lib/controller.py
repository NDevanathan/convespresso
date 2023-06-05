import serial

@dataclass
class State:
    temperature: float,
    pressure: float:,
    heat_level: float,
    pump_level: float

@dataclass
class Action:
    heat_level: float,
    pump_level: float

class Controller:
    TERMINATOR = '\r'.encode('UTF8')

    def __init__(self, timeout=1):
        self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=timeout)
        self.send("from main import *")

    def send(self, text: str):
        line = '%s\r\f' % text
        self.serial.write(line.encode('utf-8'))
        reply = self.receive()
        #reply = reply.replace('>>> ','') # lines after first will be prefixed by a propmt
        #if reply != text: # the line should be echoed, so the result should match
        #    raise ValueError('expected %s got %s' % (text, reply))

    def receive(self) -> str:
        line = self.serial.read_until(self.TERMINATOR)
        return line.decode('UTF8').strip()

    def close(self):
        self.serial.close()
        
    def poll_state(self):
        self.send("get_state()")
        reply = self.receive()
        return [float(r) for r in reply.split("\n")]
        
    def send_action(self, heat_level: float, pump_level: float):
        self.send("get_state({},{})".format(heat_level, pump_level))
