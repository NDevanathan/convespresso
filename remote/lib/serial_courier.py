import serial

class SerialCourier:
    """
    SerialCourier class is responsible for managing the serial communication
    between the current script and the Espresso Machine through its driver
    implemented on an Arduino board.
    """
    TERMINATOR = '\r'.encode('UTF8')

    def __init__(self, timeout=1):
        """
        Constructor method that sets up the serial communication.

        Args:
            timeout (int): A value to determine the maximum waiting time
                           for data reception on serial communication.
        """
        self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=timeout)
        self.send("from main import *")

    def send(self, text: str):
        """
        Send text data to the Arduino connected via serial communication.

        Args:
            text (str): Text data to be sent.
        """
        line = '%s\r\f' % text
        self.serial.write(line.encode('utf-8'))
        reply = self.receive()

    def receive(self) -> str:
        """
        Receive text data from the Arduino through serial communication.

        Returns:
            str: Received data as a string.
        """
        line = self.serial.read_until(self.TERMINATOR)
        return line.decode('UTF8').strip()

    def flush(self):
        """
        Clear the input and output buffer of the serial communication.
        """
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

    def close(self):
        """
        Close the serial communication.
        """
        self.serial.close()

    def get_state(self):
        """
        Request the current state of the Espresso Machine which includes
        temperature, pressure, heat level, and pump level.

        Returns:
            List[float]: A list containing the machine's current state.
        """
        self.flush()
        self.send("get_state()")
        reply = self.receive()
        return [float(r) for r in reply.split(" ")]

    def take_action(self, heat_level: float, pump_level: float):
        """
        Set the heat level and pump level of the Espresso Machine.

        Args:
            heat_level (float): The desired heat level for the Espresso Machine.
            pump_level (float): The desired pump level for the Espresso Machine.
        """
        self.send("take_action({},{})".format(heat_level, pump_level))
        
    def refresh_display(self, mode, temp_targ, pres_targ, mass_targ, sec):
        """
        Update the display values of the Espresso Machine.

        Args:
            mode (str): The display mode for the Espresso Machine (e.g. coffee making, idle).
            temp_targ (float): The desired temperature to display.
            pres_targ (float): The desired pressure to display.
            mass_targ (float): The desired mass to display.
            sec (int): The desired time duration in seconds to display.
        """
        self.send("refresh_display('{}',{},{},{},{})".format(mode, temp_targ, pres_targ, mass_targ, sec))
        
    def close_valve(self):
        """
        Request the Arduino to close the valve of the Espresso Machine.
        """
        self.send("close_valve()")
