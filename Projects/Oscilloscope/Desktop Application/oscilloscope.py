from monitor import Monitor
from serial import Serial
import numpy as np

class Oscilloscope():
    """ Oscilloscope class, opens a Serial Port with the given configuration and starts a Monitor
        printer with Matplotlib, and plots each bit of the byte received via UART protocol.
    """

    BYTE_LENGTH = 8

    def __init__(self, port='COM0', baudrate='9600'):

        # Create the pySerial instance, and open the serial port
        self.serial = Serial(port, baudrate)
        if (self.serial.isOpen() == False):
            self.serial.open()

        # Create the internal monitor
        self.monitor = Monitor()
    
    def run(self):
        """ Starts running the Oscilloscope on the Serial Port,
            listening each byte received from the measuring device and updating
            the plot.
        """
        is_running = True
        while is_running:
            try:
                # Wait the next byte frame from device
                data_frame = ord(self.serial.read())

                # Parse the byte as 8 digital samples
                data_array = np.zeros(Oscilloscope.BYTE_LENGTH)
                for i in range(Oscilloscope.BYTE_LENGTH):
                    data_array[Oscilloscope.BYTE_LENGTH - 1 - i] = 1 if data_frame & 0x01 > 0 else 0
                    data_frame = data_frame >> 1
                

                # Update oscilloscope print
                self.monitor.add_data(data_array)
            except KeyboardInterrupt:
                is_running = False