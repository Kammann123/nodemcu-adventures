from oscilloscope import Oscilloscope

# Request to the user the Port Number and Baud Rate used
port = input('Enter the Port used, check on device management (COM0, COM1, ...) =>  ')
baudrate = input('Enter the Baud Rate used, check on device configuration (9600, ...) =>  ')

# Instance and run the oscilloscope
osc = Oscilloscope(port, baudrate)
osc.run()