from monitor import Monitor
from time import sleep

# Creating the instance of Oscilloscope Monitor
monitor = Monitor()

data = 10

# Start listening for messages and plot the stream
while True:
    sleep(0.5)
    monitor.add_data([data])

    if data == 10:
        data = 0
    else:
        data = 10