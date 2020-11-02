from pylive import live_plotter
import numpy as np

# Variable initialization
size = 100
x_vec = np.linspace(0,1,size+1)[0:-1]
y_vec = np.linspace(0,0,size+1)[0:-1]
line1 = []

# Ask the user for the current UART serial port

# Start listening for messages and plot the stream
while True:
    # rand_val = np.random.randn(1)
    # y_vec[-1] = rand_val
    line1 = live_plotter(x_vec,y_vec,line1, identifier='Oscilloscope Traze')
    # y_vec = np.append(y_vec[1:], 0.0)