from matplotlib import pyplot
import numpy as np

# Using ggplot style for more sophisticated visuals
pyplot.style.use('ggplot')

def OscilloscopeMonitor(Object):
    """ OscilloscopeMonitor class is a class which prints using Matplotlib
        data being captured by an external agent in a Figure.
    """

    # Static flag, check whether the Matplotlib Module has already
    # been set on interactive mode or not...
    self.ion_is_on = False

    def __init__(self, xlabel='X Label', ylabel='Y Label', title='Title', size=200):
        # Configuration of the Oscilloscope monitor
        self.size = size

        # Variables needed to save the x, y plot data
        self.x_data = np.linspace(0, size - 1, size)
        self.y_data = np.array([0 for i in range(self.size)])

        # Turn the interactive mode on of the Matplotlib Module
        if self.ion_is_on:
            pyplot.ion()
            self.ion_is_on = True
        
        # Create the Figure and Axes to plot with Matplotlib
        self.figure = pyplot.figure(figsize=(13, 6))
        self.axes = self.figure.add_subplot(111)

        # Create the line plot to be updated
        self.line, = self.axes.plot(self.x_data, self.y_data, alpha=0.8)

        # Updating the plot, label, title and other things
        pyplot.xlabel(xlabel)
        pyplot.ylabel(ylabel)
        pyplot.title(title)
        pyplot.show()
    
    def sample():