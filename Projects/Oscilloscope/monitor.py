from matplotlib import pyplot
import numpy as np

# Using ggplot style for more sophisticated visuals
pyplot.style.use('ggplot')

class Monitor():
    """ Monitor class is a class which prints using Matplotlib
        data being captured by an external agent in a Figure.
    """

    # Static flag, check whether the Matplotlib Module has already
    # been set on interactive mode or not...
    ion_is_on = False

    # Static parameters of configuration for the Oscilloscope
    OVERSAMPLING_FACTOR = 10

    def __init__(self, ylim=(-12, 12), xlabel='X Label', ylabel='Y Label', title='Title', size=100):
        # Configuration of the Oscilloscope monitor
        self.size = size

        # Variables needed to save the x, y plot data
        self.x_data = np.linspace(0, size * Monitor.OVERSAMPLING_FACTOR - 1, size * Monitor.OVERSAMPLING_FACTOR)
        self.y_data = np.zeros(size)

        # Turn the interactive mode on of the Matplotlib Module
        if not Monitor.ion_is_on:
            pyplot.ion()
            Monitor.ion_is_on = True
        
        # Create the Figure and Axes to plot with Matplotlib
        self.figure = pyplot.figure(figsize=(13, 6))
        self.axes = self.figure.add_subplot(111)

        # Create the line plot to be updated
        data = np.zeros(self.size * Monitor.OVERSAMPLING_FACTOR)
        self.line, = self.axes.plot(self.x_data, data, alpha=0.8)

        # Updating the plot, label, title and other things
        pyplot.ylim(ylim)
        pyplot.xlabel(xlabel)
        pyplot.ylabel(ylabel)
        pyplot.title(title)
        pyplot.show()
    
    def add_data(self, new_values):
        """ Add new sampled data to the oscilloscope, shifting values in the logged sequence
            @param new_values   List of new vales to be added
        """
        # Initialization of index variable to operate the list
        i = 0

        # Shifting values in the array
        while (i < self.size - len(new_values)):
            self.y_data[i] = self.y_data[i + len(new_values)]
            i = i + 1
        
        # Pushing new values into the Oscilloscope array
        self.y_data[i:] = new_values

        # Update data
        self._update_ydata()

    """ Private Methods """
    def _update_ydata(self):
        # Realod with oversampling to hold samples
        data = np.zeros(self.size * Monitor.OVERSAMPLING_FACTOR)
        for i in range(self.size):
            for j in range(Monitor.OVERSAMPLING_FACTOR):
                data[i * Monitor.OVERSAMPLING_FACTOR + j] = self.y_data[i]

        # Update values in the plot and pause to catch the updates
        self.line.set_ydata(data)
        pyplot.pause(0.1)

        

