#!/usr/bin/env python

import matplotlib
matplotlib.use('GtkAgg')
import matplotlib.pyplot as plt
import numpy
import time
import os

# ugly function required for old version of matplotlib
def pause(plt, interval):
    figManager = plt._pylab_helpers.Gcf.get_active()
    if figManager is not None:
        canvas = figManager.canvas
        canvas.draw()
        was_interactive = plt.isinteractive()
        if not was_interactive:
            plt.ion()
            plt.show(False)
        canvas.start_event_loop(interval)
        if not was_interactive:
            plt.ioff()
        return

plt.figure(1);
#line, = plt.plot(0,0);
plt.draw()
plt.hold(False)

for i in range(0,1000):
    file_name = '/Network/Servers/titian/Volumes/titian/kalakris/.ros/noiseless_%d.txt'%(i)
    data = numpy.genfromtxt(file_name)
    plt.hold(False)
    plt.plot(data[:,0], data[:,1], 'g', aa=True)

    for j in range(0,20):
        file_name = '/Network/Servers/titian/Volumes/titian/kalakris/.ros/noisy_%d_%d.txt'%(i,j)
        if not os.path.exists(file_name):
          continue
        data2 = numpy.genfromtxt(file_name)
        plt.hold(True)
        plt.plot(data2[:,0], data2[:,1], 'r', linewidth=0.1, aa=True)

    plt.axis([0,1,0,1])
    print i
    #line.set_xdata(data[:,0]);
    #line.set_ydata(data[:,1]);
    #plt.draw()
    pause(plt, 0.01)
    
plt.show()
