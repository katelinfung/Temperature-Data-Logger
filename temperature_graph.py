# temperature_graph.py
# Date: March 5th 2021
# Displays Temperature Vs Time on a Stripchart

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys, time, math
import serial

xsize=30

# configure the serial port
ser = serial.Serial(
    #port='COM3', # Change as needed
    port='COM7', # Change as needed
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS
)
ser.isOpen()
   
def data_gen():
    t = data_gen.t
    while True:
       t+=0.5
       strin = ser.readline()
       print (strin.decode('utf-8'))
       num = float(strin.decode('utf-8'))
       print (num)
       print(type(num))
       val = 100.0*(num-2.73)
       #val = 8000*num-21828
       print("val", val)
       yield t, val

def run(data):
    # update the data
    t,y = data
    if t>-1:
        xdata.append(t)
        ydata.append(y)
        if t>xsize: # Scroll to the left.
            ax.set_xlim(t-xsize, t)
        line.set_data(xdata, ydata)

    return line,

def on_close_figure(event):
    sys.exit(0)

data_gen.t = -1
fig = plt.figure()
fig.canvas.mpl_connect('close_event', on_close_figure)
ax = fig.add_subplot(111)
line, = ax.plot([], [], lw=2)
ax.set_ylim(-5, 40)
ax.set_xlim(0, xsize)
ax.grid()
xdata, ydata = [], []

# Titles and Axis Labels
plt.title("Temperature") 
plt.xlabel("Time (Seconds)") 
plt.ylabel("Temperature (Degrees Celsius)") 


# Important: Although blit=True makes graphing faster, we need blit=False to prevent
# spurious lines to appear when resizing the stripchart.
ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=100, repeat=False)
plt.show()
