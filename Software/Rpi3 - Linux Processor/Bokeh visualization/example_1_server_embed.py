# This Python file uses the following encoding: utf-8
import numpy as np
from numpy import pi

from bokeh.client import push_session
from bokeh.driving import cosine
from bokeh.plotting import figure, curdoc

from bokeh.layouts import column, gridplot
from bokeh.models import Range1d

x = np.linspace(0, 4*pi, 80)
y = np.sin(x)

# Roll graph.
plot_roll_yrange = Range1d(start=-180, end=180)
plot_roll_xrange = Range1d(start=0, end=120)

plot_roll = figure(width=750, plot_height=250, title='Roll', y_range = plot_roll_yrange, x_range = plot_roll_xrange)
plot_roll.yaxis.axis_label = 'Roll [°]'
plot_roll.xaxis.axis_label = 'Tiempo [s]'
roll_line = plot_roll.line([0, 4*pi], [-1, 1], color="navy")

# Pitch Graph
plot_pitch_yrange = Range1d(start=-180, end=180)
plot_pitch_xrange = Range1d(start=0, end=120)

plot_pitch = figure(width=750, plot_height=250, title='Pitch', y_range = plot_pitch_yrange, x_range = plot_pitch_xrange)
plot_pitch.yaxis.axis_label = 'Pitch [°]'
plot_pitch.xaxis.axis_label = 'Tiempo [s]'
pitch_line = plot_pitch.line([0, 4*pi], [-1, 1], color="navy")

# Yaw Graph
plot_yaw_yrange = Range1d(start=-180, end=180)
plot_yaw_xrange = Range1d(start=0, end=120)

plot_yaw = figure(width=750, plot_height=250, title='Yaw', y_range = plot_yaw_yrange, x_range = plot_yaw_xrange)
plot_yaw.yaxis.axis_label = 'Yaw [°]'
plot_yaw.xaxis.axis_label = 'Tiempo [s]'
yaw_line = plot_pitch.line([0, 4*pi], [-1, 1], color="navy")


# Motor force graph
plot_motor_yrange = Range1d(start=0, end=100)
plot_motor_xrange = Range1d(start=0, end=120)

plot_motor = figure(width=750, plot_height=250, title='Potencia de motores', y_range = plot_motor_yrange, x_range = plot_motor_xrange)
plot_motor.yaxis.axis_label = 'Potencia de motores [%]'
plot_motor.xaxis.axis_label = 'Tiempo [s]'
motor_line_1 = plot_motor.line([0, 4*pi], [-1, 1], color="blue")
motor_line_2 = plot_motor.line([0, 4*pi], [-1, 1], color="green")
motor_line_3 = plot_motor.line([0, 4*pi], [-1, 1], color="red")
motor_line_4 = plot_motor.line([0, 4*pi], [-1, 1], color="yellow")


# open a session to keep our local document in sync with server
session = push_session(curdoc())

@cosine(w=0.03)
def update(step):
    # updating a single column of the the *same length* is OK
    pitch_line.data_source.data["y"] = y * step * 20
    # pitch_line.glyph.line_alpha = 1 - 0.8 * abs(step)

curdoc().add_periodic_callback(update, 50)

session.show(gridplot([[plot_roll,plot_motor],[plot_pitch,None],[plot_yaw,None]])) # open the document in a browser

session.loop_until_closed() # run forever
