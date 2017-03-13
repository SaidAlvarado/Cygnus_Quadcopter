# This Python file uses the following encoding: utf-8
import numpy as np
from numpy import pi
import time

from bokeh.client import push_session
from bokeh.driving import cosine
from bokeh.plotting import figure, curdoc

from bokeh.layouts import column, gridplot
from bokeh.models import Range1d
############################################## ROS imports  #########################
# Import the float array message
import rospy
import tf
from geometry_msgs.msg import Pose
##################################################################################

#Initialize variables
current_roll = 0
current_pitch = 0
current_yaw = 0
current_motor_1 = 0
current_motor_2 = 0
current_motor_3 = 0
current_motor_4 = 0
time_frame = [0]
vector_roll = [0]
vector_pitch = [0]
vector_yaw = [0]
vector_motor_1 = [0]
vector_motor_2 = [0]
vector_motor_3 = [0]
vector_motor_4 = [0]
# Intialize time
start_time = time.time()
current_time = 0






# Roll graph.
plot_roll_yrange = Range1d(start=-180, end=180)
plot_roll_xrange = Range1d(start=0, end=120)

plot_roll = figure(width=750, plot_height=250, title='Roll', y_range = plot_roll_yrange, x_range = plot_roll_xrange)
plot_roll.yaxis.axis_label = 'Roll [°]'
plot_roll.xaxis.axis_label = 'Tiempo [s]'
roll_line = plot_roll.line(vector_roll, time_frame, color="navy")

# Pitch Graph
plot_pitch_yrange = Range1d(start=-180, end=180)
plot_pitch_xrange = Range1d(start=0, end=120)

plot_pitch = figure(width=750, plot_height=250, title='Pitch', y_range = plot_pitch_yrange, x_range = plot_pitch_xrange)
plot_pitch.yaxis.axis_label = 'Pitch [°]'
plot_pitch.xaxis.axis_label = 'Tiempo [s]'
pitch_line = plot_pitch.line(vector_pitch, time_frame, color="navy")

# Yaw Graph
plot_yaw_yrange = Range1d(start=-180, end=180)
plot_yaw_xrange = Range1d(start=0, end=120)

plot_yaw = figure(width=750, plot_height=250, title='Yaw', y_range = plot_yaw_yrange, x_range = plot_yaw_xrange)
plot_yaw.yaxis.axis_label = 'Yaw [°]'
plot_yaw.xaxis.axis_label = 'Tiempo [s]'
yaw_line = plot_pitch.line(vector_yaw, time_frame, color="navy")


# Motor force graph
plot_motor_yrange = Range1d(start=0, end=100)
plot_motor_xrange = Range1d(start=0, end=120)

plot_motor = figure(width=750, plot_height=250, title='Potencia de motores', y_range = plot_motor_yrange, x_range = plot_motor_xrange)
plot_motor.yaxis.axis_label = 'Potencia de motores [%]'
plot_motor.xaxis.axis_label = 'Tiempo [s]'
motor_line_1 = plot_motor.line(vector_motor_1, time_frame, color="blue")
motor_line_2 = plot_motor.line(vector_motor_2, time_frame, color="green")
motor_line_3 = plot_motor.line(vector_motor_3, time_frame, color="red")
motor_line_4 = plot_motor.line(vector_motor_4, time_frame, color="yellow")


# open a session to keep our local document in sync with server
session = push_session(curdoc())

@cosine(w=0.03)
def update(step):
    # updating a single column of the the *same length* is OK
    # Update vectors
    print pitch_line.data_source.data
    vector_roll.append(current_roll)
    vector_pitch.append(current_pitch)
    vector_yaw.append(current_yaw)
    vector_motor_1.append(current_motor_1)
    vector_motor_2.append(current_motor_2)
    vector_motor_3.append(current_motor_3)
    vector_motor_4.append(current_motor_4)
    time_frame.append(time.time() - start_time)

    # Check that the vectors is not too big
    if len(time_frame) >= 5000:
        vector_roll.pop(0)
        vector_pitch.pop(0)
        vector_yaw.pop(0)
        vector_motor_1.pop(0)
        vector_motor_2.pop(0)
        vector_motor_3.pop(0)
        vector_motor_4.pop(0)
        time_frame.pop(0)


    pitch_line.data_source.data = dict(x=vector_pitch,y=time_frame)
    # pitch_line.data_source.data["x"] = time_frame
    roll_line.data_source.data = dict(x=vector_roll,y=time_frame)
    # roll_line.data_source.data["x"] = time_frame
    yaw_line.data_source.data = dict(x=vector_yaw,y=time_frame)
    # yaw_line.data_source.data["x"] = time_frame
    motor_line_1.data_source.data = dict(x=vector_motor_1,y=time_frame)
    # motor_line_1.data_source.data["x"] = time_frame
    motor_line_2.data_source.data = dict(x=vector_motor_2,y=time_frame)
    # motor_line_2.data_source.data["x"] = time_frame
    motor_line_3.data_source.data = dict(x=vector_motor_3,y=time_frame)
    # motor_line_3.data_source.data["x"] = time_frame
    motor_line_4.data_source.data = dict(x=vector_motor_4,y=time_frame)
    # motor_line_4.data_source.data["x"] = time_frame

    # Check the x range
    if time_frame[-1] > 120:
        plot_yaw.x_range(Range1d(start=time_frame[-1] - 120, end=time_frame[-1]))
        plot_roll.x_range(Range1d(start=time_frame[-1] - 120, end=time_frame[-1]))
        plot_pitch.x_range(Range1d(start=time_frame[-1] - 120, end=time_frame[-1]))
        plot_motor.x_range(Range1d(start=time_frame[-1] - 120, end=time_frame[-1]))



session.show(gridplot([[plot_roll,plot_motor],[plot_pitch,None],[plot_yaw,None]])) # open the document in a browser
curdoc().add_periodic_callback(update, 1000)

print "termine la inicializacion"
session.loop_until_closed() # run forever
