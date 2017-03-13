#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from arduino_dynamic_reconfigure_proxy.cfg import TeensyConfig

# Import the float array message
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension




def callback(config, level):
    # rospy.loginfo("""Reconfigure Request: {Motor_Master_Switch}, {motor_1}, {motor_2}, {motor_3}, {motor_4},\
        #   {Controller_ON_OFF}, {Roll_SetPoint}, {Pitch_SetPoint}, {Yaw_SetPoint}, {Pitch_Kp}, {Pitch_Kd}, {Pitch_Ki},\
            # {Roll_Kp}, {Roll_Kd}, {Roll_Ki}, {Yaw_Kp}, {Yaw_Kd}, {Yaw_Ki},""".format(**config))
    # return config
    global pub
    if not rospy.is_shutdown():
        multfloatlayout = MultiArrayLayout([MultiArrayDimension('parameters',18,18)],0)
        multfloat = Float32MultiArray(multfloatlayout, [float(config.Motor_Master_Switch),\
                                                        config.motor_1,\
                                                        config.motor_2,\
                                                        config.motor_3,\
                                                        config.motor_4,\
                                                        float(config.Controller_ON_OFF),\
                                                        config.Roll_SetPoint,\
                                                        config.Pitch_SetPoint,\
                                                        config.Yaw_SetPoint,\
                                                        config.Pitch_Kp,\
                                                        config.Pitch_Kd,\
                                                        config.Pitch_Ki,\
                                                        config.Roll_Kp,\
                                                        config.Roll_Kd,\
                                                        config.Roll_Ki,\
                                                        config.Yaw_Kp,\
                                                        config.Yaw_Kd,\
                                                        config.Yaw_Ki])
        # array_float = [1.0, 2.0, 3.0, 2.0, 1.0, 1.0, 2.0, 3.0, 2.0, 1.0, 1.0, 2.0, 3.0, 2.0, 1.0, 1.0, 2.0, 3.0]
        rospy.loginfo(multfloat)
        pub.publish(multfloat)
    return config

if __name__ == "__main__":

    try:
        pub = rospy.Publisher('teensy_dynamic_reconfigure', Float32MultiArray, queue_size=10)
        rospy.init_node("arduino_dynamic_reconfigure_proxy", anonymous = True)
        srv = Server(TeensyConfig, callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    srv = Server(TeensyConfig, callback)
    rospy.spin()
