#!/usr/bin/env python
# Script that listen to a Geometry Pose Message, transforms it into a Euler rotation, ans publishes it as a
# Vector3 message. with x = ROLL, y = PITCH and z = YaW.
# Reference = http://answers.ros.org/question/69754/quaternion-transformations-in-python/
import rospy
import tf
import math

# Import the float array message
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3

# Function callback used when a quaternion is received
def callback(data):
    global pub
    if not rospy.is_shutdown():
        rospy.loginfo(data)
        # Use TF to convert the quaternion to euler, the publish it
        euler_msg = Vector3();
        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        euler_msg.x = euler[0]*180/math.pi
        euler_msg.y = euler[1]*180/math.pi
        euler_msg.z = euler[2]*180/math.pi
        # publish the euler vetor as ROLL, PITH, YAW.
        pub.publish(euler_msg)


if __name__ == "__main__":

    try:
        pub = rospy.Publisher('teensy_euler', Vector3, queue_size=10)
        rospy.init_node("quaternion2euler", anonymous = True)
        rospy.Subscriber("quadcopter_pose", Pose, callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
