#!/usr/bin/env python

from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

def controller_callback(error):
    global sumError_x
    global sumError_y
    command = Twist()
    Kp_x = 1/640
    Ki_x = 1/100000
    Kp_y = 1/360
    Ki_y = 1/100000

    sumError_x = sumError_x + error.x
    sumError_y = sumError_y + error.y
    command.linear.x = Kp_x * error.x + Ki_x * sumError_x
    command.linear.y = Kp_y * error.y + Ki_y * sumError_y
    control_publisher.publish(command)
    # print error
    # print sumError_y, sumError_x
    # print command
    # print "*************************"

if __name__ == "__main__":
    rospy.init_node('controller')
    rospy.Subscriber('error', Point, controller_callback)
    control_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sumError_x = 0
    sumError_y = 0
    rospy.spin()
