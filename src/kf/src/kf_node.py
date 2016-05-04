#!/usr/bin/env python
from __future__ import absolute_import
import random
import math
import bisect
import numpy as np
# ROS Related Stuff
import rospy
import tf
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# Filter
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

def observation_callback(data):
    global observation
    print "hi"
    observation = data

def img_callback(img):
    global frame
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(img, "bgr8")
    frame = np.array(frame, dtype=np.uint8)



if __name__ == '__main__':
    rospy.init_node('particle_filter', anonymous=True)
    rospy.Subscriber("/poi", PointStamped, observation_callback)
    rospy.Subscriber("/ardrone/bottom/image_raw", Image, img_callback, queue_size=5)
    error_pub = rospy.Publisher("/error", PointStamped, queue_size=5)

    frame = None
    observation = None

    f = KalmanFilter(dim_x=2, dim_z=2)
    f.x = np.array([[0.], [0.]])       # initial state (location and velocity)

    f.F = np.array([[1.0002,0.],
                    [0.,1.0002]])    # state transition matrix

    f.P = np.array([[10.0,   0.0],
                    [0.0, 10.0]])

    f.R = np.array([[0.0001, 0.0],
                    [0.0, 0.0001]])

    f.H = np.array([[1.0, 0.0],
                    [0.0, 1.0]])
    f.Q = Q_discrete_white_noise(dim = 2, dt = 100.0, var = 5000.0)

    rate = rospy.Rate(100) # 10hz
    cv2.namedWindow("Kalman Filter Output",1)

    while not rospy.is_shutdown():
        print "Prior Belief :", f.x
        f.predict()
        x_bar, covar = f.get_prediction()
        print "Prediction :", x_bar, covar

        if observation != None:
            z = np.array([[observation.point.x], [observation.point.y]])
		    # print "Predicted State: "
		    # print x_bar
            print "Observation: ", z
            f.update(z)
            print "Updated State: ", f.x

        if frame != None:
            cv2.circle(frame, (int(f.x[0][0]), int(f.x[1][0])), int(20), (0, 255, 0), 3, 8, 0)
            if observation != None:
                cv2.circle(frame, (int(observation.point.x), int(observation.point.y)), int(observation.point.z), (0, 0, 255), 3, 8, 0)
            cv2.imshow("Kalman Filter Output", frame)
            cv2.waitKey(20)

        frame_width = 640
        frame_height = 360

        error_x = f.x[0][0] - frame_width/2
        error_y = f.x[1][0] - frame_height/2

        Error = PointStamped()
        Error.header.frame_id = 'error_frame'
        Error.header.stamp = rospy.Time.now()
        Error.point.x = error_x
        Error.point.y = error_y
        Error.point.z = 0.0
        error_pub.publish(Error)
        rate.sleep()


    # rospy.spin()
