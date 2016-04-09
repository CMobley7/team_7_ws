#!/usr/bin/env python
from __future__ import absolute_import
import random
import math
import bisect
import numpy as np
# ROS Related Stuff
import rospy
import tf
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo

# Filter
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

def observation_callback(data):
    global firstStep
    # rospy.loginfo(rospy.get_caller_id() + "Object detected at %f, %f, %f ", data.point.x, data.point.y, data.point.z)
    z = np.array([[data.point.x], [data.point.y]])
    # print z
    # print z.shape
    if(firstStep):
        f.x = f.x = np.array([[data.point.x], [data.point.y]])
        firstStep = False
        return


    print "Prior Belief :", f.x
    f.predict()
    # print "Predicted State: "
    # print x_bar
    print "Observation: ", z
    print "Updated State: ", f.x
    f.update(z)
    # observation is available; first correction, then prediction.
    # correction()
    # r = rospy.Rate(10) # 10hz
    # while true: # continue predicting until a new observation becomes avaliable at 10 hz
    #   prediction()
    #   r.sleep()
    # TODO: Do I need to create a sensor likelihood?

def camera_info_callback(data):
    global intrinsicMatrix
    intrinsicMatrix = data.K
    intrinsicMatrix = np.array(intrinsicMatrix).reshape([3,3])

if __name__ == '__main__':
    rospy.init_node('particle_filter', anonymous=True)
    rospy.Subscriber("/poi", PointStamped, observation_callback)
    rospy.Subscriber("/ardrone/bottom/camera_info", CameraInfo, camera_info_callback)
    pub = rospy.Publisher('belief', OccupancyGrid, queue_size=10)
    intrinsicMatrix = None
    firstStep = True

    f = KalmanFilter(dim_x=2, dim_z=2)
    f.x = np.array([[0.], [0.]])       # initial state (location and velocity)

    f.F = np.array([[1.0002,0.],
                    [0.,1.0002]])    # state transition matrix

    f.H = np.array([[1,0.], [0,1]])    # Measurement function
    f.P *= 20.                   # covariance matrix
    f.R = 5                      # state uncertainty
    f.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.03) # process uncertainty

    # belief                    = OccupancyGrid()
    # belief.info.map_load_time = rospy.get_rostime()  # Belief time
    # belief.info.resolution    = 1				     # m/cell
    # belief.info.width         = 100					 # cells
    # belief.info.height        = 60					 # cells
    # belief.data = None
    rospy.spin()
