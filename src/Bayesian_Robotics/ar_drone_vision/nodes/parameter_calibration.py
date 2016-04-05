#!/usr/bin/env python

""" parameter_cablibration.py - Version 1.0 2016-02-23

    Based on the R. Patrick Goebel's cv_bridge_demo.py demo code

    A ROS-to-OpenCV node used to find appropriate parameter values for filters and feature detectors

    Created for the CCAM Project

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""

import rospy
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from cv2 import BORDER_DEFAULT
from cv2.cv import CV_HOUGH_GRADIENT

class parameterCalibration():
    def __init__(self):
        self.node_name = "parameter_calibration"

        rospy.init_node(self.node_name)

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        # Create the OpenCV display window for the RGB image
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        cv.MoveWindow(self.cv_window_name, 25, 75)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Initialize necessary parameters
        self.frame_size = None
        self.frame_width = None
        self.frame_height = None
        self.mask = None
        self.cameraMatrix = np.array([(2529.3016912669586, 0.0, 1007.0532160786125), (0.0, 2524.6309899852313, 650.2969085717225) , (0.0, 0.0, 1.0)])
        self.distCoeffs = np.array([-0.006795069030464255, -0.5045652004390003, 0.004947680741251182, 0.005813011948658337, 0.0])
        self.projectionMatrix = np.array([(2494.93408203125, 0.0, 1015.7040773447079, 0.0), (0.0, 2516.773681640625, 652.354580721294, 0.0), (0.0, 0.0, 1.0, 0.0)])

        # Filter parameters
        self.medianBlur_ksize = rospy.get_param("~medianBlur_ksize", 3)
        self.GaussianBlur_ksize_width = rospy.get_param("~GaussianBlur_ksize_width", 5)
        self.GaussianBlur_ksize_height = rospy.get_param("~GaussianBlur_ksize_height", 5)
        self.GaussianBlur_sigmaX = rospy.get_param("~GaussianBlur_sigmaX", 0)
        self.GaussianBlur_sigmaY = rospy.get_param("~GaussianBlur_sigmaY", 0)
        self.Canny_threshold1 = rospy.get_param("~Canny_threshold1", 90)
        self.Canny_threshold2 = rospy.get_param("~Canny_threshold2", 15)
        self.Canny_apertureSize = rospy.get_param("~Canny_apertureSize", 3)
        self.Canny_L2gradient = rospy.get_param("~L2gradient", False)

        # Store all the feature parameters together for passing to filters
        self.medianBlur_params = dict(ksize = self.medianBlur_ksize)
        self.GaussianBlur_params = dict(ksize = (self.GaussianBlur_ksize_width,self.GaussianBlur_ksize_height),
                                        sigmaX = self.GaussianBlur_sigmaX,
                                        sigmaY = self.GaussianBlur_sigmaY,
                                        borderType = BORDER_DEFAULT)
        self.Canny_params = dict(threshold1 = self.Canny_threshold1,
                                 threshold2 = self.Canny_threshold2,
                                 apertureSize = self.Canny_apertureSize,
                                 L2gradient = self.Canny_L2gradient)

        # Create an image publisher to ouput processed image
        self.image_pub = rospy.Publisher("processed_image", Image)

        # Subscribe to the camera image and depth topics and set the appropriate callbacks
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)

        rospy.loginfo("Waiting for image topics...")
        rospy.wait_for_message("input_rgb_image", Image)
        rospy.loginfo("Ready.")


    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e

        # Convert the image to a numpy array since most cv2 functions
        # require numpy arrays.
        frame = np.array(frame, dtype=np.uint8)

        # Store the frame width and height in a pair of global variables
        if self.frame_width is None:
            self.frame_size = (frame.shape[1], frame.shape[0])
            self.frame_width, self.frame_height = self.frame_size

        # Process the frame using the process_image() function
        display_image = self.process_image(frame)

        # Display the image.
        cv2.imshow(self.node_name, display_image)

        #Use cv_bridge to convert the OpenCV image to ROS formate
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(display_image, "mono8"))
        except CvBridgeError as e:
            print (e)

        # Process any keyboard commands
        self.keystroke = cv2.waitKey(5)
        if self.keystroke != -1:
            cc = chr(self.keystroke & 255).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")

    def process_image(self, frame):
        # # Mask image
        # self.mask = np.zeros_like(frame)
        # x, y, w, h = 3 * self.frame_width / 8, 3 * self.frame_height / 8, self.frame_width / 4, self.frame_height / 4
        # self.mask[y:y+h, x:x+w] = 255
        # masked_image = cv2.bitwise_and(frame, self.mask)

        # Create a grey scale version of the image
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Equalize the histogram to reduce lighting effects
        grey = cv2.equalizeHist(grey)

        # Remove salt-and-pepper, and white noise by using a combination of a median and Gaussian filter
        grey = cv2.medianBlur(grey, **self.medianBlur_params)
        grey = cv2.GaussianBlur(grey, **self.GaussianBlur_params)

        # Compute edges using the Canny edge filter
        edges = cv2.Canny(grey, **self.Canny_params)

        return edges

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

def main(args):
    try:
        parameterCalibration()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
