#!/usr/bin/env python

""" circle_detector.py - Version 1.0 2016-02-23

    Based on the R. Patrick Goebel's good_feature.py demo code

    Locate HoughCircles feature closest to center of the image to track in a video stream.

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
import cv2
import cv2.cv as cv
import numpy as np
from cv2 import BORDER_DEFAULT
from cv2.cv import CV_HOUGH_GRADIENT
from base_detector import BaseDetector

class LandingZoneDetector(BaseDetector):
    def __init__(self, node_name):
        super(LandingZoneDetector, self).__init__(node_name)

        # Should I show text on the display?
        self.show_text = rospy.get_param("~show_text", True)

        # How big should the feature points be (in pixels)?
        self.feature_size = rospy.get_param("~feature_size", 3)

        # Initialize key variables
        self.feature_point = list()
        self.tracked_point = list()
        self.mask = None

    def process_image(self, cv_image):
        try:
            #Conver Image to HSV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Mask image
            self.mask = np.zeros_like(cv_image)
            h = hsv[:,:,0] == 0
            s = hsv[:,:,1] == 0
            v = hsv[:,:,2] == 255
            self.mask[h & s & v] = 255
            self.mask = self.mask.astype(np.uint8)

            kernel = np.ones((20,20), np.uint8)
            self.mask = cv2.dilate(self.mask, kernel, iterations = 1)

            if 	np.where(self.mask == 255):
                index = np.where(self.mask == 255)
                row_max = np.amax(index[0])
                col_max = np.amax(index[1])
                row_min = np.amin(index[0])
                col_min = np.amin(index[1])

                masked_image = cv2.bitwise_and(cv_image, self.mask)
                masked_image = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
                cv2.imshow('masked_image',masked_image)

                # Get the HoughCircles feature closest to the image's center
                feature_point = self.get_feature_point(masked_image)

                if feature_point is not None and len(feature_point) > 0:
                    # Draw the center of the circle
                    cv2.circle(self.marker_image, (feature_point[0], feature_point[1]), self.feature_size, (0, 0, 255, 0), cv.CV_FILLED, 8, 0)
                    # Draw the outer circle
                    cv2.circle(self.marker_image, (feature_point[0], feature_point[1]), feature_point[2], (0, 255, 0, 0), self.feature_size, 8, 0)

                    # Convert feature_point from image coordinates to world coordinates
                    feature_point = np.array((feature_point[0], feature_point[1]))

                    # Provide self.tracked_point to publish_poi to be published on the /poi topic
                    self.tracked_point = feature_point
        except:
            pass

        return cv_image

    def get_feature_point(self, input_image):
        # Initialize key variables
        feature_points = list()
        feature_point = list()
        frame_height, frame_width = input_image.shape

        # Compute the x, y, and radius of viable circle features
        fp = cv2.HoughCircles(input_image, method=CV_HOUGH_GRADIENT, dp=1, minDist=frame_width/2)

        if fp is not None and len(fp) > 0:
            # Convert fp to a Numpy array
            feature_points = np.float32(fp).reshape(-1, 3)
            feature_point = feature_points[0]

        return feature_point

if __name__ == '__main__':
    try:
        node_name = "landing_zone_detector"
        LandingZoneDetector(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down the Landing Zone Detector node."
        cv.DestroyAllWindows()
