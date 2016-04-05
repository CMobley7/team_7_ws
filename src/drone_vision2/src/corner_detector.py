#!/usr/bin/env python

""" corner_detector.py - Version 1.0 2016-02-23

    Based on the R. Patrick Goebel's good_feature.py demo code

    Locate good features closest to center of the image to track in a video stream.

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
from base_detector import BaseDetector

class CornerDetector(BaseDetector):
    def __init__(self, node_name):
        super(CornerDetector, self).__init__(node_name)

        # Should I show text on the display?
        self.show_text = rospy.get_param("~show_text", True)

        # How big should the feature points be (in pixels)?
        self.feature_size = rospy.get_param("~feature_size", 1)

        # Filter parameters
        self.medianBlur_ksize = rospy.get_param("~medianBlur_ksize", 9)
        self.GaussianBlur_ksize_width = rospy.get_param("~GaussianBlur_ksize_width", 15)
        self.GaussianBlur_ksize_height = rospy.get_param("~GaussianBlur_ksize_height", 15)
        self.GaussianBlur_sigmaX = rospy.get_param("~GaussianBlur_sigmaX", 0)
        self.GaussianBlur_sigmaY = rospy.get_param("~GaussianBlur_sigmaY", 0)

        # Store all the feature parameters together for passing to filters
        self.medianBlur_params = dict(ksize = self.medianBlur_ksize)
        self.GaussianBlur_params = dict(ksize = (self.GaussianBlur_ksize_width,self.GaussianBlur_ksize_height),
                                        sigmaX = self.GaussianBlur_sigmaX,
                                        sigmaY = self.GaussianBlur_sigmaY,
                                        borderType = BORDER_DEFAULT)
        # GoodFeaturesToTrack parameters
        self.goodFeaturesToTrack_maxCorners = rospy.get_param("~goodFeaturesToTrack_maxCorners", 15)
        self.goodFeaturesToTrack_qualityLevel = rospy.get_param("~goodFeaturesToTrack_qualityLevel", 0.05)
        self.goodFeaturesToTrack_minDistance = rospy.get_param("~goodFeaturesToTrack_minDistance", 70)
        self.goodFeaturesToTrack_blockSize = rospy.get_param("~goodFeaturesToTrack_blockSize", 15)
        self.goodFeaturesToTrack_useHarrisDetector = rospy.get_param("~goodFeaturesToTrack_useHarrisDetector", True)
        self.goodFeaturesToTrack_k = rospy.get_param("~goodFeaturesToTrack_k", 0.05)

        # Store all parameters together for passing to the goodFeaturesToTrack detector
        self.goodFeaturesToTrack_params = dict(maxCorners = self.goodFeaturesToTrack_maxCorners,
                       qualityLevel = self.goodFeaturesToTrack_qualityLevel,
                       minDistance = self.goodFeaturesToTrack_minDistance,
                       blockSize = self.goodFeaturesToTrack_blockSize,
                       useHarrisDetector = self.goodFeaturesToTrack_useHarrisDetector,
                       k = self.goodFeaturesToTrack_k)

        # Initialize key variables
        self.feature_point = list()
        self.tracked_point = list()
        self.mask = None

    def process_image(self, cv_image):
        try:
            # Mask image
            self.mask = np.zeros_like(cv_image)
            x, y, w, h = self.frame_width / 4, self.frame_height / 4, self.frame_width / 2, self.frame_height / 2
            self.mask[y:y+h, x:x+w] = 255
            masked_image = cv2.bitwise_and(cv_image, self.mask)

            # Create a grey scale version of the image
            grey = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)

            # Equalize the histogram to reduce lighting effects
            grey = cv2.equalizeHist(grey)

            # Remove salt-and-pepper, and white noise by using a combination of a median and Gaussian filter
            grey = cv2.medianBlur(grey, **self.medianBlur_params)
            grey = cv2.GaussianBlur(grey, **self.GaussianBlur_params)

            # Get the HoughCircles feature closest to the image's center
            feature_point = self.get_feature_point(grey)

            if feature_point is not None and len(feature_point) > 0:
                # Draw feature point
                cv2.circle(self.marker_image, (feature_point[0], feature_point[1]), self.feature_size, (0, 0, 255, 0), cv.CV_FILLED, 8, 0)

                # Convert feature_point from image coordinates to world coordinates
                feature_point = np.dot(np.linalg.pinv(self.projectionMatrix), np.array([feature_point[0],feature_point[1], 1]))
                feature_point = feature_point / feature_point[2]
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

        # Compute the x, y, and radius of viable circle features
        fp = cv2.goodFeaturesToTrack(input_image, **self.goodFeaturesToTrack_params)
        if fp is not None and len(fp) > 0:
            # Convert fp to a Numpy array
            feature_points = np.float32(fp).reshape(-1, 2)

            # Compute the strong corner closest to the camera's center
            distances = np.sqrt((feature_points[:,0]-self.frame_width/2)**2 + (feature_points[:,1]-self.frame_height/2)**2)
            sorted_distances_indicies = np.argsort(distances)
            sorted_feature_points = feature_points[sorted_distances_indicies]
            feature_point = sorted_feature_points[0]

        return feature_point

if __name__ == '__main__':
    try:
        node_name = "corner_detector"
        CornerDetector(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down the Corner Detector node."
        cv.DestroyAllWindows()
