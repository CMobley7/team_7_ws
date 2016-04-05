#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import cv 
import time
import numpy as np 

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic",Image,queue_size=10)
    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/ardrone/bottom/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    #Actul content code starts from here 
    #The code find object contours on both HSV thresholed mask and gray image
    #since when taking images from an actual camera, finding contours on gray image might not be reliable 
    #convert to HSV color space. 
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    im_shape=cv_image.shape
    mask=np.zeros(im_shape)
    a=hsv[:,:,0]==0
    b=hsv[:,:,1]==0
    c=hsv[:,:,2]==255 
    mask[a & b & c ]=255; 
    mask=np.uint8(mask);
    #convert the image frame to gray image 
    gray= cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(gray,200,255,cv2.THRESH_BINARY)
    #find contour in the gray image
    contours,hierarchy = cv2.findContours(thresh, 1, 2)
    if len(contours)!=0:
    	cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)
    	cnt = contours[0]
    	cv2.drawContours(cv_image, [cnt], 0, (0,0,255), 3)

    #find contour in the mask
    ret,thresh = cv2.threshold(gray,200,255,cv2.THRESH_BINARY)
    contours,hierarchy = cv2.findContours(thresh, 1, 2)    
    if len(contours)!=0:
	cv2.drawContours(mask, contours, -1, (0,255,0), 3)
    	cnt = contours[0]
    	#print('cnt')
    	#print(cnt)
    	cv2.drawContours(mask, [cnt], 0, (0,0,255), 3)
     #minimum bounding box(please don't delet this just yet) 
    	rect = cv2.minAreaRect(cnt)
    	#box = cv2.boxPoints(rect)
    	#box = np.int0(box)
    	#cv2.drawContours(img,[box],0,(0,0,255),2)
     #calculate image moments to find centroids     
    	M = cv2.moments(cnt)
        if M['m00'] !=0:
        	cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		centroid=(cx,cy)
		cv2.circle(mask,centroid,1,(255,0,0),2)		
    
     #size and perimeter can also be found. They should be used when the program is ran on actual images
	#area = cv2.contourArea(cnt)
        #perimeter = cv2.arcLength(cnt,True)
    cv2.imshow("Image window", cv_image)
    cv2.imshow("Mask",mask)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('roboteye', anonymous=True)
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    print('run')
    main(sys.argv)
