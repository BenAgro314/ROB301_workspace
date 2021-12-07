#!/usr/bin/env python
"""
Created on Sun Sep 22 04:56:35 2019

@author: Sherry Chen
"""


import roslib
import sys
import rospy
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import cv2
import numpy as np

def new_line_detector(file_name):
    img = cv2.imread('tape1.jpg')
    print(img)
    # Convert the img to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Apply edge detection method on the image
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    cv2.imwrite('edges.jpg', edges)
    # This returns an array of r and theta values
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 100)

    # The below for loop runs till r and theta values
    # are in the range of the 2d array
    print(lines)
    detected_lines=[]
    for r, theta in lines[0]:
        # Stores the value of cos(theta) in a
        a = np.cos(theta)

        # Stores the value of sin(theta) in b
        b = np.sin(theta)

        # x0 stores the value rcos(theta)
        x0 = a * r

        # y0 stores the value rsin(theta)
        y0 = b * r

        # x1 stores the rounded off value of (rcos(theta)-1000sin(theta))
        x1 = int(x0 + 1000 * (-b))

        # y1 stores the rounded off value of (rsin(theta)+1000cos(theta))
        y1 = int(y0 + 1000 * (a))

        # x2 stores the rounded off value of (rcos(theta)+1000sin(theta))
        x2 = int(x0 - 1000 * (-b))

        # y2 stores the rounded off value of (rsin(theta)-1000cos(theta))
        y2 = int(y0 - 1000 * (a))

        # cv2.line draws a line in img from the point(x1,y1) to (x2,y2).
        # (0,0,255) denotes the colour of the line to be
        # drawn. In this case, it is red.
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
        detected_lines.append(((x1, y1), (x2, y2)))
        # All the changes made in the input image are finally
    # written on a new image houghlines.jpg
    print(detected_lines)
    cv2.imwrite('linesDetected.jpg', img)



class LineDetector:
    def __init__(self):
        self.img_sub = rospy.Subscriber('raspicam_node/image', Image, self.camera_callback, queue_size = 1)
    def camera_callback(self,data):
        rospy.loginfo('callback')
        bridge = CvBridge()
        try:
            cv_img = bridge.imgmsg_to_cv2(data, "mono8")
            #print(cv_img.shape) = (480,640)
            cv_img = cv_img[:400]
        except CvBridgeError as e:
            cv_img=None
            print(e)
            
        try:
            #edges = cv2.Canny(cv_img, 50, 150, apertureSize=3)
            edges=cv2.Canny(cv_img, 100,200)            
            # This returns an array of r and theta values
            lines = cv2.HoughLines(edges, 1, np.pi / 180, 100)
            print(lines)
            # The below for loop runs till r and theta values
            # are in the range of the 2d array
            #rospy.loginfo(lines)
            detected_lines=[]
            if lines is None:
                rospy.loginfo('False')
            else:
                
                rospy.loginfo('True')
            for r, theta in lines[0]:
                # Stores the value of cos(theta) in a
                a = np.cos(theta)
        
                # Stores the value of sin(theta) in b
                b = np.sin(theta)
        
                # x0 stores the value rcos(theta)
                x0 = a * r
        
                # y0 stores the value rsin(theta)
                y0 = b * r
        
                # x1 stores the rounded off value of (rcos(theta)-1000sin(theta))
                x1 = int(x0 + 1000 * (-b))
        
                # y1 stores the rounded off value of (rsin(theta)+1000cos(theta))
                y1 = int(y0 + 1000 * (a))
        
                # x2 stores the rounded off value of (rcos(theta)+1000sin(theta))
                x2 = int(x0 - 1000 * (-b))
        
                # y2 stores the rounded off value of (rsin(theta)-1000cos(theta))
                y2 = int(y0 - 1000 * (a))
        
                # cv2.line draws a line in img from the point(x1,y1) to (x2,y2).
                # (0,0,255) denotes the colour of the line to be
                # drawn. In this case, it is red.
                
                #cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                detected_lines.append(((x1, y1), (x2, y2)))
                # All the changes made in the input image are finally
            # written on a new image houghlines.jpg
            rospy.loginfo(detected_lines)
            #if detected_lines!= []:
            #    rospy.loginfo('True')
            #else:
            #    rospy.loginfo('False')
        except Exception as e:
            print(e)
            

        
        

    


def main():
    #new_line_detector('tape.jpg')
	rospy.init_node('line_detector')
	#color_sensor_publisher = rospy.Publisher('color_mono', String ,queue_size=1)
	#camera_subscriber = rospy.Subscriber('raspicam_node/image', Image, camera_callback, queue_size = 1)
	Line_detector = LineDetector()
	rospy.spin()
    


if __name__ == '__main__':
    main()