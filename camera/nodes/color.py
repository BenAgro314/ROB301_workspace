#!/usr/bin/env python

import roslib
import sys
import rospy
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class camera_rgb(object):
    def __init__(self):
        self.color_sensor_publisher = rospy.Publisher('color_rgb', String ,queue_size=1)
        self.camera_subscriber = rospy.Subscriber('raspicam_node/image', Image, self.camera_callback, queue_size = 1)
    
    def camera_callback(self,data):
        bridge = CvBridge()
        try:
            cv_img = bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)
        
        
        array = cv_img[:200]
#        array = cv_img[:350]
          
        #print(array.shape)
        #rospy.loginfo(array)
        intermediate=np.mean(array, axis=0)
        color=np.mean(intermediate, axis=0)
        #print(color.shape)        
        print('r:{}, g:{}, b:{}'.format(color[0], color[1], color[2]))

        
        #self.color_sensor_publisher.publish(str(index))
        #print(index)
        self.color_sensor_publisher.publish('r:{}, g:{}, b:{}'.format(color[0], color[1], color[2]))
        return


def main():
    rospy.init_node('camera_rgb')
    Camera = camera_rgb()
    rospy.spin()


if __name__ == '__main__':
    main()
