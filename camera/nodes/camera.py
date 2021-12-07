#!/usr/bin/env python

import roslib
import sys
import rospy
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class camera_mono(object):
	def __init__(self):
		self.color_sensor_publisher = rospy.Publisher('color_mono', String ,queue_size=1)
		self.camera_subscriber = rospy.Subscriber('raspicam_node/image', Image, self.camera_callback, queue_size = 1)
	
	def camera_callback(self,data):
		bridge = CvBridge()
		try:
			cv_img = bridge.imgmsg_to_cv2(data, "mono8")
	    	except CvBridgeError as e:
			print(e)

		array = cv_img
		#rospy.loginfo(array)

		mid = len(array)//2
		array = array[100:300]
		#print(array.shape)
		array = np.mean(array, axis=0)
		new_array = []
		for i in range(5,len(array)-6):
			new_array.append(np.mean(array[i-5:i+5]))
		#print(array.shape)
		index = np.argmax(new_array)
		self.color_sensor_publisher.publish(str(index))
		print(index)
		#rospy.sleep(0.0001)
		return


def main():
	rospy.init_node('camera_rgb')
	#color_sensor_publisher = rospy.Publisher('color_mono', String ,queue_size=1)
	#camera_subscriber = rospy.Subscriber('raspicam_node/image', Image, camera_callback, queue_size = 1)
	Camera = camera_mono()
	rospy.spin()


if __name__ == '__main__':
	main()
