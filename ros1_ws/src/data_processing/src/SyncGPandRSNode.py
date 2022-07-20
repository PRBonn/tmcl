#!/usr/bin/env python3


import glob
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
from message_filters import ApproximateTimeSynchronizer, Subscriber
from data_processing.srv import NoArguments

class SyncGPandRSNode():

	def __init__(self)->None:

		goproTopic = rospy.get_param('goproTopic')
		cameraTopic = rospy.get_param('cameraTopic')
		self.dumpFolder = rospy.get_param('dumpFolder')

		self.bridge = CvBridge()
		self.gopro = []
		self.camera = []
		self.counter = 0

		self.srv = rospy.Service('dump_images', NoArguments, self.dump_images_service)

		gopro_sub = Subscriber(goproTopic, Image)
		camera_sub = Subscriber(cameraTopic, Image)

		self.ats = ApproximateTimeSynchronizer([gopro_sub, camera_sub], queue_size=5, slop=0.1)
		self.ats.registerCallback(self.callback)


	def callback(self, gopro_msg, camera_msg):

		gp = self.bridge.imgmsg_to_cv2(gopro_msg, desired_encoding='passthrough')
		cam = self.bridge.imgmsg_to_cv2(camera_msg, desired_encoding='passthrough')
		self.gopro.append(gp)
		self.camera.append(cam)
		self.counter += 1
		print(self.counter)



	def dump_images_service(self, request):

		for i in range(len(self.gopro)):
				img_name = '{0:04d}'.format(i) + ".png"
				cv2.imwrite(self.dumpFolder + "gopro/" + img_name, self.gopro[i])
				cv2.imwrite(self.dumpFolder + "camera/" + img_name, self.camera[i])

		print("dumped")

		return "dumped"


if __name__ == "__main__":


    rospy.init_node('SyncGPandRSNode', anonymous=True)
    #rate = rospy.Rate(10)
    posn = SyncGPandRSNode()
    rospy.spin()