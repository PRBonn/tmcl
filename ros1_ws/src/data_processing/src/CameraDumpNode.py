#!/usr/bin/env python3


import glob
import numpy as np
import cv2
import rospy
from data_processing.srv import NoArguments
import pandas as pd
from sensor_msgs.msg import Image
import os
from cv_bridge import CvBridge 

class CameraDumpNode():

	def __init__(self)->None:

		cameraTopic = rospy.get_param('~cameraTopic')
		self.dumpFolder = rospy.get_param('dumpFolder')
		self.camName = rospy.get_param('~camName')
		print(self.camName)
		self.path = []
		self.stamp = []
		self.img = []
		self.bridge = CvBridge()

		if not os.path.exists(self.dumpFolder + self.camName):
			os.makedirs(self.dumpFolder + self.camName)

		self.srv = rospy.Service(self.camName + '/dump_images', NoArguments, self.dump_images_service)

		cam_sub = rospy.Subscriber(cameraTopic, Image, self.callback, queue_size=1000)
		self.cnt = 0



	def callback(self,img_msg):

		img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
		#self.img.append(img)

		img_name = '{0:05d}'.format(self.cnt) + ".png"
		self.path.append(img_name)
		self.stamp.append(int(img_msg.header.stamp.to_nsec()))
		img_name = self.dumpFolder + self.camName + "/" + img_name
		cv2.imwrite(img_name, img)

		print(self.cnt)
		self.cnt += 1

		
		
	def dump_images_service(self, request):


		# for i in range(len(self.path)):
		# 	img_name = self.dumpFolder + self.camName + "/" + self.path[i]
		# 	cv2.imwrite(img_name, self.img[i])

		data = {'t' : self.stamp,
				'type' : self.camName,
				'data' : self.path}
		df = pd.DataFrame(data)
		df.to_pickle(self.dumpFolder + "/" + self.camName + ".pickle")

		
			
		print("dumped")

		return "dumped"


if __name__ == "__main__":


    rospy.init_node('image')
    posn = CameraDumpNode()
    rospy.spin()

    # df = pd.read_pickle("/home/nickybones/data/MCL/Dump/scans.pickle")
    # print(df.head())