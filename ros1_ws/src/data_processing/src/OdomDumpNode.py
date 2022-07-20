#!/usr/bin/env python3


import glob
import numpy as np
import cv2
import rospy
from data_processing.srv import NoArguments
from nav_msgs.msg import Odometry 
import pandas as pd


class OdomDumpNode():

	def __init__(self)->None:

		odomTopic = rospy.get_param('odomTopic')
		self.dumpFolder = rospy.get_param('dumpFolder')
		self.odom = []
		self.stamp = []

		self.srv = rospy.Service('dump_odom', NoArguments, self.dump_odom_service)

		scan_sub = rospy.Subscriber(odomTopic, Odometry, self.callback)
		self.cnt = 0



	def callback(self, odom_msg):

		p = odom_msg.pose.pose.position
		q = odom_msg.pose.pose.orientation
		odom = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
		#print((odom))
		self.odom.append(odom)
		self.stamp.append(int(odom_msg.header.stamp.to_nsec()))
		#print(self.cnt)
		self.cnt += 1

		
		

	def dump_odom_service(self, request):

		data = {'t' : self.stamp,
				'type' : 'odom',
				'data' : self.odom}
		df = pd.DataFrame(data)
		df.to_pickle(self.dumpFolder + "odom.pickle")

		
			
		print("dumped")

		return "dumped"


if __name__ == "__main__":


    rospy.init_node('OdomDumpNode', anonymous=True)
    posn = OdomDumpNode()
    rospy.spin()

    # df = pd.read_pickle("/home/nickybones/data/MCL/Dump/odom.pickle")
    # print(df.head())