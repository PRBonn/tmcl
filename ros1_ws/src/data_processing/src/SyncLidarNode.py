#!/usr/bin/env python3


import glob
import numpy as np
import cv2
import rospy
from data_processing.srv import NoArguments
import pandas as pd
from nmcl_msgs.msg import MergedLaserScan


class SyncLidarNode():

	def __init__(self)->None:

		mergedScanTopic = rospy.get_param('mergedScanTopic')
		self.dumpFolder = rospy.get_param('dumpFolder')
		self.xy = []
		self.stamp = []

		self.srv = rospy.Service('dump_scans', NoArguments, self.dump_scans_service)

		scan_sub = rospy.Subscriber(mergedScanTopic, MergedLaserScan, self.callback)
		self.cnt = 0



	def callback(self, merged_msg):

		self.xy.append(merged_msg.xy)
		self.stamp.append(int(merged_msg.header.stamp.to_nsec()))
		#print(self.cnt)
		self.cnt += 1

		
		

	def dump_scans_service(self, request):

		data = {'t' : self.stamp,
				'type' : 'lidar',
				'data' : self.xy}
		df = pd.DataFrame(data)
		df.to_pickle(self.dumpFolder + "scans.pickle")

		
			
		print("dumped")

		return "dumped"


if __name__ == "__main__":


    rospy.init_node('SyncLidarNode', anonymous=True)
    posn = SyncLidarNode()
    rospy.spin()

    # df = pd.read_pickle("/home/nickybones/data/MCL/Dump/scans.pickle")
    # print(df.head())