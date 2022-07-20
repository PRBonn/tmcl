#!/usr/bin/env python3


import numpy as np
import pandas as pd
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv
from GTpickle import interpolate


def wrapToPi(theta):
    while theta < -np.pi:
        theta = theta + 2 * np.pi
    while theta > np.pi:
        theta = theta - 2 * np.pi
    return theta



class AMCLEval():

	def __init__(self)->None:

		poseTopic = rospy.get_param('~poseTopic')
		self.pose_sub = rospy.Subscriber(poseTopic, PoseWithCovarianceStamped, self.callback, queue_size=100)


		picklePath = rospy.get_param('~picklePath')
		df = pd.read_pickle(picklePath)
		df_sub = df.loc[df['type'] == 'gt']
		gt_t = df_sub['t'].to_numpy()
		print(gt_t.flatten().shape)
		gt_poses = df_sub['data'].to_numpy()
		intr = interpolate(gt_t, gt_poses)
		self.intr = intr

		csvPath = rospy.get_param('~csvPath')
		f = open(csvPath, 'w')
		self.csv = csv.writer(f)
		header = ["t","pose_x","pose_y","pose_yaw","cov_x","cov_y","cov_yaw","gt_x", "gt_y","gt_yaw"]
		self.csv.writerow(header)

	def callback(self, pose_msg):

		t = int(pose_msg.header.stamp.to_nsec())

		pose = pose_msg.pose.pose
		pose_x = pose.position.x
		pose_y = pose.position.y
		qz = pose.orientation.z
		qw = pose.orientation.w
		pose_yaw = wrapToPi(2 * np.arctan(qz/qw))

		cov_x = pose_msg.pose.covariance[0]
		cov_y = pose_msg.pose.covariance[7]
		cov_yaw = pose_msg.pose.covariance[35]

		gt = self.intr(t)
		gt_x = gt[0]
		gt_y = gt[1]
		gt_yaw = gt[2]

		row = [t,pose_x,pose_y,pose_yaw,cov_x,cov_y,cov_yaw,gt_x, gt_y,gt_yaw]
		self.csv.writerow(row)








if __name__ == "__main__":

	rospy.init_node('amcl_eval')
	posn = AMCLEval()
	rospy.spin()
