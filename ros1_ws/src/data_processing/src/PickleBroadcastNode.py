#!/usr/bin/env python3


import glob
import numpy as np
import cv2
import rospy
from data_processing.srv import NoArguments
import pandas as pd
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
from geometry_msgs.msg import PoseStamped
import tf
from sensor_msgs.msg import PointCloud2, PointField
import ros_numpy 
from ros_numpy import numpy_msg



def wrapToPi(theta):
    while theta < -np.pi:
        theta = theta + 2 * np.pi
    while theta > np.pi:
        theta = theta - 2 * np.pi
    return theta


def arrToPclMSG(npoints, occluded_points, t):

	points_arr = np.zeros((npoints,), dtype=[
	('x', np.float32),
	('y', np.float32),
	('z', np.float32),
	('r', np.uint8),
	('g', np.uint8),
	('b', np.uint8)])

	for i in range(npoints):
		points_arr['x'][i] = occluded_points[i][0]
		points_arr['y'][i] = occluded_points[i][1]

	points_arr['z'] = 0
	points_arr['r'] = 0
	points_arr['g'] = 255
	points_arr['b'] = 0

	cloud_msg = ros_numpy.msgify(numpy_msg(PointCloud2), points_arr)
	cloud_msg.header.stamp = rospy.Time(t)
	cloud_msg.header.frame_id = "world"
	
	return cloud_msg


class PickleBroadcastNode():

	def __init__(self, pickleFolder)->None:


		df = pd.read_pickle(pickleFolder + "gtmerged.pickle")

		cam0pub = rospy.Publisher('/pickle/camera0', Image, queue_size=10)
		cam1pub = rospy.Publisher('/pickle/camera1', Image, queue_size=10)
		cam2pub = rospy.Publisher('/pickle/camera2', Image, queue_size=10)
		cam3pub = rospy.Publisher('/pickle/camera3', Image, queue_size=10)
		gtPub = rospy.Publisher('/pickle/gt', PoseStamped, queue_size=10)
		occlidarPub = rospy.Publisher('/pickle/occlidar', PointCloud2, queue_size=10)
		lidarPub = rospy.Publisher('/pickle/lidar', PointCloud2, queue_size=10)

		bridge = CvBridge()

		cnt = 0

		rate = rospy.Rate(200)

		lidar = None
		k_0 = np.array([[380.07025146484375, 0.0, 324.4729919433594], [0.0, 379.66119384765625, 237.78517150878906], [0.0, 0.0, 1.0]])
		k_inv0 = np.linalg.inv(k_0)
		o_T = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])



		for index, row in df.iterrows():

			# if "camera" in row['type']:
			# 	cam = row['type']
			# 	imgpath = pickleFolder + cam + "/" + row['data']
			# 	img = cv2.imread(imgpath)
			# 	img_msg = bridge.cv2_to_imgmsg(img,  encoding="rgb8")
			# 	t = row['t'] / 10**9
			# 	img_msg.header.stamp = rospy.Time(t)
			# 	if cam == "camera0":
			# 		cam0pub.publish(img_msg)
			# 	elif cam == "camera1":
			# 		cam1pub.publish(img_msg)
			# 	elif cam == "camera2":
			# 		cam2pub.publish(img_msg)
			# 	elif cam == "camera3":
			# 		cam3pub.publish(img_msg)

			if row['type'] == 'gt':
				pose_msg = PoseStamped()
				data = row['data']
				pose_msg.pose.position.x = data[0]
				pose_msg.pose.position.y = data[1]
				pose_msg.pose.position.z = 1

				quaternion = tf.transformations.quaternion_from_euler(0, 0, data[2] + np.pi *0.5)
				pose_msg.pose.orientation.x = quaternion[0]
				pose_msg.pose.orientation.y = quaternion[1]
				pose_msg.pose.orientation.z = quaternion[2]
				pose_msg.pose.orientation.w = quaternion[3]
				t = row['t'] / 10**9
				pose_msg.header.stamp = rospy.Time(t)
				pose_msg.header.frame_id = "map"
				gtPub.publish(pose_msg)
				cnt += 1
				#print(t)
				rate.sleep()

			# elif row['type'] == 'lidar':

			# 	lidar = row['data']
				
			# elif row['type'] == 'sem0':

			# 	if lidar is None:
			# 		continue

			# 	data = np.asarray(row['data'])
			# 	boxes = data.reshape((-1, 4))
			# 	#print(boxes)
			# 	occluded_angles = []
			# 	for b in boxes:
			# 		p1 = np.array([[b[0]], [b[3]], [1]])
			# 		p2 = np.array([[b[2]], [b[3]], [1]])
			# 		p1_k = k_inv0 @ p1
			# 		p2_k = k_inv0 @ p2
			# 		p1_k = p1_k /p1_k[2]
			# 		p2_k = p2_k /p2_k[2]
			# 		p1_c = o_T @ p1_k
			# 		p2_c = o_T @ p2_k
			# 		t1 = wrapToPi(np.arctan2(p1_c[1], p1_c[0]))
			# 		t2 = wrapToPi(np.arctan2(p2_c[1], p2_c[0]))
			# 		occluded_angles.append([t1, t2])
				
			# 	npoints = int(len(lidar) / 2)
			# 	occluded_points = []
			# 	seen_points = []

			# 	for i in range(npoints):

			# 		p = np.array([lidar[2 * i], lidar[2 * i + 1], 1])
			# 		theta = wrapToPi(np.arctan2(p[1], p[0]))

			# 		for j in range(len(occluded_angles)):
			# 			t1, t2 = occluded_angles[j]
			# 			#print(t1, t2, theta)
			# 			if (theta < t1) and (theta > t2):
			# 				occluded_points.append(p)
			# 			else:
			# 				seen_points.append(p)

			# 	t = row['t'] / 10**9
			# 	cloud_msg = arrToPclMSG(len(occluded_points), occluded_points, t)
			# 	occlidarPub.publish(cloud_msg)
			# 	cloud_msg = arrToPclMSG(len(seen_points), seen_points, t)
			# 	lidarPub.publish(cloud_msg)












		


if __name__ == "__main__":


    rospy.init_node('PickleBroadcastNode', anonymous=True)
    pickleFolder = "/home/nickybones/data/MCL/2021_12_07/Run1/"
    posn = PickleBroadcastNode(pickleFolder)
    rospy.spin()

  