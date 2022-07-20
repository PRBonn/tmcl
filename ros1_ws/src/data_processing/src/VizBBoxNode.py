
import numpy as np
import cv2
import pandas as pd
import rospy # Python library for ROS
from cv_bridge import CvBridge 
from sensor_msgs.msg import PointCloud2, PointField, LaserScan, Image, CameraInfo
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import struct
from sensor_msgs import point_cloud2
import glob
import time




def wrapToPi(theta):
    while theta < -np.pi:
        theta = theta + 2 * np.pi
    while theta > np.pi:
        theta = theta - 2 * np.pi
    return theta


class VizBBoxNode():

	def __init__(self, picklePath)->None:

		# df = pd.read_csv(picklePath + "scan_0009458.csv")
		# x = df['x']
		# y = df['y']
		# mask = df['mask']

		filepaths = glob.glob(picklePath + "*.csv")
		filepaths = [ x for x in filepaths if "scan" in x ]
		filepaths = sorted(filepaths)

		self.pc2pub = rospy.Publisher('HumanMask', PointCloud2, queue_size=1000)
		
		for f in filepaths:
			df = pd.read_csv(f)
			
			x = df['x'].to_numpy()
			y = df['y'].to_numpy()
			mask = df['mask'].to_numpy()
			mask = 1.0 - mask
			self.broadcastCloud(x, y, mask)
			time.sleep(0.2)
			print(f)
		

		# df = pd.read_pickle(picklePath)

		# k_ = np.array([[380.07025146484375, 0.0, 324.4729919433594], [0.0, 379.66119384765625, 237.78517150878906], [0.0, 0.0, 1.0]])
		# k_ = np.reshape(k_, (3,3))
		# self.k_inv = np.linalg.inv(k_)


		# # go from image frame to camera frame
		# self.tc = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
		# self.camAngles = [0, -np.pi/2, np.pi, np.pi/2]

		# self.pc2pub = rospy.Publisher('HumanMask', PointCloud2, queue_size=1000)

		# self.occludingAngles = []
		# for i in range(4):
		# 	self.occludingAngles.append([])

		# rate = rospy.Rate(1) 
		# lidar = None
		# sem0 = None
		# sem1 = None
		# sem2 = None
		# sem3 = None

		# for index, row in df.iterrows():

		# 	if row['type'] == 'lidar':
		# 		lidar = row['data']
		# 		lidar = np.array(lidar)
		# 		lidar = lidar.reshape((-1, 2)).T
		# 		self.VizBBox(lidar)

		# 	elif row['type'] == 'sem0':
		# 		sem0 = row['data']
		# 		sem0 = np.array(sem0)
		# 		sem0 = sem0.reshape((-1, 4))
		# 		#print(sem0.shape)

		# 		self.ComputeOccludingAngles(sem0, 0)

		# 	elif row['type'] == 'sem1':
		# 		sem1 = row['data']
		# 		sem1 = np.array(sem1)
		# 		sem1 = sem1.reshape((-1, 4))
		# 		#print(sem0.shape)

		# 		self.ComputeOccludingAngles(sem1, 1)

		# 	elif row['type'] == 'sem2':
		# 		sem2 = row['data']
		# 		sem2 = np.array(sem2)
		# 		sem2 = sem2.reshape((-1, 4))
		# 		#print(sem0.shape)

		# 		self.ComputeOccludingAngles(sem2, 2)

		# 	elif row['type'] == 'sem3':
		# 		sem3 = row['data']
		# 		sem3 = np.array(sem3)
		# 		sem3 = sem3.reshape((-1, 4))
		# 		#print(sem0.shape)

		# 		self.ComputeOccludingAngles(sem3, 3)

			#rate.sleep()


	def uv2cameraFrame(self, u1, v1, u2, v2):

		# create homogenous vector
		p1 = np.array([u1, v1, 1]).T
		p2 = np.array([u2, v2, 1]).T

		# multiply by inverse calibration matrix
		p1k_ = self.k_inv @ p1
		p2k_ = self.k_inv @ p2 

		# divide by z component to homogenize it
		p1k = p1k_ / p1k_[2]
		p2k = p2k_ / p2k_[2]

		# go from image frame to camera frame
		p1c = self.tc @ p1k
		p2c = self.tc @ p2k

		return p1c, p2c


	def createPointCloud(self, occluded, notoccluded):

		points3d = []
		n = occluded.shape[1]
		m = notoccluded.shape[1]

		for i in range(n):
		    p = occluded[:, i]
		    x = p[0]
		    y = p[1]
		    z = 0
		    r = 255
		    g = 0
		    b = 0
		    a = 255
		    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
		    pt = [x, y, z, rgb]
		    points3d.append(pt)

		for i in range(m):
		    p = notoccluded[:, i]
		    x = p[0]
		    y = p[1]
		    z = 0
		    r = 0
		    g = 255
		    b = 0
		    a = 255
		    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
		    pt = [x, y, z, rgb]
		    points3d.append(pt)


		points3d.append([0, 0, 1, struct.unpack('I', struct.pack('BBBB', 255, 255, 255, 255))[0]])

		fields = [PointField('x', 0, PointField.FLOAT32, 1),
		          PointField('y', 4, PointField.FLOAT32, 1),
		          PointField('z', 8, PointField.FLOAT32, 1),
		          # PointField('rgb', 12, PointField.UINT32, 1),
		          PointField('rgba', 12, PointField.UINT32, 1),
		          ]


		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = "map"
		# h = header
		pc2 = point_cloud2.create_cloud(header, fields, points3d)

		return pc2


	def ComputeOccludingAngles(self, pred_boxes, camID):

		self.occludingAngles[camID].clear()
		n = len(pred_boxes)

		for i in range(n):

		    pred_box = pred_boxes[i]

		    # coords of the bottom corners of the bounding box
		    u1 = pred_box[0] # left side
		    u2 = pred_box[2] # right side
		    v1 = pred_box[3]
		    v2 = pred_box[3]

		    p1c, p2c = self.uv2cameraFrame(u1, v1, u2, v2)


		    t1 = wrapToPi(np.arctan2(p1c[1], p1c[0]) + self.camAngles[camID])
		    t2 = wrapToPi(np.arctan2(p2c[1], p2c[0]) + self.camAngles[camID])
 
		    self.occludingAngles[camID].append([t1, t2])


	def VizBBox(self, points_3d):

		scanNum = points_3d.shape[1]
		occludedPoints = np.zeros(scanNum, dtype=bool)

		for j in range(scanNum):

			p_3d = points_3d[:, j]
			theta = wrapToPi(np.arctan2(p_3d[1], p_3d[0]))

			for c in range(4):

				c_occAngles = self.occludingAngles[c]
				for a in range(len(c_occAngles)):
					t1, t2 = c_occAngles[a]
					if (theta < t1) and (theta > t2):
						occludedPoints[j] = True

		occluded = points_3d[:, occludedPoints]
		notoccluded = points_3d[:, np.logical_not(occludedPoints)]

		pc2 = self.createPointCloud(occluded, notoccluded)
		self.pc2pub.publish(pc2)


	def broadcastCloud(self, x, y, mask):

		scanNum = len(x)
		points_3d = np.zeros((scanNum, 3), dtype=float)

		for i in range(scanNum):
			points_3d[i, 0] = x[i]
			points_3d[i, 1] = y[i]
			points_3d[i, 2] = 1

		#print(points_3d)
		occludedPoints = mask.astype('bool')
		occluded = points_3d[occludedPoints].T
		notoccluded = points_3d[np.logical_not(occludedPoints)].T

		pc2 = self.createPointCloud(occluded, notoccluded)
		self.pc2pub.publish(pc2)




	# def VizBBox(self, points_3d, pred_boxes, camID):

	# 	scanNum = points_3d.shape[1]
	# 	occludingAngles[camID].clear()
	# 	occludedPoints = np.zeros(scanNum, dtype=bool)

	# 	#print(pred_class_names)

	# 	n = len(pred_boxes)


	# 	for i in range(n):

		   
	# 	    pred_box = pred_boxes[i]

	# 	    # coords of the bottom corners of the bounding box
	# 	    u1 = pred_box[0] # left side
	# 	    u2 = pred_box[2] # right side
	# 	    v1 = pred_box[3]
	# 	    v2 = pred_box[3]

	# 	    p1c, p2c = self.uv2cameraFrame(u1, v1, u2, v2)


	# 	    t1 = wrapToPi(np.arctan2(p1c[1], p1c[0]))
	# 	    t2 = wrapToPi(np.arctan2(p2c[1], p2c[0]))
	# 	    #print(t1, t2)

	# 	    for j in range(scanNum):
	# 	        p_3d = points_3d[:, j]
	# 	        theta = wrapToPi(np.arctan2(p_3d[1], p_3d[0]))
	# 	        #print(theta)
	# 	        if (theta < t1) and (theta > t2):
	# 	            occludedPoints[j] = True

	# 	    occludingAngles.append([t1, t2])


	# 	occluded = points_3d[:, occludedPoints]
	# 	notoccluded = points_3d[:, np.logical_not(occludedPoints)]

	# 	pc2 = self.createPointCloud(occluded, notoccluded)
	# 	self.pc2pub.publish(pc2)


if __name__ == "__main__":


	#picklePath = "/home/nickybones/data/MCL/2021_12_07/Run4/textgtmerged.pickle"
	picklePath = "/home/nickybones/data/MCL/2021_12_08/Run2/results/"

	rospy.init_node('VizBBoxNode', anonymous=True)

	viz = VizBBoxNode(picklePath)