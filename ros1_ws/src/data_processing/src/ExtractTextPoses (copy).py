import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd


textPoses = {
	"Room 1" : [7.8, -2.65, 0.00930304 + np.pi],
	"Room 2" : [3.95, -2.68, 0.00930304 + np.pi],
	"Room 3" : [-5.3, -1.05, 0.00930304 - np.pi * 0.5],
	"Room 4" : [-6.02, -2.72, 0.00930304 + np.pi],
	"Room 5" : [-8.49, -0.37, 0.00930304 + np.pi * 0.5],
	"Room 6" : [-11.488, -4.97, 0.00930304],
	"Room 7" : [-7.04, -4.95, 0.00930304],
	"Room 8" : [-3.77, -4.93, 0.00930304],
	"Room 9" : [0.72, -4.89, 0.00930304],
	"Room 10" : [9.53, -4.799, 0.00930304]
	}


def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def rotationMatrixToEulerAngles(R) :
    assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])



def computeHist(poses, weights):

	xmin = np.min(poses[:, 0])
	xmax = np.max(poses[:, 0])
	ymin = np.min(poses[:, 1])
	ymax = np.max(poses[:, 1])

	xedges = np.arange(xmin, xmax, step=0.01)
	yedges = np.arange(ymin, ymax, step=0.01)
	bins = (xedges, yedges)

	poses = poses.reshape(-1, 3)
	Hall, xedges, yedges = np.histogram2d(poses[:, 0].squeeze(), poses[:, 1].squeeze(), bins=(xedges, yedges))
	weights = np.asarray(weights).reshape(-1, 1)
	weighted_poses = poses * weights
	Hmatch, xedges, yedges = np.histogram2d(weighted_poses[:, 0].squeeze(), weighted_poses[:, 1].squeeze(), bins=(xedges, yedges))
	Hnorm = Hmatch / Hall

	return Hnorm



def vizPoses(poses):

	xmin = np.min(poses[:, 0])
	xmax = np.max(poses[:, 0])
	ymin = np.min(poses[:, 1])
	ymax = np.max(poses[:, 1])

	ax = plt.gca()
	ax.set_xlim([xmin, xmax])
	ax.set_ylim([0, ymax])

	plt.scatter(poses[:, 0], poses[:, 1], alpha=0.2)
	plt.show()


def VizHeatMapandPose(poses, H):

	xmin = np.min(poses[:, 0])
	xmax = np.max(poses[:, 0])
	ymin = np.min(poses[:, 1])
	ymax = np.max(poses[:, 1])

	plt.figure(figsize=(20, 12), dpi=80)
	
	fig1 = plt.subplot(121)
	fig1.title.set_text('Robot Poses')
	ax = plt.gca()
	ax.axis('equal')
	ax.set_xlim([xmin, xmax])
	ax.set_ylim([ymin, ymax])
	plt.scatter(poses[:, 0], poses[:, 1], alpha=0.2)

	fig2 = plt.subplot(122)
	fig2.title.set_text('Detection Likelihood')
	H = H.T
	im = plt.imshow(H, origin='lower')
	cbar = fig2.figure.colorbar(im, ax=fig2)
	
	plt.show()
	

def getRelPoses(posefilename, roomname):

	posefile = open(posefilename, 'r')
	lines = posefile.readlines()
	robotPoses = []
	relativePoses = []
	ids = []

	roompose = textPoses[roomname]
	#roomTrans = np.array([[1, 0, roompose[0]], [0, 1, roompose[1]], [0, 0, 1]])
	roomTrans = np.array([[np.cos(roompose[2]), -np.sin(roompose[2]), roompose[0]], [np.sin(roompose[2]), np.cos(roompose[2]), roompose[1]], [0, 0, 1]])
	roomTrans = np.linalg.inv(roomTrans)

	 
	count = 0
	for l in lines:
		comps = l.split()
		poseid = int(comps[0])
		strmtx = np.asarray(comps[4:])
		mtx = strmtx.reshape((4,4)).astype(float)
		R = mtx[:3, :3]
		yaw, pitch, roll = rotationMatrixToEulerAngles(R)
		x = mtx[0][3]
		y = mtx[1][3]
		robotPoses.append(np.array([[x], [y], yaw]))

		relpose = np.array([[x], [y], [1]])
		relpose = roomTrans @ relpose
		relpose[2] = yaw - roompose[2]
		relativePoses.append(relpose)
		ids.append(poseid)


	relativePoses = np.asarray(relativePoses)
	robotPoses = np.asarray(robotPoses)
	
	return relativePoses, robotPoses, ids

def performance(predfilename, roomname):

	matches = []

	roomstrs = roomname.split(' ')
	roomset = set(roomstrs)
	setsize = len(roomset)
	mergedname = roomname.replace(" ", "")


	predfile = open(predfilename, 'r')
	lines = predfile.readlines()
	for l in lines:
		pred = l.split(',')
		predset = set(pred)

		flag = 0
		if (predset.intersection(roomset) == setsize):
			flag = 1
		elif mergedname in predset:
			flag = 1


		matches.append(flag)

	#print(len(matches))
	matches = np.asarray(matches)

	return matches


def findWallAngle():

	points = np.zeros((4,2))
	points[0] = textPoses['Room 6']
	points[1] = textPoses['Room 7']
	points[2] = textPoses['Room 8']
	points[3] = textPoses['Room 10']

	vx,vy,x,y = cv2.fitLine(np.float32(points), cv2.DIST_L2,0,0.01,0.01)

	x_axis      = np.array([1, 0])   
	text_line   = np.array([vx, vy])  # unit vector in the same direction as your line
	dot_product = np.dot(x_axis, text_line)
	angle_2_x   = np.arccos(dot_product)

	return angle_2_x

def loadPickle(path):

	df = pd.read_pickle(path)

	rel_x = df['rel_x']
	rel_y = df['rel_y']
	matches = df['match']
	print(rel_x.shape)

	relativePoses = np.concatenate((rel_x, rel_y), axis=0)
	print(relativePoses.shape)
	H = computeHist(relativePoses, matches)
	VizHeatMapandPose(relativePoses, H)


if __name__ == "__main__":


	roomname = "Room 4"
	# posefilename ="/home/nickybones/data/Text/Dump/" + roomname + "/gopro/poses.txt"
	# relativePoses, robotPoses, ids = getRelPoses(posefilename, roomname)
	# predfilename = "/home/nickybones/data/Text/Dump/" + roomname + "/camera/predictions.txt"
	# matches = performance(predfilename, roomname)
	# matches = matches[ids]
	# H = computeHist(relativePoses, matches)


	# data = {
	# 'x': robotPoses[:, 0].squeeze(),
 #    'y': robotPoses[:, 1].squeeze(),
 #    'theta': robotPoses[:, 2].squeeze(),
 #    'rel_x': relativePoses[:, 0].squeeze(),
 #    'rel_y': relativePoses[:, 1].squeeze(),
 #    'rel_theta': relativePoses[:, 2].squeeze(),
 #    'match': matches
	# }
	# df = pd.DataFrame(data)
	# df.to_pickle("/home/nickybones/data/Text/Dump/" + roomname + ".pickle")

	# VizHeatMapandPose(relativePoses, H)

	loadPickle("/home/nickybones/data/Text/Dump/" + roomname + ".pickle")

    