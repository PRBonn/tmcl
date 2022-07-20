import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation as R

from GMAP import GMAP


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


def wrapToPi(theta):
    while theta < -np.pi:
        theta = theta + 2 * np.pi
    while theta > np.pi:
        theta = theta - 2 * np.pi
    return theta


def vizORientations(poses):

	#plt.gca().invert_yaxis()
	x = poses[:, 0]
	y = poses[:, 1]
	yaw = poses[:, 2]

	plt.scatter(x, y, c='k', s=2)

	for i in range(len(yaw)):
		ang = yaw[i][0]
		dx = np.cos(ang)
		dy = np.sin(ang)
		plt.arrow(x[i][0], y[i][0], 0.05*dx, 0.05*dy) 

	#plt.savefig("/home/nickybones/Code/YouBotMCL/ncore/build/" + str(90 * a) + "particles.png")
	#plt.close()
	plt.show()


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

def avgYaw(angles):

	x = np.cos(angles)
	y = np.sin(angles)
	avg_x = np.mean(x)
	avg_y = np.mean(y)

	avg_yaw = np.arctan2(avg_y, avg_x)

	return avg_yaw


def computeFloorHist(gmap, poses, matches, room_num):


	h, w = gmap.map.shape
	textmap = np.zeros((h, w, 3), dtype='uint8')
	heatmap = np.zeros((h, w), dtype='float32')
	yawmap = np.zeros((h, w), dtype='uint8')
	detect_orientations = poses[:, 2]
	detect_orientations = detect_orientations[matches > 0]
	avg_yaw = wrapToPi(avgYaw(detect_orientations))
	#print("avg_yaw: {}".format(avg_yaw))

	avg_yaw = round(255 * ((avg_yaw + np.pi) / (2 * np.pi)))
	roommap = np.zeros((h, w), dtype='uint8')
	print("avg_yaw: {}".format(avg_yaw))

	for i in range(poses.shape[0]):
		p = poses[i]
		m = matches[i]

		u, v = gmap.world2map(p)
		if m > 0:
			heatmap[v, u] += 1.0
			yawmap[v, u] = avg_yaw
			roommap[v, u] = room_num

	heatmap = 255 * (heatmap / np.max(heatmap))
	textmap[:, :, 0] = heatmap.astype("uint8")
	textmap[:, :, 1] = yawmap
	textmap[:, :, 2] = roommap
	#heatmap = heatmap / np.sum(heatmap)

	return textmap




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
	

def getShiftedPoses(posefilename):

	posefile = open(posefilename, 'r')
	lines = posefile.readlines()
	robotPoses = []
	ids = []

	camTF = np.array([[0.08], [0], [1]])

	 
	count = 0
	for l in lines:
		comps = l.split()
		poseid = int(comps[0])
		strmtx = np.asarray(comps[4:])
		mtx = strmtx.reshape((4,4)).astype(float)
		Rot = mtx[:3, :3]
		r = R.from_matrix(Rot)
		_, _, roll = r.as_euler('xyz', degrees=False)
		x = mtx[0][3]
		y = mtx[1][3]
		pose = np.array([[x], [y], [roll]])
		theta = roll + np.pi / 2
		trans = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
		gt = trans @ camTF
		gt[0] += x
		gt[1] += y
		gt[2] = theta
		robotPoses.append(gt)
		ids.append(poseid)


	robotPoses = np.asarray(robotPoses)
	
	return robotPoses, ids


def getRelPoses(posefilename, roomname):

	posefile = open(posefilename, 'r')
	lines = posefile.readlines()
	robotPoses = []
	relativePoses = []
	ids = []

	roompose = textPoses[roomname]
	dummyTrans = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
	#roomTrans = np.array([[1, 0, roompose[0]], [0, 1, roompose[1]], [0, 0, 1]])
	roomTrans = np.array([[np.cos(roompose[2]), -np.sin(roompose[2]), roompose[0]], [np.sin(roompose[2]), np.cos(roompose[2]), roompose[1]], [0, 0, 1]])
	roomTrans = np.linalg.inv(roomTrans)

	 
	count = 0
	for l in lines:
		comps = l.split()
		poseid = int(comps[0])
		strmtx = np.asarray(comps[4:])
		mtx = strmtx.reshape((4,4)).astype(float)
		Rot = mtx[:3, :3]
		r = R.from_matrix(Rot)
		_, _, roll = r.as_euler('xyz', degrees=False)
		x = mtx[0][3]
		y = mtx[1][3]
		pose = np.array([[x], [y], [roll]])
		robotPoses.append(pose)

		relpose = np.array([[x], [y], [1]])
		relpose = roomTrans @ relpose
		relpose[2] = roll - roompose[2]
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

# def loadPickle(path):

# 	df = pd.read_pickle(path)

# 	rel_x = df['rel_x']
# 	rel_y = df['rel_y']
# 	matches = df['match']
# 	print(rel_x.shape)

# 	relativePoses = np.concatenate((rel_x, rel_y), axis=0)
# 	print(relativePoses.shape)
# 	H = computeHist(relativePoses, matches)
# 	VizHeatMapandPose(relativePoses, H)


def oneRoom(room_num):


	print("room: {}".format(room_num))
	roomname = "Room " + str(room_num)
	roomFolder = "/home/nickybones/data/Text/2021_12_01/Room" + str(room_num) + "/"
	posefilename = roomFolder + "poses.txt"
	robotPoses, ids = getShiftedPoses(posefilename)
	predfilename = roomFolder + "predictions.txt"
	matches = performance(predfilename, roomname)
	matches = matches[ids]
	#vizORientations(robotPoses)
	#H = computeHist(robotPoses, matches)
	#VizHeatMapandPose(robotPoses, H)

	gridmap = cv2.imread("/home/nickybones/Code/YouBotMCL/ncore/data/floor/Faro/FMap.pgm")
	gmap = GMAP(gridmap, 0.05, [-13.9155, -10.99537, 0.0])
	

	textmap = computeFloorHist(gmap, robotPoses, matches, room_num)
	cv2.imwrite(roomFolder + "../Room " + str(room_num) + ".png", textmap)
	
	r = textmap[:, :, 0]
	locations = cv2.findNonZero(r)
	x,y,w,h = cv2.boundingRect(locations)
	print("{0}, {1}, {2}, {3}".format(x,y,w,h))

	
	g = np.zeros(gmap.map.shape, dtype='uint8')
	b = np.zeros(gmap.map.shape, dtype='uint8')
	heatmap = cv2.merge((r,g,b))
	heatmap = 255 * heatmap
	cv2.rectangle(heatmap,(x,y),(x+w,y+h),(200,0,0),2)
	comb = 255 - gridmap + heatmap
	#cv2.imshow("", comb)
	#cv2.waitKey()



if __name__ == "__main__":


	gridmap = cv2.imread("/home/nickybones/Code/YouBotMCL/ncore/data/floor/Faro/FMap.pgm")
	gmap = GMAP(gridmap, 0.05, [-13.9155, -10.99537, 0.0])
	h, w = gmap.map.shape
	textmap = np.zeros((h, w, 3), dtype='float32')


	for i in range(1, 11):
		oneRoom(i)
		


	