import numpy as np
import pandas as pd
import math
from scipy.interpolate import interp1d
from bisect import insort, bisect_left
from collections import deque
from itertools import islice
from scipy.spatial.transform import Rotation as R



def str2Pose(line):

	comps = line.split()
	pid = int(comps[0])
	strmtx = np.asarray(comps[4:])
	mtx = strmtx.reshape((4,4)).astype(float)
	Rot = mtx[:3, :3]
	#yaw, pitch, roll = rotationMatrixToEulerAngles(R)
	r = R.from_matrix(Rot)
	_, _, roll = r.as_euler('xyz', degrees=False)
	x = mtx[0][3]
	y = mtx[1][3]

	return x, y, roll, pid


def running_median_insort(seq, window_size):
	"""Contributed by Peter Otten"""
	seq = iter(seq)
	d = deque()
	s = []
	result = []
	for item in islice(seq, window_size):
	    d.append(item)
	    insort(s, item)
	    result.append(s[len(d)//2])
	m = window_size // 2
	for item in seq:
	    old = d.popleft()
	    d.append(item)
	    del s[bisect_left(s, old)]
	    insort(s, item)
	    result.append(s[m])
	return result


def interpolate(t, poses, kind='nearest', window=12, tol=0.5):
	"""Contributed by Jerome Guzzi and adapted to our data"""

	n = len(poses)
	data = np.zeros((n, 4))

	for i in range(n):
	  data[i, 0] = t[i]
	  data[i, 1:] = poses[i]


	# Filter outliers in yaw
	data = data[np.abs(data[:, 3] - running_median_insort(data[:, 3], window)) < tol]
	data[0, -1] = np.fmod(data[0, -1], 2 * np.pi)

	return interp1d(data[:, 0], data[:, 1:], axis=0, fill_value='extrapolate', assume_sorted=True, kind=kind)




def localizationPickle(dumpFolder, pickleFolder):

	pickleName = pickleFolder + "gopro.pickle"
	posesFile = dumpFolder + "poses.txt"

	df = pd.read_pickle(pickleName)

	f = open(posesFile, "r")
	lines = f.readlines()

	gt_poses = []
	gt_t = []

	for i in range(len(lines)):
		x, y, yaw, pid = str2Pose(lines[i])
		gt_poses.append(np.array([x, y, yaw]))
		row = df.iloc[pid]
		gt_t.append(row['t'])

	
	mergedPickle = dumpFolder + "merged.pickle"
	
	intr = interpolate(gt_t, gt_poses)

	merged_df = pd.read_pickle(mergedPickle)
	merged_t = merged_df['t']

	intr_poses = intr(merged_t).tolist()
	print(intr_poses)

	data = {'t': merged_t, 'type': 'gt', 'data': intr_poses}
	df = pd.DataFrame(data)

	dfcomb = pd.concat([df,merged_df], axis=0)
	dfcomb['t'] = dfcomb['t'].astype(int)
	dfcomb = dfcomb.sort_values(by='t')
	dfcomb.to_pickle(dumpFolder + "gtmerged.pickle")



if __name__ == "__main__":


	dumpFolder = "/home/nickybones/data/MCL/2021_12_07/Run2/"
	pickleFolder = "/home/nickybones/data/MCL/2021_12_07/Run2/"


	localizationPickle(dumpFolder, pickleFolder)
	


		