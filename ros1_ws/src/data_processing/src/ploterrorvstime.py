import numpy as np
import pandas as pd
import cv2
from GMAP import GMAP
from evalutils import cam2BaselinkTF, angDist, EvalAllRuns, point2inch, computeErr4Sequence, compueAvgErr4Sequence
import matplotlib.pyplot as plt
from matplotlib import rcParams
from matplotlib.ticker import FormatStrFormatter


rcParams['hatch.linewidth'] = 5.0
rcParams.update({'font.size': 7})
rcParams.update({'pdf.fonttype': 42})
# rcParams['text.usetex'] = True
linewidth_latex = 245.71811


# def computeErr4Sequence(df):

# 	samples = len(df['pose_x'].to_numpy()) - 25 #1554

# 	p_x = df['pose_x'].to_numpy()
# 	p_y = df['pose_y'].to_numpy()
# 	p_yaw = df['pose_yaw'].to_numpy()
# 	t = df['t']

# 	p_traj = np.zeros((len(p_x), 2))
# 	for i in range(samples):
# 	 	p_traj[i] = [p_x[i], p_y[i]]

# 	gt_x = df['gt_x'].to_numpy()
# 	gt_y = df['gt_y'].to_numpy()
# 	gt_yaw = df['gt_yaw'].to_numpy() #+ 0.5 * np.pi
# 	gt_traj = np.zeros((len(gt_x), 3))

# 	for i in range(samples):
# 	 	#gt_traj[i] = [gt_x[i], gt_y[i]]
# 	 	gt = np.array([gt_x[i], gt_y[i], gt_yaw[i]])
# 	 	gt_traj[i] = cam2BaselinkTF(gt)

# 	gt_fix = gt_traj

# 	err_xy_dist = np.sqrt((p_traj[:, 0] - gt_fix[:, 0]) ** 2 + (p_traj[:, 1] - gt_fix[:, 1]) ** 2)

# 	err_ang_dist = np.zeros(len(p_yaw))

# 	for i in range(samples):
# 		theta = p_yaw[i]
# 		theta2 = gt_fix[i][2]

# 		u = np.array([np.cos(theta), np.sin(theta)])
# 		v = np.array([np.cos(theta2), np.sin(theta2)])
# 		dist = angDist(u, v)
# 		err_ang_dist[i] = dist


# 	return err_xy_dist, err_ang_dist, t.to_numpy()

# def compueAvgErr4Sequence(err_xy_dist, err_ang_dist):

# 	samples = len(err_xy_dist)
# 	err_xy_avg = np.zeros(samples)
# 	err_ang_avg = np.zeros(samples)

# 	for i in range(samples):
# 		xy = np.mean(err_xy_dist[:i])
# 		ang = np.mean(err_ang_dist[:i])
# 		err_xy_avg[i] = xy
# 		err_ang_avg[i] = ang

# 	return err_xy_avg, err_ang_avg


if __name__ == "__main__":

	folderPath = "/home/nickybones/data/iros2022/2021_12_08_Run3/particles_10000/"
	r = 0
	types = ["notext", "seeds", "naivebb", "onlywhennotbb", "singletimebb"]
	colors = ['r', 'y', 'b', 'k', 'g']
	markers = ['*', 'o', '^', 'D', 's']
	fig = plt.figure(figsize=(3.41275152778, 3.41275152778))


	for i, type_run in enumerate(types):
		#type_run = "singletimebb"
		csv_file_path = folderPath + type_run + "/Run" + str(r)+ "/poseestimation.csv"
		df = pd.read_csv(csv_file_path)  
		err_xy_dist, err_ang_dist, t = computeErr4Sequence(df)
		err_xy_avg, err_ang_avg = compueAvgErr4Sequence(err_xy_dist, err_ang_dist)
		t = t/1000000000
		t -= t[0]
		plt.plot(t, err_xy_avg, marker=markers[i], label=type_run, markersize=1, alpha=0.7)

	plt.title('Average Trajectory Error (xy) vs. Time')
	plt.legend()
	plt.ylabel('m')
	plt.xlabel('s')
	plt.tight_layout(pad=0.0, rect=[0.01, 0.01, 0.99, 0.99])
	fig.savefig(folderPath + 'Average Trajectory Error (xy) vs. Time.eps', format='eps')
	plt.show()