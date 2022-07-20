import numpy as np
import pandas as pd
import cv2
from scipy.spatial import distance
from evalutils import cam2BaselinkTF, angDist



def computeErr4Sequence(df_pred, df_gt):

    samples = len(df_pred['pose_x'].to_numpy()) - 25 #1554

    p_x = df_pred['pose_x'].to_numpy()
    p_y = df_pred['pose_y'].to_numpy()
    p_yaw = df_pred['pose_yaw'].to_numpy()
    t = df_pred['t']

    p_traj = np.zeros((len(p_x), 2))
    for i in range(samples):
        p_traj[i] = [p_x[i], p_y[i]]

    gt_x = df_gt['gt_x'].to_numpy()
    gt_y = df_gt['gt_y'].to_numpy()
    gt_yaw = df_gt['gt_yaw'].to_numpy() #+ 0.5 * np.pi
    gt_traj = np.zeros((len(gt_x), 3))

    for i in range(samples):
        #gt_traj[i] = [gt_x[i], gt_y[i]]
        gt = np.array([gt_x[i], gt_y[i], gt_yaw[i]])
        gt_traj[i] = cam2BaselinkTF(gt)

    gt_fix = gt_traj

    err_xy_dist = np.sqrt((p_traj[:, 0] - gt_fix[:, 0]) ** 2 + (p_traj[:, 1] - gt_fix[:, 1]) ** 2)

    err_ang_dist = np.zeros(len(p_yaw))

    for i in range(samples):
        theta = p_yaw[i]
        theta2 = gt_fix[i][2]

        u = np.array([np.cos(theta), np.sin(theta)])
        v = np.array([np.cos(theta2), np.sin(theta2)])
        dist = angDist(u, v)
        err_ang_dist[i] = dist


    return err_xy_dist, err_ang_dist, t.to_numpy()



def time_to_convergence(err_xy_dist, err_ang_dist, th):


	for i in range(len(err_xy_dist)):

		if err_xy_dist[i] < th:
			return  i

	return -1


if __name__ == "__main__":

	types = ["amcl", "notext", "hardbox", "softbox", "nicky"]
	names = ["AMCL", "MCL     ", "SM1     ", "SM2     ", "MCL+Text"]
	#run_nums = [10]
	#run_nums = [3]
	run_nums = [4, 1, 3, 2]
	ds = ["D1", "D2", "D3", "D4"]
	#ds = ["D3"]
	#ds = [""]
	#folderPath = "/home/nickybones/data/iros2022/FMap/2021_12_08_Run3/" 
	folderPath = "/home/nickybones/data/iros2022/GMap/" 
	particles = [300]
	th = 0.5
	p = 300
	s1 = []
	s2 = []

	#for p in particles:
	for i, tp in enumerate(types):
		strout = names[i] + ":"
		strout2 = names[i] + ":"
		for d in range(len(ds)):
			#strout = names[i] + ":"
			#strout2 = names[i] + ":"
			for r in range(run_nums[d]):
				csv_file_path = folderPath + ds[d] + "/particles_" + str(p) + "/" + tp + "/Run" + str(r) + "/poseestimation.csv"
				df_pred = pd.read_csv(csv_file_path)
				df_gt = pd.read_csv(csv_file_path)
				err_xy_dist, err_ang_dist, t = computeErr4Sequence(df_pred, df_gt)
				ind = time_to_convergence(err_xy_dist, err_ang_dist, th)

				if ind == -1:
					strout2 += " &{-}/{-}    "
					#strout += "	&({:.1f})".format(-1)
					strout += "	&{:.1f}".format(-1)
				else:
					t_tot = (t[len(err_xy_dist)-25] - t[0]) / 1000000000
					t_conv = (t[ind] - t[0]) / 1000000000
					avg_xy = np.mean(err_xy_dist[ind:])
					avg_ang = np.mean(err_ang_dist[ind:])
					#strout += "	&({:.1f})".format(t_conv)
					strout += "	&{:.1f}".format(t_conv)
					if t_conv / t_tot > 0.95 :
						strout2 += " &{-}/{-}    "
					else:
						strout2 += " &{:.3f}/{:.3f}".format(avg_ang, avg_xy)
				#strout2 += "{}/{}	".format(ind, len(err_xy_dist))
				#print(strout2)
		strout += "\\\\"
		strout2 += "\\\\"
		print(strout2)
		s1.append(strout)
		s2.append(strout2)
