
import numpy as np
import pandas as pd
import cv2
from scipy.spatial import distance
import matplotlib.pyplot as plt
from matplotlib import rcParams
from matplotlib.ticker import FormatStrFormatter

rcParams['hatch.linewidth'] = 5.0
rcParams.update({'font.size': 7})
rcParams.update({'pdf.fonttype': 42})
# rcParams['text.usetex'] = True
linewidth_latex = 245.71811


def point2inch(x):
    return x / 72.


def angDist(a, b):

    dist = 1 - (a @ b.T)/(np.linalg.norm(a) * np.linalg.norm(b))

    return dist


def cam2BaselinkTF(gt):

    camTF = np.array([0.08, 0.02, 1]);
    #theta = gt[2] + np.pi * 88 / 180#np.pi / 2
    theta = gt[2] + np.pi / 2
    v = np.array([0, 0, theta])

    trans = np.array([[np.cos(theta), np.sin(theta), 0], [-np.sin(theta), np.cos(theta), 0], [0, 0, 1]])

    gt_fix = trans @ camTF;
    gt_fix[0] += gt[0];
    gt_fix[1] += gt[1];
    gt_fix[2] = theta

    return gt_fix


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
