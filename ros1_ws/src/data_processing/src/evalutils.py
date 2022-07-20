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

    camTF = np.array([0.08, 0, 1]);
    theta = gt[2] + np.pi / 2
    v = np.array([0, 0, theta])

    trans = np.array([[np.cos(theta), np.sin(theta), 0], [-np.sin(theta), np.cos(theta), 0], [0, 0, 1]])

    gt_fix = trans @ camTF;
    gt_fix[0] += gt[0];
    gt_fix[1] += gt[1];
    gt_fix[2] = theta

    return gt_fix



def EvalSingleRun(csv_file_path):


    #csv_file_path = "/home/nickybones/data/iros2022/2021_12_08_Run3/" + t + "/Run" + str(r)+ "/poseestimation.csv"
    df = pd.read_csv(csv_file_path)  

    samples = len(df['pose_x'].to_numpy()) - 25 #1554

    p_x = df['pose_x'].to_numpy()
    p_y = df['pose_y'].to_numpy()
    p_yaw = df['pose_yaw'].to_numpy()

    p_traj = np.zeros((len(p_x), 2))
    for i in range(samples):
        p_traj[i] = [p_x[i], p_y[i]]

    gt_x = df['gt_x'].to_numpy()
    gt_y = df['gt_y'].to_numpy()
    gt_yaw = df['gt_yaw'].to_numpy() #+ 0.5 * np.pi
    gt_traj = np.zeros((len(gt_x), 3))

    for i in range(samples):
        #gt_traj[i] = [gt_x[i], gt_y[i]]
        gt = np.array([gt_x[i], gt_y[i], gt_yaw[i]])
        gt_traj[i] = cam2BaselinkTF(gt)

    gt_fix = gt_traj

    err_xy_dist = np.sqrt((p_traj[:, 0] - gt_fix[:, 0]) ** 2 + (p_traj[:, 1] - gt_fix[:, 1]) ** 2)

    err_xy_dist_max = np.max(err_xy_dist)
    err_xy_dist_avg = np.mean(err_xy_dist)

    err_ang_dist = np.zeros(len(p_yaw))

    for i in range(samples):
        theta = p_yaw[i]
        theta2 = gt_fix[i][2]

        u = np.array([np.cos(theta), np.sin(theta)])
        v = np.array([np.cos(theta2), np.sin(theta2)])
        dist = angDist(u, v)
        err_ang_dist[i] = dist

    err_ang_dist_max = np.max(err_ang_dist)
    err_ang_dist_avg = np.mean(err_ang_dist)

    return err_xy_dist_avg, err_ang_dist_avg


def EvalAllRuns(run_nums, type_run, folderPath):

    xy_avg = []
    ang_avg = []
    xy_max = []
    ang_max = []


    for r in range(0, run_nums):
        csv_file_path = folderPath + type_run + "/Run" + str(r)+ "/poseestimation.csv"
        #csv_file_path = "/home/nickybones/data/MCL/2021_12_08/Run3/"+ type_run + "/Run" + str(r)+ "/poseestimation.csv"
        err_xy_dist_avg, err_ang_dist_avg = EvalSingleRun(csv_file_path)
        xy_avg.append(err_xy_dist_avg)
        ang_avg.append(err_ang_dist_avg)
        #print("Run {}, xy_avg_err {}".format(r, err_xy_dist_avg))

    print("{} & ({:.3f}, {:.3f}) & ({:.3f}, {:.3f}) & ({:.3f}, {:.3f}) & ({:.3f}, {:.3f})  \\\\ \\hline".format(type_run, np.mean(xy_avg[0:3]), np.mean(ang_avg[0:3]), np.mean(xy_avg[3:6]), np.mean(ang_avg[3:6]), np.mean(xy_avg[6:9]), np.mean(ang_avg[6:9]), np.mean(xy_avg[9:12]), np.mean(ang_avg[9:12])))

    return np.mean(xy_avg[0:3]), np.mean(ang_avg[0:3]), np.mean(xy_avg[3:6]), np.mean(ang_avg[3:6]), np.mean(xy_avg[6:9]), np.mean(ang_avg[6:9]), np.mean(xy_avg[9:12]), np.mean(ang_avg[9:12])


def computeErr4Sequence(df):

    samples = len(df['pose_x'].to_numpy()) - 25 #1554

    p_x = df['pose_x'].to_numpy()
    p_y = df['pose_y'].to_numpy()
    p_yaw = df['pose_yaw'].to_numpy()
    t = df['t']

    p_traj = np.zeros((len(p_x), 2))
    for i in range(samples):
        p_traj[i] = [p_x[i], p_y[i]]

    gt_x = df['gt_x'].to_numpy()
    gt_y = df['gt_y'].to_numpy()
    gt_yaw = df['gt_yaw'].to_numpy() #+ 0.5 * np.pi
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

def compueAvgErr4Sequence(err_xy_dist, err_ang_dist):

    samples = len(err_xy_dist)
    err_xy_avg = np.zeros(samples)
    err_ang_avg = np.zeros(samples)

    for i in range(samples):
        xy = np.mean(err_xy_dist[:i])
        ang = np.mean(err_ang_dist[:i])
        err_xy_avg[i] = xy
        err_ang_avg[i] = ang

    return err_xy_avg, err_ang_avg
