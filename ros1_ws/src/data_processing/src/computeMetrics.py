import numpy as np
import pandas as pd
import cv2
from scipy.spatial import distance
from evalutils import EvalAllRuns 


def svd_icp(ti, tj):  
	mu_i = np.mean(ti, axis=0)  
	mu_j = np.mean(tj, axis=0)  
	sigma_ij = 1 / (ti.shape[0]) * (ti - mu_i).T @ (tj - mu_j)  
	U, _, VT = np.linalg.svd(sigma_ij)  
	R = U @ VT      
	t = mu_i.T - R @ mu_j.T                                                       
	return R, t



if __name__ == "__main__":

	types = ["particles_300/notext", "particles_300/seeds", "particles_300/naivebb", "particles_300/onlywhennotbb", "particles_300/singletimebb"]
	#types = ["particles_1000/notext", "particles_1000/seeds", "particles_1000/naivebb", "particles_1000/onlywhennotbb", "particles_1000/singletimebb"]
	#types = ["particles_10000/notext", "particles_10000/seeds", "particles_10000/naivebb", "particles_10000/onlywhennotbb", "particles_10000/singletimebb"]
	#types = ["particles_500/notext", "particles_500/seeds", "particles_500/naivebb", "particles_500/onlywhennotbb", "particles_500/singletimebb"]
	run_nums = 12
	folderPath = "/home/nickybones/data/iros2022/2021_12_08_Run3/" 
	#folderPath = "/home/nickybones/data/MCL/2021_12_08/Run3/" 
	#EvalAllRuns(run_nums, "particles_300/singletimebb")
	#EvalAllRuns(run_nums, "particles_300/singletimebb", folderPath)


	for t in types:
	 	EvalAllRuns(run_nums, t, folderPath)

	#csv_file_path = "/home/nickybones/data/MCL/2021_12_08/Run3/amcl_results.csv"
	# csv_file_path = "/home/nickybones/data/MCL/2021_12_08/Run3/particles_1000/singletimebb/Run0/poseestimation.csv"
	# err_xy_dist_avg, err_ang_dist_avg = EvalSingleRun(csv_file_path)
	# print(err_xy_dist_avg, err_ang_dist_avg)