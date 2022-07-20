import numpy as np
import pandas as pd
import cv2
from scipy.spatial import distance
from evalutils import EvalAllRuns 



if __name__ == "__main__":

	types = ["notext", "singletimebb"]
	run_nums = 10
	folderPath = "/home/nickybones/data/iros2022/2022_01_14_Run1/" 
	particles = [300, 500, 1000, 10000]
	for t in types:
		res_str = "{}".format(t)
		for p in particles:
			ang_avg, xy_avg = EvalAllRuns(run_nums, t, folderPath + "particles_" + str(p) + "/")
			s1_ang, s1_xy, s2_ang, s2_xy = np.mean(ang_avg[0:5]), np.mean(xy_avg[0:5]), np.mean(ang_avg[5:10]), np.mean(xy_avg[5:10])
			res_str += " & {:.3f}/{:.3f}".format(s1_ang, s1_xy)
		res_str += "  \\\\"
		print(res_str)
