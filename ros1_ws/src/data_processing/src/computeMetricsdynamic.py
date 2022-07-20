import numpy as np
import pandas as pd
import cv2
from scipy.spatial import distance
from evalutils import EvalAllRuns 


if __name__ == "__main__":

	types = ["notext", "seeds", "naivebb", "singletimebb", "onlywhennotbb"]
	run_nums = 12
	folderPath = "/home/nickybones/data/iros2022/2021_12_08_Run3/" 
	particles = [300, 500, 1000, 10000]
	for p in particles:
		print("particles: {}".format(p))
		for t in types:
			ang_avg, xy_avg = EvalAllRuns(run_nums, t, folderPath + "particles_" + str(p) + "/")
			s1_ang, s1_xy, s2_ang, s2_xy, s3_ang, s3_xy, s4_ang, s4_xy = np.mean(ang_avg[0:3]), np.mean(xy_avg[0:3]), np.mean(ang_avg[3:6]), np.mean(xy_avg[3:6]), np.mean(ang_avg[6:9]), np.mean(xy_avg[6:9]), np.mean(ang_avg[9:12]), np.mean(xy_avg[9:12])
			res_str = "{} & {:.3f}/{:.3f} & {:.3f}/{:.3f} & {:.3f}/{:.3f} & {:.3f}/{:.3f}  \\\\ ".format(t, s1_ang, s1_xy, s2_ang, s2_xy, s3_ang, s3_xy, s4_ang, s4_xy)
			print(res_str)
	