import numpy as np
import pandas as pd
import cv2
from GMAP import GMAP
from evalutils import cam2BaselinkTF, angDist, EvalAllRuns, point2inch
import matplotlib.pyplot as plt
from matplotlib import rcParams
from matplotlib.ticker import FormatStrFormatter

rcParams['hatch.linewidth'] = 5.0
rcParams.update({'font.size': 7})
rcParams.update({'pdf.fonttype': 42})
# rcParams['text.usetex'] = True
linewidth_latex = 245.71811


if __name__ == "__main__":

	folderPath = "/home/nickybones/data/iros2022/2021_12_08_Run3/particles_"
	partciles = [300, 500, 1000, 10000]
	run_nums = 12
	type_run = "singletimebb"


	#types = ["notext", "seeds", "naivebb", "onlywhennotbb", "singletimebb"]
	colors = ['r', 'y', 'b', 'k', 'g']
	markers = ['*', 'o', '^', 'D', 's']
	fig = plt.figure(figsize=(point2inch(linewidth_latex), point2inch(linewidth_latex)))

	for i, p in enumerate(partciles):
		s1_xy, s1_ang, s2_xy, s2_ang, s3_xy, s3_ang, s4_xy, s4_ang = EvalAllRuns(run_nums, type_run, folderPath + str(p) + "/")
		s_xy = [s1_xy, s2_xy, s3_xy, s4_xy]
		s_ang = [s1_ang, s2_ang, s3_ang, s4_ang]
		plt.plot([1,2,3,4], s_xy, marker=markers[i], label=p, markersize=1)

	plt.title('Average Trajectory Error (xy) vs. Particle #')
	plt.legend()
	plt.ylabel(' m ')
	plt.xlabel('#')
	plt.tight_layout(pad=0.0, rect=[0.01, 0.01, 0.99, 0.99])
	fig.savefig(folderPath + 'Average Trajectory Error (xy) vs. Particle #.eps', format='eps')
	plt.show()