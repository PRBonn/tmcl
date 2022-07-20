import numpy as np
import pandas as pd
import cv2
from converge import time_to_convergence, computeErr4Sequence
from evalutils import point2inch
import matplotlib.pyplot as plt
from matplotlib import rcParams
from matplotlib.ticker import FormatStrFormatter
from matplotlib import cm

rcParams['hatch.linewidth'] = 5.0
rcParams.update({'font.size': 7})
rcParams.update({'pdf.fonttype': 42})
# rcParams['text.usetex'] = True
linewidth_latex = 245.71811


if __name__ == "__main__":

	folderPath = "/home/nickybones/data/iros2022/FMap/2021_12_08_Run3/particles_"
	particles = [300, 500, 1000, 10000]
	seq = ["S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8", "S9", "S10"]
	rn = 10
	th = 0.5
	type_run = "nicky"
	resultsFolder = "/home/nickybones/Code/zimmerman2022iros/pics/"

	#types = ["notext", "seeds", "naivebb", "onlywhennotbb", "singletimebb"]
	colors = ['r', 'y', 'b', 'k', 'g']
	markers = ['*', 'o', '^', 'D', 's']
	fig = plt.figure(figsize=(point2inch(linewidth_latex), 0.5 * point2inch(linewidth_latex)))

	xy_matrix = np.zeros((4, rn))
	ang_matrix = np.zeros((4, rn))

	x_ticks = [5 ,16, 27, 38]

	for i, p in enumerate(particles):
		print(p)
		for r in range(rn):
			csv_file_path = folderPath + str(p) + "/" + type_run + "/Run" + str(r) + "/poseestimation.csv"
			df_pred = pd.read_csv(csv_file_path)
			df_gt = pd.read_csv(csv_file_path)
			err_xy_dist, err_ang_dist, t = computeErr4Sequence(df_pred, df_gt)
			ind = time_to_convergence(err_xy_dist, err_ang_dist, th)
			t_conv = (t[ind] - t[0]) / 1000000000
			t_tot = (t[len(err_xy_dist)-25] - t[0]) / 1000000000
			avg_xy = np.mean(err_xy_dist[ind:])
			avg_ang = np.mean(err_ang_dist[ind:])

			xy_matrix[i, r] = avg_xy

	clr = cm.nipy_spectral(np.linspace(0, 1, rn))

	for r in range(rn):
		plt.bar([r ,rn+r+1, rn*2+r+2, rn*3+r+3], xy_matrix[:, r], width=1, label=seq[r], color=clr[r])

	plt.legend(ncol=5, loc='best')
	plt.ylim(0, 3)
	plt.ylabel(' ATE (m)')
	plt.xlabel('Number of particles N')
	plt.xticks(ticks=x_ticks, labels=particles)
	plt.tight_layout(pad=0.0, rect=[0.01, 0.01, 0.99, 0.99])
	#fig.savefig(resultsFolder + 'AverageTrajectoryErrorVSParticleNum.eps', format='eps')
	plt.show()