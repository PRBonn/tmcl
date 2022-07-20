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

	folderPath = "/home/nickybones/data/iros2022/FMap/2021_12_08_Run3/particles_300/"
	seq = ["S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8", "S9", "S10"]
	rn = 10
	th = 0.5
	type_run = "nicky"
	resultsFolder = "/home/nickybones/Code/zimmerman2022iros/pics/"

	
	types = ["seeds", "repeat", "conservative", "nicky"]
	names = ["seeds", "repeat", "conservative", "MCL+text"]
	colors = ['r', 'y', 'b', 'k', 'g']
	markers = ['*', 'o', '^', 'D', 's']
	fig = plt.figure(figsize=(point2inch(linewidth_latex), 0.5 * point2inch(linewidth_latex)))

	xy_avg = []

	x_ticks = [0.5 ,1.1, 1.7, 2.3]

	for i, tp in enumerate(types):
		xy_vec = np.zeros((1, rn))
		for r in range(rn):
			csv_file_path = folderPath + tp + "/Run" + str(r) + "/poseestimation.csv"
			df_pred = pd.read_csv(csv_file_path)
			df_gt = pd.read_csv(csv_file_path)
			err_xy_dist, err_ang_dist, t = computeErr4Sequence(df_pred, df_gt)
			ind = time_to_convergence(err_xy_dist, err_ang_dist, th)
			t_conv = (t[ind] - t[0]) / 1000000000
			t_tot = (t[len(err_xy_dist)-25] - t[0]) / 1000000000
			avg_xy = np.mean(err_xy_dist[ind:])
			avg_ang = np.mean(err_ang_dist[ind:])

			xy_vec[:, r] = avg_xy

		xy_avg.append(np.mean(xy_vec))

	print(xy_avg)
	#print(xy_avg.shape)

	clr = cm.rainbow(np.linspace(0, 1, 2*len(types)))
	clrs = [clr[1], clr[3], clr[6], clr[7]]
	for i, tp in enumerate(types):

		plt.bar([i*0.6 + 0.5], xy_avg[i], width=0.3, label=names[i], color=clrs[i])


	plt.legend(ncol=1, loc='best')
	plt.ylim(0, 0.8)
	plt.ylabel(' ATE (m)')
	plt.xlabel('Injection method')
	plt.xticks(ticks=x_ticks, labels=names)
	plt.tight_layout(pad=0.0, rect=[0.01, 0.01, 0.99, 0.99])
	#fig.savefig(resultsFolder + 'AblationAVG.eps', format='eps')
	fig.savefig(resultsFolder + 'AblationAVG.pdf')
	plt.show()