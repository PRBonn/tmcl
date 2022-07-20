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
	seq = ["S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8", "S9", "S10"]
	rn = 10
	th = 0.5
	resultsFolder = "/home/nickybones/Code/zimmerman2022iros/pics/"
	particles = [300, 500, 1000, 10000]
	types = ["amcl", "notext", "hardbox", "softbox", "nicky"]
	names = ["AMCL", "MCL", "SM1", "SM2", "MCL+text"]
	type_num = len(types)
	
	colors = ['r', 'y', 'b', 'k', 'g']
	markers = ['*', 'o', '^', 'D', 's']
	fig = plt.figure(figsize=(point2inch(linewidth_latex), 0.5 * point2inch(linewidth_latex)))

	xy_avg = np.zeros((type_num, 4))

	x_ticks = [2 , 8, 14, 20]

	for j, p in enumerate(particles):
		for i, tp in enumerate(types):
			#xy_vec = np.zeros((1, rn))
			xy_vec = []
			for r in range(rn):
				csv_file_path = folderPath + str(p) + "/" + tp + "/Run" + str(r) + "/poseestimation.csv"
				df_pred = pd.read_csv(csv_file_path)
				df_gt = pd.read_csv(csv_file_path)
				err_xy_dist, err_ang_dist, t = computeErr4Sequence(df_pred, df_gt)
				ind = time_to_convergence(err_xy_dist, err_ang_dist, th)
				if ind > -1:
					t_conv = (t[ind] - t[0]) / 1000000000
					t_tot = (t[len(err_xy_dist)-25] - t[0]) / 1000000000
					if t_conv / t_tot < 0.95 :
						avg_xy = np.mean(err_xy_dist[ind:])
						avg_ang = np.mean(err_ang_dist[ind:])
						xy_vec.append(avg_xy)
					else:
						xy_vec.append(np.nanmean(err_xy_dist))
				else:
					xy_vec.append(np.nanmean(err_xy_dist))

				#xy_vec[:, r] = avg_xy
			xy_vec = np.asarray(xy_vec)	
			xy_avg[i, j]= np.nanmean(xy_vec)

	print(xy_avg)
	print(xy_avg.shape)
	print(xy_avg[0, :])

	clr = cm.rainbow(np.linspace(0, 1, 2*len(types)))
	clrs = [clr[0], clr[1], clr[5], clr[7], clr[9]]


	#plt.bar([0, 1, 2, 3], xy_avg[0, :], width=0.3, label=names[0], color=clr[0])

	for i, tp in enumerate(types):
		plt.bar([i , i + type_num+1 , i + type_num*2+2 , i + type_num*3+3], xy_avg[i, :], width=1, label=names[i], color=clrs[i])


	plt.legend(ncol=1, loc='best')
	#plt.ylim(0, 0.8)
	plt.ylabel(' ATE (m)')
	plt.xlabel('Number of particles')
	plt.xticks(ticks=x_ticks, labels=particles)
	plt.tight_layout(pad=0.0, rect=[0.01, 0.01, 0.99, 0.99])
	#fig.savefig(resultsFolder + 'AblationAVG.eps', format='eps')
	fig.savefig(resultsFolder + 'ParticleNumAVG.pdf')
	plt.show()