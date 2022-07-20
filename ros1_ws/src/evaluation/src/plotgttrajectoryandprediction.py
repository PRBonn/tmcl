import numpy as np
import pandas as pd
import cv2
from evalutils import cam2BaselinkTF, angDist, EvalAllRuns, point2inch
import matplotlib.pyplot as plt
from matplotlib import cm

from matplotlib import rcParams
from matplotlib.ticker import FormatStrFormatter
from GMAP import GMAP

rcParams['hatch.linewidth'] = 5.0
rcParams.update({'font.size': 7})
rcParams.update({'pdf.fonttype': 42})
# rcParams['text.usetex'] = True
linewidth_latex = 245.71811



if __name__ == "__main__":

	#run_name = "2021_12_08/Run3"
	run_name = "2021_12_08_Run3"
	#folderPath = "/home/nickybones/data/MCL/Dump/"
	#folderPath = "/home/nickybones/data/MCL/" + run_name + "/particles_"
	folderPath = "/home/nickybones/data/iros2022/FMap/" + run_name + "/particles_"
	partciles = [300, 500, 1000, 10000]
	types = ["gt", "notext", "singletimebb"]
	names = ["ground truth", "without text", "with text"]
	csv_file_path =  folderPath + str(300) + "/notext/Run8/poseestimation.csv"
	csv_file_path2 =  folderPath + str(300) + "/nicky/Run8/poseestimation.csv"
	#csv_file_path =  folderPath + "poseestimation.csv"
	resultsFolder = "/home/nickybones/Code/zimmerman2022iros/pics/"

	gridmap = cv2.imread("/home/nickybones/Code/YouBotMCL/ncore/data/floor/FMap/FMap.pgm")
	mapdoors = cv2.cvtColor(cv2.imread("/home/nickybones/Code/zimmerman2022iros/pics/FMapDoors.png"), cv2.COLOR_BGR2RGB)
	gmap = GMAP(gridmap, 0.05, [-13.9155, -10.99537, 0.0])

	df = pd.read_csv(csv_file_path)  
	df2 = pd.read_csv(csv_file_path2)  


	samples = len(df['gt_x'].to_numpy()) - 25 #1554
	samples = 100

	gt_x = df['gt_x'].to_numpy()
	gt_y = df['gt_y'].to_numpy()
	gt_yaw = df['gt_yaw'].to_numpy()

	p_x = df['pose_x'].to_numpy()
	p_y = df['pose_y'].to_numpy()
	p_yaw = df['pose_yaw'].to_numpy()

	pt_x = df2['pose_x'].to_numpy()
	pt_y = df2['pose_y'].to_numpy()
	pt_yaw = df2['pose_yaw'].to_numpy()

	fig = plt.figure(figsize=(point2inch(linewidth_latex), 0.5 * point2inch(linewidth_latex)))
	clr = cm.rainbow(np.linspace(0, 1, samples))
	im = plt.imshow(mapdoors)
	#cbar = fig.colorbar(im, ticks=[0, 85, 170, 255])
	#cbar.set_ticklabels(np.array([0, int(samples / 3), int(2 * samples / 3), samples]))

	offset = 450
	#for i in range(samples):
	for i in range(offset, offset+samples):
	 	gt = [gt_x[i], gt_y[i]]
	 	p = [p_x[i], p_y[i]]
	 	pt = [pt_x[i], pt_y[i]]
	 	u, v = gmap.world2map(gt)
	 	up, vp = gmap.world2map(p)
	 	ut, vt = gmap.world2map(pt)
	 	plt.plot(u, v, 'o',  markersize=1, color='k', label=names[0])
	 	plt.plot(up, vp, 'o',  markersize=1, color='b', label=names[1])
	 	plt.plot(ut, vt, 'o',  markersize=1, color='g', label=names[2])

	 	if i%40 == 0 :
		 	plt.plot(u, v, '>',  markersize=5, color='k')
		 	plt.plot(up, vp, '<', markersize=5, color='b')
		 	plt.plot(ut, vt, '>',  markersize=5, color='g')
	 	

	lgnd = plt.legend(names, markerscale=5)
	
	plt.axis('off')
	plt.tight_layout(pad=0.0, rect=[0.01, 0.01, 0.99, 0.99])
	fig.savefig(resultsFolder + 'MotivationTrajectory.eps', format='eps')
	fig.savefig(resultsFolder + 'MotivationTrajectory.pdf')
	plt.show()