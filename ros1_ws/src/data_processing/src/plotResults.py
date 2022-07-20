import numpy as np
import pandas as pd
import cv2
from GMAP import GMAP


if __name__ == "__main__":
	csv_file_path = "/media/nickybones/My Passport/youbot_localization/2021_12_08/Run3/results/nosemnotext/Run3/poseestimation.csv"
	csv_file_path2 = "/media/nickybones/My Passport/youbot_localization/2021_12_08/Run3/results/nosemyessophitext/Run3/poseestimation.csv"
	img_file_path = "/home/nickybones/Code/YouBotMCL/ncore/data/floor/Faro/FMap.pgm"
	df = pd.read_csv(csv_file_path)  
	df2 = pd.read_csv(csv_file_path2)  
	gridmap = cv2.imread(img_file_path)
	print(df.head())

	p_x = df['pose_x']
	p_y = df['pose_y']
	p_yaw = df['pose_yaw']

	p_x2 = df2['pose_x']
	p_y2 = df2['pose_y']
	p_yaw2 = df2['pose_yaw']

	gt_x = df['gt_x']
	gt_y = df['gt_y']
	gt_yaw = df['gt_yaw']

	gmap = GMAP(gridmap, 0.05, [-13.9155, -10.99537, 0.0])


	for i in range(len(p_x)):
		p = [p_x[i], p_y[i]]
		gt = [gt_x[i], gt_y[i]]
		p2 = [p_x2[i], p_y2[i]]
		img = gridmap.copy()
		u, v = gmap.world2map(p)
		u2, v2 = gmap.world2map(gt)
		u3, v3 = gmap.world2map(p2)

		img = cv2.circle(img, (u, v), radius=1, color=(255, 0, 255), thickness=-1)
		img = cv2.circle(img, (u2, v2), radius=1, color=(255, 0, 0), thickness=-1)
		img = cv2.circle(img, (u3, v3), radius=1, color=(0, 255, 0), thickness=-1)
		cv2.imshow("", img)
		cv2.waitKey()




