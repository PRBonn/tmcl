
import numpy as np
import cv2
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches



def prepareSem(data):

	sem = np.array(data)
	sem = sem.reshape((-1, 4))

	return sem

def drawBox(img, sem, id):

	if sem is not None:
		fig, ax = plt.subplots()
		ax.imshow(img)
		for i in range(sem.shape[0]):
			box = sem[i]
			rect = patches.Rectangle((box[0], box[1]), box[2]-box[0], box[3]-box[1], linewidth=1, edgecolor='r', facecolor='none')
			ax.add_patch(rect)
		plt.show()
	if sem is not None:
		if sem.shape[0] == 0:
			print("no detection, camera {}".format(id))




if __name__ == "__main__":
	picklePath = "/home/nickybones/data/MCL/2021_12_08/Run2/textgtmerged.pickle"
	imgFolder = "/home/nickybones/data/MCL/2021_12_08/Run2/camera"
	df = pd.read_pickle(picklePath)

	img = None
	sem0 = None
	sem1 = None
	sem2 = None
	sem3 = None

	cnt = 0

	for index, row in df.iterrows():

		if cnt < 5500:
			cnt += 1
			continue

		if row['type'] == 'sem0':
			sem0 = prepareSem(row['data'])
		
		elif row['type'] == 'sem1':
			sem1 = prepareSem(row['data'])

		elif row['type'] == 'sem2':
			sem2 = prepareSem(row['data'])

		elif row['type'] == 'sem3':
			sem3 = prepareSem(row['data'])


		elif row['type'] == 'camera0':
			img_name = row['data']
			img = cv2.imread(imgFolder + "0/" + img_name)
			drawBox(img, sem0, 0)
				# fig, ax = plt.subplots()
				# ax.imshow(img)
				# #print("camera0 " + str(row['t']))
				# for i in range(sem0.shape[0]):
				# 	box = sem0[i]
				# 	print(box)
				# 	print(img_name)

				# 	rect = patches.Rectangle((box[0], box[1]), box[2]-box[0], box[3]-box[1], linewidth=1, edgecolor='r', facecolor='none')
				# 	ax.add_patch(rect)
				# plt.show()
		
		elif row['type'] == 'camera1':
			img_name = row['data']
			img = cv2.imread(imgFolder + "1/" + img_name)
			drawBox(img, sem1, 1)


		elif row['type'] == 'camera2':
			img_name = row['data']
			img = cv2.imread(imgFolder + "2/" + img_name)
			drawBox(img, sem2, 2)

		elif row['type'] == 'camera3':
			img_name = row['data']
			img = cv2.imread(imgFolder + "3/" + img_name)
			drawBox(img, sem3, 3)


		cnt += 1

		#print(row['type'] + " " + str(row['t']))
		#print(row['data'])


	# elif row['type'] == 'camera0':
	# 	img_name = row['data']
	# 	img = cv2.imread(imgFolder + "0/" + img_name)

	# 	if sem0 is not None and sem0.shape[0]:
	# 		fig, ax = plt.subplots()
	# 		ax.imshow(img)
	# 		#print("camera0 " + str(row['t']))
	# 		for i in range(sem0.shape[0]):
	# 			box = sem0[i]
	# 			print(box)
	# 			print(img_name)

	# 			rect = patches.Rectangle((box[0], box[1]), box[2]-box[0], box[3]-box[1], linewidth=1, edgecolor='r', facecolor='none')
	# 			ax.add_patch(rect)
	# 		plt.show()

	# elif row['type'] == 'camera1':
	# 	img_name = row['data']
	# 	img = cv2.imread(imgFolder + "1/" + img_name)

	# 	if sem1 is not None and sem1.shape[0]:
	# 		fig, ax = plt.subplots()
	# 		ax.imshow(img)
	# 		#print("camera0 " + str(row['t']))
	# 		for i in range(sem1.shape[0]):
	# 			box = sem1[i]
	# 			print(box)
	# 			print(img_name)

	# 			rect = patches.Rectangle((box[0], box[1]), box[2]-box[0], box[3]-box[1], linewidth=1, edgecolor='r', facecolor='none')
	# 			ax.add_patch(rect)
	# 		plt.show()



