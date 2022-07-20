import numpy as np
import pandas as pd
import math




def addText(dumpFolder):

	matches = []
	text_t = []
	types = []

	for c in range(4):
		df_cam = pd.read_pickle(dumpFolder + "camera" + str(c) + ".pickle")
		textFile = dumpFolder + "predictions" + str(c) + ".txt"
		f = open(textFile, "r")
		lines = f.readlines()

		for i in range(len(lines)):
			matches.append(lines[i])
			row = df_cam.iloc[i]
			text_t.append(row['t'])
			types.append('text' + str(c))


	data = {'t': text_t, 'type': types, 'data': matches}
	df = pd.DataFrame(data)

	return df


def addSemantics(dumpFolder):

	df = pd.read_pickle(dumpFolder + "person.pickle")


	# t = df['t']
	# types = df['type']
	# seman = []

	# for index, row in df.iterrows():
	# 	sem = row['boxes']
	# 	sem = np.array(sem).flatten().astype(float)
	# 	seman.append(sem)

	# data = {'t': t, 'type': types, 'data': seman}
	# new_df = pd.DataFrame(data)

	# new_df.to_pickle(dumpFolder + "person2.pickle")

	# return new_df
	return df


if __name__ == "__main__":

	dumpFolder = "/home/nickybones/data/MCL/2021_12_08/Run3/"
	pickleFolder = "/home/nickybones/data/MCL/2021_12_08/Run3/"
	pickleName = pickleFolder + "gtmerged.pickle"

	df_merged = pd.read_pickle(pickleName)

	df_text = addText(dumpFolder)
	df_sem = addSemantics(dumpFolder)



	dfcomb = pd.concat([df_text, df_sem, df_merged], axis=0)
	dfcomb['t'] = dfcomb['t'].astype(int)
	dfcomb = dfcomb.sort_values(by='t')
	dfcomb.to_pickle(dumpFolder + "semtextgtmerged.pickle")

	dfcomb = pd.concat([df_text, df_merged], axis=0)
	dfcomb['t'] = dfcomb['t'].astype(int)
	dfcomb = dfcomb.sort_values(by='t')
	dfcomb.to_pickle(dumpFolder + "textgtmerged.pickle")

	dfcomb = pd.concat([df_sem, df_merged], axis=0)
	dfcomb['t'] = dfcomb['t'].astype(int)
	dfcomb = dfcomb.sort_values(by='t')
	dfcomb.to_pickle(dumpFolder + "semgtmerged.pickle")



	#df_sem = addSemantics(dumpFolder)
	#df_sem.to_pickle(dumpFolder + "person2.pickle")


		