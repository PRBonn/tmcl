import numpy as np
import pandas as pd


df = pd.read_pickle("/home/nickybones/data/MCL/2021_12_07/Run4/textgtmerged.pickle")

for index, row in df.iterrows():

	typ = row['type']
	if "sem3" in typ:
		t = row['t']
		boxes = row['data']
		print(typ, t)
		print(boxes)

		for i in range(len(boxes)):
			if boxes[i] < 0 or boxes[i] > 639:
				print(typ, t)
				print(boxes)