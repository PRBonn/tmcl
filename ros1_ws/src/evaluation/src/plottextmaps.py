import numpy as np
import pandas as pd
import cv2
from evalutils import cam2BaselinkTF, angDist, EvalAllRuns, point2inch
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.patches as patches

from matplotlib import rcParams
from matplotlib.ticker import FormatStrFormatter
from GMAP import GMAP

rcParams['hatch.linewidth'] = 5.0
rcParams.update({'font.size': 7})
rcParams.update({'pdf.fonttype': 42})
# rcParams['text.usetex'] = True
linewidth_latex = 245.71811



if __name__ == "__main__":

	folderPath = "/home/nickybones/Code/YouBotMCL/ncore/data/floor/Faro/TextMaps/"
	resultsFolder = "/home/nickybones/Code/zimmerman2022iros/pics/"
	gridmap = cv2.imread("/home/nickybones/Code/YouBotMCL/ncore/data/floor/Faro/FMap.pgm")
	gmap = GMAP(gridmap, 0.05, [-13.9155, -10.99537, 0.0])
	roomnames = ["Room 1", "Room 2", "Room 3", "Room 4", "Room 5", "Room 6", "Room 7", "Room 8", "Room 9", "Room 10"]
	roomcoords = [[410, 50], [330, 50], [240, 50], [140, 50], [50, 50], [40, 220], [120, 220], [200, 220],[310, 220], [440, 220]]

	fig, ax = plt.subplots(figsize=(point2inch(linewidth_latex), 0.5 * point2inch(linewidth_latex)))
	clr = cm.rainbow(np.linspace(0, 1, 10))
	im = ax.imshow(gridmap)

	for i in range(1, 11):
		textmap = cv2.imread(folderPath + "Room " + str(i) + ".png")
		r = textmap[:, :, 0]
		locations = cv2.findNonZero(r)
		x,y,w,h = cv2.boundingRect(locations)
		for l in locations:
			# print(l)
			# print(l.shape)
			ax.plot(l[0, 0], l[0,1], 'o', alpha=0.8, markersize=1, color=clr[i-1])

		text_kwargs = dict(ha='center', va='center', fontsize=5, color=clr[i-1])	
		ax.text(roomcoords[i - 1][0], roomcoords[i - 1][1], roomnames[i - 1], **text_kwargs)

		#rect = patches.Rectangle((x, y), w, h, linewidth=1, edgecolor='k', facecolor='none', zorder=10000)
		#ax.add_patch(rect)
	ax.axis('off')
	# text_kwargs = dict(ha='center', va='center', fontsize=5, color='k')	
	# ax.text(410, 50, 'Room 1', **text_kwargs)
	# ax.text(330, 50, 'Room 2', **text_kwargs)
	# ax.text(240, 50, 'Room 3', **text_kwargs)
	# ax.text(140, 50, 'Room 4', **text_kwargs)
	# ax.text(50, 50, 'Room 5', **text_kwargs)

	# ax.text(440, 220, 'Room 10', **text_kwargs)
	# ax.text(310, 220, 'Room 9', **text_kwargs)
	# ax.text(200, 220, 'Room 8', **text_kwargs)
	# ax.text(120, 220, 'Room 7', **text_kwargs)
	# ax.text(40, 220, 'Room 6', **text_kwargs)

	plt.tight_layout(pad=0.0, rect=[0.01, 0.01, 0.99, 0.99])
	fig.savefig(resultsFolder + 'TextMaps.eps', format='eps')
	plt.show()


