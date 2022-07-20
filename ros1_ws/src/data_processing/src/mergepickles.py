import numpy as np
import pandas as pd


dumpFolder = "/home/nickybones/data/MCL/Dump/"
pickleName = "merged.pickle"

df_s = pd.read_pickle(dumpFolder + "scans.pickle")
df_o = pd.read_pickle(dumpFolder + "odom.pickle")
df_c0 = pd.read_pickle(dumpFolder + "camera0.pickle")
df_c1 = pd.read_pickle(dumpFolder + "camera1.pickle")
df_c2 = pd.read_pickle(dumpFolder + "camera2.pickle")
df_c3 = pd.read_pickle(dumpFolder + "camera3.pickle")
df_gt = pd.read_pickle(dumpFolder + "gopro.pickle")

dfcomb = pd.concat([df_s,df_o, df_c0, df_c1, df_c2, df_c3, df_gt], axis=0)

dfcomb['t'] = dfcomb['t'].astype(int)

dfcomb = dfcomb.sort_values(by='t')
#print(dfcomb.head(20).to_string(index=False))
print((dfcomb['type']== "camera0").sum())
print((dfcomb['type']== "camera1").sum())
print((dfcomb['type']== "camera2").sum())
print((dfcomb['type']== "camera3").sum())
print((dfcomb['type']== "gopro").sum())

dfcomb.to_pickle(dumpFolder + pickleName)