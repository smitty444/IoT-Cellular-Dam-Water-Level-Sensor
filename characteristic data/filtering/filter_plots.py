# -*- coding: utf-8 -*-
"""
Created on Mon Apr 18 11:41:08 2022

@author: SmithCA
"""

import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime
import pandas as pd
import pylab as pl
import numpy as np
plt.rcParams["font.family"] = "Times New Roman"
mpl.rcParams.update({'font.size': 16})

import IPython as IP
IP.get_ipython().magic('reset -sf')
plt.close('all')


filter_df = pd.read_csv('filters.csv')
x = np.array(range(0,97))

#%% Comparing filters
plt.figure(figsize=(6.5,3.7))
plt.plot(x, filter_df["true signal"], label="true signal")
plt.plot(x, filter_df["kalman"], ":", label="Kalman filter")
plt.plot(x, filter_df["moving average"], "-.",  label="moving average filter")
plt.plot(x, filter_df["median"], "--", label="median filter")
#plt.plot(x, filter_df["last replacement"],":",  label = "last replacement filter")
plt.legend(framealpha=1)
plt.xlabel('recordings')
plt.ylabel('distance (ft)')
#plt.locator_params(axis="x", nbins=5)
#plt.xticks(rotation=45)
plt.tight_layout()
plt.grid(True)
plt.savefig('filters.png', dpi=500)

