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
#mpl.rcParams.update({'font.size': 14})

import IPython as IP
IP.get_ipython().magic('reset -sf')
plt.close('all')


filter_df = pd.read_csv('adaptive_simulations.csv')
x = np.array(range(5,102))

"""
Explanation of column headers: 
    
adaptive: median filter with the sigmoid function ranging from 20 to 4, responsive x values mapped from 0.01 - 1
adaptive_2: median filter with the sigmoid function ranging from 20 to 4, responsive x values mapped from 0.01 - 0.7
adaptive_3: last replacement filter with the sigmoid function ranging from 20 to 4, responsive x values mapped from 0.01 - 0.7
adaptive_4: last replacement filter with the original adaptive sampling function: timeInterval = (1-avgSecondDerivs)*5
adaptive_5: last replacement filter with the sigmoid function ranging from 20 to 4, responsive x values mapped from 0.01 - 0.52
adaptive_6: last replacement filter with the sigmoid function ranging from 20 to 4, responsive x values mapped from 0.01 - 0.52, first derivatives
adaptive_7: last replacement filter with the sigmoid function ranging from 20 to 4, responsive x values mapped from 0.1 - 0.5, first derivatives

conclusion: adaptive_5 is best

"""


#%% comparison of filter techniques

plt.figure(figsize=(6.5,3.5))
plt.plot(x, filter_df["true signal"], marker = "o", label="true signal")
#plt.plot(x, filter_df["adaptive"],"--", marker="o",  label = "adaptive sampling")
#plt.plot(x, filter_df["adaptive_2"],"--", marker="o",  label = "adaptive sampling")
#plt.plot(filter_df["adaptive_3_index"], filter_df["adaptive_3"],"--", marker="v",  label = "reverse sigmoid (0.01 - 0.7)")
plt.plot(filter_df["adaptive_5_index"], filter_df["adaptive_5"], ":", marker = "v", label = "reverse sigmoid (0.01 - 0.52)")
#plt.plot(filter_df["adaptive_4_index"], filter_df["adaptive_4"],":", marker = "x", label = "old algorithm")
#plt.plot(filter_df["adaptive_6_index"], filter_df["adaptive_6"],":", marker = "x", label = "reverse sigmoid first derivs")
plt.plot(filter_df["adaptive_7_index"], filter_df["adaptive_7"],":", marker = "x", label = "reverse sigmoid first derivs")
plt.legend(framealpha=1)
plt.xlabel('recordings')
plt.ylabel('distance (ft)')
#plt.locator_params(axis="x", nbins=5)
#plt.xticks(rotation=45)
plt.tight_layout()
plt.grid(True)
#plt.savefig('algorithm_comparison.png', dpi=500)
#plt.savefig('sigmoid_comparison.png', dpi=500)
plt.savefig('viewing.png', dpi=500)
