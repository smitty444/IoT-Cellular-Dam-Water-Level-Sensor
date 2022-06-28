# -*- coding: utf-8 -*-
"""
Created on Fri Apr  1 11:45:46 2022

@author: SmithCA
"""

import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime
import pandas as pd
import pylab as pl
import numpy as np
import sys
np.set_printoptions(threshold=sys.maxsize)
plt.rcParams["font.family"] = "Times New Roman"
mpl.rcParams.update({'font.size': 14})

import IPython as IP
IP.get_ipython().magic('reset -sf')
plt.close('all')

df0 = pd.read_csv('usgs_stage.csv',parse_dates=     {"Datetime" : [2,3]})
df1 = pd.read_csv('stage_data.csv', parse_dates= {"Datetime" : [3,4]})
df2 = pd.read_csv('pressure_data.csv', parse_dates={"Datetime" : [3,4]})
df3 = pd.read_csv('stage_data_filtered.csv', parse_dates= {"Datetime" : [3,4]})
df4 = pd.read_csv('usgs_elevation.csv',parse_dates=     {"Datetime" : [2,3]})
df5 = pd.read_csv('pressure_data_filtered.csv', parse_dates={"Datetime" : [3,4]})

plt.figure(figsize=(6.5,3.5))
plt.plot(df0['Datetime'], df0['stage'], '-', label='USGS 02169000')
plt.plot(df1['Datetime'], df1['value'], '--', label='ultrasonic sensor')
plt.plot(df2['Datetime'], df2['value'], ':', label='pressure sensor')
plt.legend(framealpha=1)
plt.xlabel('datetime')
plt.ylabel('elevation (ft)')
plt.locator_params(axis="x", nbins=5)
plt.xticks(rotation=45)
plt.tight_layout()
plt.grid(True)
plt.savefig('data.png', dpi=500)

#%% With manual median filter

plt.figure(figsize=(6.5,3.5))
plt.plot(df0['Datetime'], df0['stage'], '-', label='USGS 02169000')
plt.plot(df3['Datetime'], df3['value'], '--', label='ultrasonic sensor')
plt.plot(df5['Datetime'], df5['value'], ':', label='pressure sensor')
plt.legend(framealpha=1)
plt.xlabel('datetime')
plt.ylabel('elevation (ft)')
plt.locator_params(axis="x", nbins=5)
plt.xticks(rotation=45)
plt.tight_layout()
plt.grid(True)
plt.savefig('data_filtered.png', dpi=500)

