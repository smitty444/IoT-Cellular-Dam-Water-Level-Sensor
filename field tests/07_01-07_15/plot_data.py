# -*- coding: utf-8 -*-
"""
Created on Fri Apr  1 11:45:46 2022

@author: SmithCA
"""

import matplotlib as mpl
import matplotlib.pyplot as plt
#import matplotlib.dates as mdates
from datetime import datetime
import pandas as pd
import pylab as pl
import numpy as np
import sys
np.set_printoptions(threshold=sys.maxsize)
plt.rcParams["font.family"] = "Times New Roman"
mpl.rcParams.update({'font.size': 12})

import IPython as IP
IP.get_ipython().magic('reset -sf')
plt.close('all')

df1 = pd.read_csv('stage_data.csv', parse_dates= {"Datetime" : [4,5]})
df2 = pd.read_csv('pressure_data.csv', parse_dates={"Datetime" : [4,5]})
df3 = pd.read_csv('stage_data_filtered.csv', parse_dates= {"Datetime" : [4,5]})
df4 = pd.read_csv('rainfall.csv', parse_dates=['date'])
#df5 = pd.read_csv('pressure_data_filtered.csv', parse_dates={"Datetime" : [3,4]})

plt.figure(figsize=(6.5,3.5))
plt.plot(df3['Datetime'], df3['stage'], '--', label='ultrasonic sensor')
plt.plot(df2['Datetime'], df2['stage'], ':', label='pressure sensor')
#plt.plot(df6['Datetime'], df6['Sensor depth with corrected elevation'], ".-", label = "HOBO sensor")
plt.legend(framealpha=1)
plt.xlabel('datetime')
plt.ylabel('elevation (ft)')
plt.locator_params(axis="x", nbins=5)
plt.xticks(rotation=45)
plt.tight_layout()
plt.grid(True)
#plt.savefig('data.png', dpi=500)

#%% With manual median filter

plt.figure(figsize=(6.5,3.5))
plt.plot(df3['Datetime'], df3['stage'], '--', color="orange", label='ultrasonic sensor')
plt.plot(df2['Datetime'], df2['elevation'], ':', color = "green", label='pressure transducer')
plt.legend(loc="lower right")
plt.xlabel('datetime')
plt.ylabel('elevation (ft)')
plt.locator_params(axis="x", nbins=5)
plt.xticks(rotation=45)
plt.tight_layout()
plt.grid(True)
#plt.savefig('data_filtered.png', dpi=500)

#%% rainfall subplot

fig, a = plt.subplots(figsize=(6,4))

a.plot(df3['Datetime'], df3['stage'], '--', color="orange", label='ultrasonic sensor')
a.plot(df2['Datetime'], df2['elevation'], ':', color = "green", label='pressure transducer')
a.set_ylabel('elevation (ft)')
a.set_xlabel('datetime')
a.tick_params('x', rotation=45)
plt.legend(framealpha=1)

a2 = a.twinx()
a2.bar(df4['date'], df4['rainfall (in)'], alpha=0.5, align='edge', width=1.0, label='rainfall')
a2.set_ylabel('rainfall (in)')

plt.legend(framealpha=1)

# plt.xlabel('datetime')
# a.ylabel('elevation (ft)')
# a2.ylabel('rainfall (in)')
plt.locator_params(axis="x", nbins=5)
#plt.xticks(rotation=45)
plt.tight_layout()
plt.grid(True)
plt.savefig('rainfall.png', dpi=500)