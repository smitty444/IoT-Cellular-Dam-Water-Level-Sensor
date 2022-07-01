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
df6 = pd.read_csv('hobo_data.csv', parse_dates = {"Datetime" : [0]})

plt.figure(figsize=(6.5,3.5))
plt.plot(df0['Datetime'], df0['stage'], '-', label='USGS 02169000')
plt.plot(df1['Datetime'], df1['value'], '--', label='ultrasonic sensor')
plt.plot(df2['Datetime'], df2['value'], ':', label='pressure sensor')
#plt.plot(df6['Datetime'], df6['Sensor depth with corrected elevation'], ".-", label = "HOBO sensor")
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
plt.plot(df5['Datetime'], df5['elevation'], ':', label='pressure sensor')
plt.plot(df6['Datetime'], df6['Sensor depth with corrected elevation'], "-.", label = "HOBO sensor")
#plt.legend(framealpha=1)
plt.xlabel('datetime')
plt.ylabel('elevation (ft)')
plt.locator_params(axis="x", nbins=5)
plt.xticks(rotation=45)
plt.tight_layout()
plt.grid(True)
#plt.savefig('data_filtered.png', dpi=500)


#%% Comparing raw water HOBO pressure with pressure sensor

fig,a = plt.subplots(figsize=(6.5,4))
#a.plot(df0['Datetime'], df0['stage'], '-', label='USGS stage', linewidth=0.75)
#plt.plot(df1['Datetime'], df1['value'], '--', label='ultrasonic sensor', linewidth=0.75)
a.plot(df5['Datetime'], df5['value'], ':', color = 'green', label='pressure sensor', linewidth=0.75)
a2 = a.twinx()
a2.plot(plt.plot(df6['Datetime'], df6['Abs Pres, psi (LGR S/N: 1181041, SEN S/N: 1181041)'], "-.", color = 'red', label = "HOBO sensor", linewidth=0.75))

plt.legend(framealpha=1)
plt.xlabel('datetime')
a.ylabel('elevation (ft)')
a2.ylabel('pressure (psi)')
plt.locator_params(axis="x", nbins=5)
plt.xticks(rotation=45)
plt.tight_layout()
plt.grid(True)
#plt.savefig('pressure.png', dpi=500)