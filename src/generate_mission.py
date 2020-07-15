# -*- coding: utf-8 -*-
"""
Created on Thu Nov 15 16:39:45 2018
Script to calculate GPS coordinates for entering into QGroundControl. Haversine formula accounts for distance "as the crow flies"
but we need 300 m radius from the AUT disregarding the curvature of earth. This script calculates an arc length that accounts for
that error and then calculates GPS coords using haversine and the new great circle distance. It also plots altitude/GPS info.
Reference for equations: https://www.movable-type.co.uk/scripts/latlong.html
The coordinates are given in three different arrays: latitude, longitude, and elevation. Starting from the first element, every 19
elements in each of the arrays correspond to one over-the-horizon pass beginning with theta = 0 (bearing). If there are errors with
the graphs, comment out the lines calling the TeX interpreter.
Wed May 1 2019: added 3D plot of spatial coverage, and converted coordinates from radians to degrees.
@author: Krishna Makhija
"""
#%%
import numpy as np
import math
import matplotlib.pyplot as plt
import csv
import os
import atexit

from matplotlib import rc
from mpl_toolkits.mplot3d import Axes3D

#rc('font',**{'family':'serif','serif':['Computer Modern']})
#rc('text', usetex=True)

Er = 6.3781e6    # radius of earth. Is there a more precise value?
# r = 300        # distance from AUT

# Begin Milton airfield test

home_lat = 37.99416837
home_lon = -78.39746091

r = 120 # Max radius allowed at Milton airfield

global latitude
global longitude
global elevation

latitude = None
longitude = None
elevation = None

def save_mission():
    lat_list = []
    lon_list = []
    alt_list = []
    for index in range(20, 37):
        lat_list.append(latitude[index])
        lon_list.append(longitude[index])
        alt_list.append(elevation[index])
    file_path = os.path.expanduser('~/catkin_ws/src/Drone-Nav/mission/waypoints.csv')
    with open(file_path, mode = 'w') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter = ',', quoting = csv.QUOTE_NONNUMERIC)
        for index in range(0, len(lat_list)):
            csv_writer.writerow([lat_list[index],
                                 lon_list[index],
                                 alt_list[index]])

atexit.register(save_mission)

# End Milton airfield test

theta = np.deg2rad(np.linspace(0, 180, 19))     # bearing
latStart = home_lat
longStart = home_lon
latStart = math.radians(latStart)
longStart = math.radians(longStart)

elevationAngle = np.deg2rad(np.linspace(0, 180, 19))
elevation = r*np.sin(elevationAngle)
d = r*np.cos(elevationAngle)

phiPath = np.arcsin(d/Er)
segmentPath = 2*np.pi*Er*phiPath/(2*np.pi)
deltaPath = segmentPath/Er

dp, t = np.meshgrid(deltaPath, theta)

latPath = (np.arcsin(np.sin(latStart)*np.cos(dp) + np.cos(latStart)*np.sin(dp)*np.cos(t)))
longPath = (longStart + np.arctan2(np.sin(t)*np.cos(latStart)*np.sin(dp), np.cos(dp) - np.sin(latStart)*np.sin(latPath)))

### Convert back to degrees. Is this more convenient?
latPath = np.rad2deg(latPath)
longPath = np.rad2deg(longPath)


longitude = longPath.flatten()
latitude = latPath.flatten('F')
elevation = np.tile(np.array(elevation), (1, 19)).flatten('F')


p = np.linspace(0, longitude.shape[0]-19, longitude.shape[0]/19)
p = [ int(x) for x in p ]

r = np.arange(1, len(p), 2)
r = [int(x) for x in r]

#### Arrange vectors in a more convenient "flow". This will set a sequence of coords such that the drone does not make back-and-forth
### passes across the reference point.
latBranch = [latitude[0: 19]]
longBranch = [longitude[0:19]]

for q in r:

    latBranch1 = np.flip(latitude[19*q: 19*q + 19], axis=0)
    latBranch0 = latitude[19*(q+1): 19*(q+1) + 19]
    latBranch.append(latBranch1)
    latBranch.append(latBranch0)

for q in r:

    longBranch1 = np.flip(longitude[19*q: 19*q + 19], axis=0)
    longBranch0 = longitude[19*(q+1): 19*(q+1) + 19]
    longBranch.append(longBranch1)
    longBranch.append(longBranch0)


plt.figure()
[plt.plot(latitude[n:n+18], longitude[n:n+18], marker='.') for n in p]
plt.xlabel("Latitude (degrees)")
plt.ylabel("Longitude (degrees)")
plt.title("Flight trajectories on lat/long plane")
plt.grid()
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
[ax.plot(latitude[n:n+19], longitude[n:n+19], elevation[n:n+19], marker='o', linestyle=':') for n in p]
fig.suptitle('Spatial coverage with 10 degrees step in El/Az', fontsize=18)
ax.set_xlabel('Latitude (degrees)', fontsize=12)
ax.set_ylabel('Longitude (degrees)', fontsize=12)
ax.set_zlabel('Elevation (m)', fontsize=12)
plt.tick_params(labelsize=12)
plt.show()
