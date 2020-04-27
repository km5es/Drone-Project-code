#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 15 16:39:45 2018
Modified Oct 25th 2019 for functionality to create WP file.

Script to calculate GPS coordinates for entering into QGroundControl. Haversine formula accounts for distance "as the crow flies" 
but we need 300 m radius from the AUT disregarding the curvature of earth. This script calculates an arc length that accounts for 
that error and then calculates GPS coords using haversine and the new great circle distance. It also plots altitude/GPS info.
Reference for equations: https://www.movable-type.co.uk/scripts/latlong.html

The coordinates are given in three different arrays: latitude, longitude, and elevation. Starting from the first element, every 19 
elements in each of the arrays correspond to one over-the-horizon pass beginning with theta = 0 (bearing). If there are errors with
the graphs, comment out the lines calling the TeX interpreter. 

To fly one pass at Milton latmilton and longmilton are the vectors to use for spanning the entire runway.

Then, lat_wp, long_wp, and el_wp are vectors for dumping into a .waypoints file. This file can be loaded to a FC via a GCS to effect
a complete hemispherical flight.

TODO: Specify starting (home) position and RTL waypoint.
    FIXME: Test out new RTL and home WPs. Also added hold parameter in argparse.
        FIXME: QGC/PX4 does not understand RTL? Only viable option seems to be Land (21). Look into this later. Change the final WP to be
        Land (21) at starting position.
            FIXME: Final land point should NOT be reference position as that is the AUT!!
TODO: Add parameter for constraining speed for the entire flight.
TODO: Create a way to generate specific flight paths, for e.g. sorties 4-6.
TODO: (optional) write code to plot straight to Google Maps in 3D.


@author: Krishna Makhija
"""
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import rc
from mpl_toolkits.mplot3d import Axes3D
import argparse
from pathlib import Path
from os.path import expanduser
#import gmplot

#rc('font',**{'family':'serif','serif':['Computer Modern']})
#rc('text', usetex=True)

#############################################
################# Define starting variables
#############################################
Er = 6.3781e6  # radius of earth. Is there a more precise value?
radius = 60        # distance from AUT  
heading = 0
theta = np.deg2rad(np.linspace(0, 180, 19))     # bearing
latStart = 37.994125                          
longStart = -78.397535                        # Milton Airfield
missionStart_alt = 15
wp_hold_time = 0
passes = 18

parser = argparse.ArgumentParser(description='Generate a flight path for beam mapping with waypoints at every 10 degrees in theta/phi. The WP are outputted in a file titled hemisphere_waypoints.waypoints')
parser.add_argument('--radius', type=float, help='radius of each pass over-the-horizon. Default = 60m')
parser.add_argument('--lat', type=float, help='Latitude of reference point (AUT). Default = Milton tarmac.')
parser.add_argument('--long', type=float, help='Longitude of reference point (AUT). Default = Milton tarmac.')
parser.add_argument('--heading', type=int, help='Heading to be maintained throughout flight. Default = 0 degrees.')
parser.add_argument('--hold', type=int, help='Time to hold at each waypoint in seconds. Default = 0 seconds.')
parser.add_argument('--passes', type=int, help='Number of passes to generate. Default = 18.')
args = parser.parse_args()

if args.radius:
    radius=args.radius
    print("The radius of each over-the-horizon path is %s m." %radius)

if args.lat:
    latStart = args.lat
    print("The reference position is at %s degrees latitude." %args.lat)

if args.long:
    longStart = args.long
    print("The reference position is at %s degrees longitude." %args.long)

if args.heading:
    heading = args.heading
    print("The heading will be maintained at %s degrees." %args.heading)

if args.hold:
    wp_hold_time = args.hold
    print("The hold time at each waypoint will be %s seconds." %args.hold)

if args.passes:
    passes = args.passes
    print("The flight path will have %s passes." %args.passes)

latStart = math.radians(latStart)
longStart = math.radians(longStart)
elevationAngle = np.deg2rad(np.linspace(0, 180, 19))
elevation = radius*np.sin(elevationAngle)
d = radius*np.cos(elevationAngle)

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
    

### Used for plotting waypoints on https://www.easymapmaker.com/
latcircle = []
for n in p:
    latcircle.append(latitude[n])
    latcircle.append(latitude[n+18])
    
longcircle = []
for n in p:
    longcircle.append(longitude[n])
    longcircle.append(longitude[n+18])

######### Milton one pass
latmilton = latBranch[1]
longmilton = longBranch[1]

#######################################
##### Create a .waypoints file ########
#######################################
### new vectors for actual flight path
lat_wp = []
for n in range(len(latBranch)-1):
    lat_wp.append(latBranch[n][1:18])
lat_wp = np.concatenate(lat_wp)

long_wp = []
for n in range(len(longBranch)-1):
    long_wp.append(longBranch[n][1:18])
long_wp = np.concatenate(long_wp)

elBranch = elevation[1:18]
el_wp = np.tile(elBranch, 18)

#h1 = np.arange(10, 190, 20)
#h2 = np.arange(180, 360, 20)
#heading = np.vstack((h2,h1)).reshape((-1,),order='F')
#heading = np.repeat(heading, 17)
#np.repeat(0, 306)

#f_sphere = open("hemisphere_waypoints.waypoints","w+")
#home = str(Path.home())    # for python3
home = expanduser("~")
f_sphere = open(home + "/" + str(int(radius)) + "m_" + str(wp_hold_time) + "s_" + str(passes) + "passes.waypoints", "w+")
f_sphere.write("QGC WPL 110\n")
### Define Mission Start point:
f_sphere.write("0\t1\t0\t16\t0\t0\t0\t")
f_sphere.write(str(heading))
f_sphere.write("\t")
f_sphere.write(str(math.degrees(latStart)))
f_sphere.write("\t")
f_sphere.write(str(math.degrees(longStart)))
f_sphere.write("\t")
f_sphere.write(str(missionStart_alt))
f_sphere.write("\t1\n")
### Define waypoints for sampling data
for i in range(len(elBranch)*passes):    ### this range is chosen to ignore waypoints at theta = 0 and 180
    f_sphere.write(str(i+1))
    f_sphere.write("\t0\t3\t16\t")
    f_sphere.write(str(wp_hold_time)+"\t0\t0\t")
    f_sphere.write(str(heading))
    f_sphere.write("\t")
    f_sphere.write(str(lat_wp[i]))
    f_sphere.write("\t")
    f_sphere.write(str(long_wp[i]))
    f_sphere.write("\t")
    f_sphere.write(str(el_wp[i]))
    f_sphere.write("\t1\n")
### Define Land (21)
f_sphere.write(str(len(long_wp)+1))
f_sphere.write("\t0\t3\t21\t0\t0\t0\t")
f_sphere.write(str(heading))
f_sphere.write("\t")
f_sphere.write(str(math.degrees(latStart)))
f_sphere.write("\t")
f_sphere.write(str(math.degrees(longStart)))
f_sphere.write("\t")
f_sphere.write(str(0))
f_sphere.write("\t1\n")
f_sphere.close()


#############################################
### Making plots for illustration and thesis?
#############################################

plt.figure()
[plt.plot(latitude[n:n+18], longitude[n:n+18]) for n in p]
plt.xlabel("Latitude (degrees)")
plt.ylabel("Longitude (degrees)")
plt.title("Flight trajectories on lat/long plane")
plt.grid()
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
[ax.plot(latitude[n:n+19], longitude[n:n+19], elevation[n:n+19], marker='o', linestyle=':') for n in p]
#fig.suptitle('Spatial coverage with 10 degrees step in El/Az', fontsize=18)
ax.set_xlabel('Latitude (degrees)', fontsize=14, labelpad=15)
ax.set_ylabel('Longitude (degrees)', fontsize=14, labelpad=15)
ax.set_zlabel('Elevation (m)', fontsize=14)
plt.tick_params(labelsize=12)
plt.show()

"""
########### Plot on Google Maps
gmap3 = gmplot.GoogleMapPlotter(latStart, longStart, 13)
gmap3.scatter( lat_wp, long_wp, '# FF0000', size = 40, marker = False ) 

# Plot method Draw a line in 
# between given coordinates 
gmap3.plot(lat_wp, long_wp, 'cornflowerblue', edge_width = 2.5) 

#gmap3.draw( "C:\\Users\\user\\Desktop\\map13.html" ) 
gmap3.draw("./map13.html")
"""