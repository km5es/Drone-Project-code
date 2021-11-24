#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 15 16:39:45 2018
Modified 3rd Jan 2021 for firmware specific flight paths.

Script to calculate GPS coordinates for entering into QGroundControl. Haversine formula accounts for distance "as the crow flies" 
but we need a fixed radius from the AUT disregarding the curvature of earth. This script calculates an arc length that accounts for 
that error and then calculates GPS coords using haversine and the new great circle distance. It also plots altitude/GPS info.
Reference for equations: https://www.movable-type.co.uk/scripts/latlong.html

The coordinates are given in three different arrays: latitude, longitude, and elevation. Starting from the first element, every 19 
elements in each of the arrays correspond to one over-the-horizon pass beginning with theta = 0 (bearing). 

Then, lat_wp, long_wp, and el_wp are vectors for dumping into a .waypoints file. This file can be loaded to a FC via a GCS to effect
a complete hemispherical flight.

TODO: Create a way to generate specific flight paths, for e.g. sorties 4-6.
TODO: add parameter to change spacing between WPs so we can have finer beam patterns
    FIXME: this seems to work but first WP starts at 0 deg. Make it so it starts at 10 deg.

@author: Krishna Makhija
"""
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
from os.path import expanduser
from termcolor import colored


####* Default values
Er                  = 6.3781e6           # radius of earth. Is there a more precise value?
radius              = 60                 # distance from AUT  
heading             = 0
latStart            = 37.994125          # Milton airfield
longStart           = -78.397535         #       coords
missionStart_alt    = 10
wp_hold_time        = 0
passes              = 18
firmware            = "Ardupilot"
spacing             = 10                 # default spacing is 10 deg


####* Set up argparser
parser = argparse.ArgumentParser(description='Generate a flight plan for beam-mapping with PRIAM. The grid will be hemispherical' + 
                                 ' shaped and centered around a reference point (the AUT). If run on the payload computer ' +
                                 'this will update the ROS mission.')
parser.add_argument('--radius', '-r', type=float, help='radius of each pass over-the-horizon. Default = 60m')
parser.add_argument('--lat', '-la', type=float, help='Latitude of reference point (AUT). Default = Milton tarmac.')
parser.add_argument('--long', '-lo', type=float, help='Longitude of reference point (AUT). Default = Milton tarmac.')
parser.add_argument('--heading', '-he', type=int, help='Heading to be maintained throughout flight. Default = 0 degrees.')
parser.add_argument('--hold', '-ho', type=int, help='Time to hold at each waypoint in seconds. Default = 0 seconds.')
parser.add_argument('--passes', '-p', type=int, help='Number of passes to generate. Default = 18.')
parser.add_argument('--firmware', '-f', type=str, help='Choose PX4 or Ardupilot.')
parser.add_argument('--spacing', '-s', type=float, help='Angular spacing between each WP in one pass. Default = 10 degrees.')
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

if args.firmware:
    firmware = args.firmware
    print("The flight path will be generated for autopilots using %s firmware. Default = Ardupilot" %args.firmware)

if args.spacing:
    spacing = args.spacing
    print("Each WP will be spaced %s degrees apart." %args.spacing)


####* array setup
spacing             = int(180.0/spacing + 1)       # no. of points in the theta/phi arrays
latStart            = math.radians(latStart)
longStart           = math.radians(longStart)
theta               = np.deg2rad(np.linspace(0, 180, spacing))
elevationAngle      = np.deg2rad(np.linspace(0, 180, spacing))


####* Calculate waypoints
elevation           = radius*np.sin(elevationAngle)
d                   = radius*np.cos(elevationAngle)    
phiPath             = np.arcsin(d/Er)
segmentPath         = 2*np.pi*Er*phiPath/(2*np.pi)
deltaPath           = segmentPath/Er

dp, t               = np.meshgrid(deltaPath, theta)

latPath             = (np.arcsin(np.sin(latStart)*np.cos(dp) + 
                         np.cos(latStart)*np.sin(dp)*np.cos(t)))
longPath            = (longStart + np.arctan2(np.sin(t)*np.cos(latStart)*np.sin(dp), 
                                      np.cos(dp) - np.sin(latStart)*np.sin(latPath)))

# Convert back to degrees.
latPath             = np.rad2deg(latPath)
longPath            = np.rad2deg(longPath)

longitude           = longPath.flatten()
latitude            = latPath.flatten('F')
elevation           = np.tile(np.array(elevation), (1, spacing)).flatten('F')

p = np.linspace(0, longitude.shape[0]-spacing, longitude.shape[0]//spacing)
p = [ int(x) for x in p ]

r = np.arange(1, len(p), 2)
r = [int(x) for x in r]


####* Arrange vectors in a more convenient "flow". This will set a sequence of coords 
####* such that the drone does not make back-and-forth
####* passes across the reference point.
latBranch           = [latitude[0:spacing]]
longBranch          = [longitude[0:spacing]]

for q in r:
    latBranch1 = np.flip(latitude[spacing*q: spacing*q + spacing], axis=0)
    latBranch0 = latitude[spacing*(q+1): spacing*(q+1) + spacing]
    latBranch.append(latBranch1)
    latBranch.append(latBranch0)

for q in r:
    longBranch1 = np.flip(longitude[spacing*q: spacing*q + spacing], axis=0)
    longBranch0 = longitude[spacing*(q+1): spacing*(q+1) + spacing]
    longBranch.append(longBranch1)
    longBranch.append(longBranch0)
    
'''
### Used for plotting waypoints on https://www.easymapmaker.com/
latcircle = []
for n in p:
    latcircle.append(latitude[n])
    latcircle.append(latitude[n+(spacing - 1)])
    
longcircle = []
for n in p:
    longcircle.append(longitude[n])
    longcircle.append(longitude[n+(spacing - 1)])

######### Milton one pass
latmilton = latBranch[1]
longmilton = longBranch[1]
'''


####* Making plots for illustration
plt.figure()
[plt.plot(latitude[n:n+(spacing-1)], longitude[n:n+(spacing-1)]) for n in p]
plt.xlabel("Latitude (degrees)")
plt.ylabel("Longitude (degrees)")
plt.title("Flight trajectories on lat/long plane")
plt.grid()
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
[ax.plot(latitude[n:n+spacing], longitude[n:n+spacing], elevation[n:n+spacing], marker='o', linestyle=':') for n in p]
#fig.suptitle('Spatial coverage with 10 degrees step in El/Az', fontsize=18)
ax.set_xlabel('Latitude (degrees)', fontsize=14, labelpad=15)
ax.set_ylabel('Longitude (degrees)', fontsize=14, labelpad=15)
ax.set_zlabel('Elevation (m)', fontsize=14)
plt.tick_params(labelsize=12)
plt.show()


####* Create a .waypoints file
# new vectors for actual flight path
lat_wp      = []
for n in range(len(latBranch)-1):
    lat_wp.append(latBranch[n][1:(spacing-1)])
lat_wp      = np.concatenate(lat_wp)

long_wp     = []
for n in range(len(longBranch)-1):
    long_wp.append(longBranch[n][1:(spacing-1)])
long_wp     = np.concatenate(long_wp)

elBranch    = elevation[1:(spacing-1)]
el_wp       = np.tile(elBranch, (spacing-1))

home        = expanduser("~")
path        = home + '/catkin_ws/src/Drone-Project-code/mission/'
f_sphere    = open(path + str(int(radius)) + "m_" + 
                    str(wp_hold_time) + "s_" + str(passes) + "passes_" + 
                            str(int(180/(spacing-1))) + "deg_" + str(firmware) + ".waypoints", "w+")
mission_f   = open(path + "mission.waypoints", "w+")      ## actual mission file used by write_WPs.py
f_sphere.write("QGC WPL 110\n")
mission_f.write("QGC WPL 110\n")

def home_pos():
    """
    Create a home position.
    """
    f_sphere.write("0\t1\t0\t16\t0\t0\t0\t")
    f_sphere.write(str(heading))
    f_sphere.write("\t")
    f_sphere.write(str(math.degrees(latStart)))
    f_sphere.write("\t")
    f_sphere.write(str(math.degrees(longStart)))
    f_sphere.write("\t")
    f_sphere.write(str(missionStart_alt))
    f_sphere.write("\t1\n")
    mission_f.write("0\t1\t0\t16\t0\t0\t0\t")
    mission_f.write(str(heading))
    mission_f.write("\t")
    mission_f.write(str(math.degrees(latStart)))
    mission_f.write("\t")
    mission_f.write(str(math.degrees(longStart)))
    mission_f.write("\t")
    mission_f.write(str(missionStart_alt))
    mission_f.write("\t1\n")

def takeoff():
    """
    Create a takeoff waypoint if firmware is chosen to be Ardupilot.
    """
    f_sphere.write("1\t0\t0\t22\t0\t0\t0\t")
    f_sphere.write(str(heading))
    f_sphere.write("\t")
    f_sphere.write(str(math.degrees(latStart)))
    f_sphere.write("\t")
    f_sphere.write(str(math.degrees(longStart)))
    f_sphere.write("\t")
    f_sphere.write(str(missionStart_alt))
    f_sphere.write("\t1\n")
    mission_f.write("1\t0\t0\t22\t0\t0\t0\t")
    mission_f.write(str(heading))
    mission_f.write("\t")
    mission_f.write(str(math.degrees(latStart)))
    mission_f.write("\t")
    mission_f.write(str(math.degrees(longStart)))
    mission_f.write("\t")
    mission_f.write(str(missionStart_alt))
    mission_f.write("\t1\n")

def yaw(start_index):
    """
    Add a yaw correction for each waypoint. Ardupilot only.
    """
    #f_sphere.write(str(start_index + 2))
    f_sphere.write("\t0\t3\t115\t")
    f_sphere.write(str(heading))
    f_sphere.write("\t0\t0\t0\t")
    f_sphere.write(str(lat_wp[start_index]))
    f_sphere.write("\t")
    f_sphere.write(str(long_wp[start_index]))
    f_sphere.write("\t")
    f_sphere.write(str(el_wp[start_index]))
    f_sphere.write("\t1\n")
    mission_f.write("\t0\t3\t115\t")
    mission_f.write(str(heading))
    mission_f.write("\t0\t0\t0\t")
    mission_f.write(str(lat_wp[start_index]))
    mission_f.write("\t")
    mission_f.write(str(long_wp[start_index]))
    mission_f.write("\t")
    mission_f.write(str(el_wp[start_index]))
    mission_f.write("\t1\n")

def wp(start_index):
    """
    Print waypoint
    """
    #f_sphere.write(str(start_index + 3))
    f_sphere.write("\t0\t3\t16\t")
    f_sphere.write(str(wp_hold_time)+"\t0\t0\t0\t")
    f_sphere.write(str(lat_wp[start_index]))
    f_sphere.write("\t")
    f_sphere.write(str(long_wp[start_index]))
    f_sphere.write("\t")
    f_sphere.write(str(el_wp[start_index]))
    f_sphere.write("\t1\n")
    mission_f.write("\t0\t3\t16\t")
    mission_f.write(str(wp_hold_time)+"\t0\t0\t0\t")
    mission_f.write(str(lat_wp[start_index]))
    mission_f.write("\t")
    mission_f.write(str(long_wp[start_index]))
    mission_f.write("\t")
    mission_f.write(str(el_wp[start_index]))
    mission_f.write("\t1\n")

def land():
    """
    Create a land waypoint.
    """
    ### Define Land (21)
    #f_sphere.write(str(len(long_wp)+1))
    f_sphere.write(str(len(elBranch)*passes + 1))
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
    mission_f.write(str(len(elBranch)*passes + 1))
    mission_f.write("\t0\t3\t21\t0\t0\t0\t")
    mission_f.write(str(heading))
    mission_f.write("\t")
    mission_f.write(str(math.degrees(latStart)))
    mission_f.write("\t")
    mission_f.write(str(math.degrees(longStart)))
    mission_f.write("\t")
    mission_f.write(str(0))
    mission_f.write("\t1\n")
    mission_f.close()

def ardupilot_plan():
    """
    Create a flight plan for an Ardupilot system.
    """
    home_pos()
    #takeoff()
    n = 0
    for i in range(len(elBranch*passes)):
        f_sphere.write(str(n + 2))
        mission_f.write(str(n + 2))
        wp(i)
        f_sphere.write(str(n + 3))
        mission_f.write(str(n + 3))
        yaw(i)
        n = n + 2
    land()

def px4_plan():
    """
    Create a flight plan for a PX4 system.
    """
    home_pos()
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
        mission_f.write(str(i+1))
        mission_f.write("\t0\t3\t16\t")
        mission_f.write(str(wp_hold_time)+"\t0\t0\t")
        mission_f.write(str(heading))
        mission_f.write("\t")
        mission_f.write(str(lat_wp[i]))
        mission_f.write("\t")
        mission_f.write(str(long_wp[i]))
        mission_f.write("\t")
        mission_f.write(str(el_wp[i]))
        mission_f.write("\t1\n")
    land()

def waypoints_csv():
    """
    Write a csv file to ./mission/waypoints.csv for the broken ROS code to work with.
    """
    wp_file = open(home + "/catkin_ws/src/Drone-Project-code/mission/waypoints.csv", "w+")
    for i in range(len(elBranch*passes)):
        wp_file.write(str(lat_wp[i]))
        wp_file.write("\t")
        wp_file.write(str(long_wp[i]))
        wp_file.write("\t")
        wp_file.write(str(el_wp[i]))
        wp_file.write("\n")

def main():
    """
    Choose between Ardupilot or PX4 flight plans.
    """
    print(colored("Saving waypoints in ./mission/ and ./mission/mission.waypoints. " + 
            "Run this on the payload computer to update the ROS mission.", "green", attrs=['bold', 'underline']))
    if firmware == "Ardupilot":
        ardupilot_plan()
    elif firmware == "PX4":
        px4_plan()
    waypoints_csv()


if __name__ == '__main__':
    main()