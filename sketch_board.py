import pandas as pd
#### unsure
import pyproj
geodesic = pyproj.Geod(ellps='WGS84')
##fwd_azimuth,back_azimuth,distance = geodesic.inv(lat1, long1, lat2, long2)
#### unsure

# list of waypoints in the north end of the boulder res
simulationWaypoints = [[40.085122, -105.216373],
                       [40.084301, -105.216620],
                       [40.083359, -105.217304],
                       [40.082407, -105.216414],
                       [40.084508, -105.215524]]
simulationWaypoints = pd.DataFrame(simulationWaypoints,columns=['Lats','Longs'])
captureWaypoint = False # this is a flag triggered by RC remote

waypointArray = []

currentLatLong = [0,0]
if captureWaypoint:
    waypointArray.append(currentLatLong)

for i in range(len(simulationWaypoints)-1):
	lat1 = simulationWaypoints.loc[i][0]
	long1 = simulationWaypoints.loc[i][1]
	lat2 = simulationWaypoints.loc[i+1][0]
	long2 = simulationWaypoints.loc[i+1][1]
##	print(lat1,long1)
	fwd_azimuth,back_azimuth,distance = geodesic.inv(long1, lat1, long2, lat2)
	print('Distance: ', distance,' in meters?')
	print('Bearing to next wpt: ',fwd_azimuth)

	print(' ')

# probably going to move this to a testing file
import matplotlib.pyplot as plt
latMax = 40.0892
latMin = 40.0657
longMax = -105.2027#-105.1926
longMin = -105.2438#-105.2435
fwd_azimuth,back_azimuth,Xdistance = geodesic.inv(longMin, latMax, longMax, latMax)
fwd_azimuth,back_azimuth,Ydistance = geodesic.inv(longMin, latMin, longMin, latMax)
print(Xdistance,Ydistance)
BBox = (longMin,   longMax,      
         latMin, latMax)

mapImage = plt.imread('BoulderRes.png')

fig, ax = plt.subplots(figsize = (8,7))
##ax.scatter(simulationWaypoints.Longs, simulationWaypoints.Lats, zorder=1, alpha= 0.8, c='k', s=10)
ax.set_title('Plotting Waypoint Data')
##ax.set_xlim(BBox[0],BBox[1])
##ax.set_ylim(BBox[2],BBox[3])
buff = max(simulationWaypoints.Longs) - min(simulationWaypoints.Longs)
ax.set_xlim(min(simulationWaypoints.Longs)-buff,max(simulationWaypoints.Longs)+buff)
ax.set_ylim(min(simulationWaypoints.Lats)-buff,max(simulationWaypoints.Lats)+buff)
ax.imshow(mapImage, zorder=0, extent = BBox)#, aspect= 'equal') # aspect doesn't seem to be working
ax.arrow(simulationWaypoints.Longs[0], simulationWaypoints.Lats[0], buff/10, buff/10, width = 0.00005, color = 'r')

# below plots those waypoints on the above map one at a time and updates
for i in range(len(simulationWaypoints)):
    ax.scatter(simulationWaypoints.Longs[i], simulationWaypoints.Lats[i], zorder=1, alpha= 0.8, c='k', s=10)
    plt.pause(1)
##plt.show()
	
