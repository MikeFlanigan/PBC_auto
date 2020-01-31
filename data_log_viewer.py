import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import pyproj
geodesic = pyproj.Geod(ellps='WGS84')

dataDir = './test_data/jan_31/'

##data = np.load(dataDir +'Datalog_0.npy')
##data = np.load(dataDir +'Datalog_1.npy')
data = np.load(dataDir + 'Datalog_2.npy')
##data = np.load(dataDir +'Datalog_3.npy')
##data = np.load(dataDir +'Datalog_4.npy') # this file seems to indicate a bias in the map or gps data


# drop empty rows
data = data[data[:,0]>0]
rows = len(data)
print(rows ,' data points')

dataDF = pd.DataFrame(data, columns = ['TS','Steer','Throt','Lat','Lon','Spd','GPSAng','CompassAng'])

print('Elapsed time: ',round(dataDF.loc[rows-1].TS - dataDF.loc[0].TS,3),' seconds')

print('avg throttle: ',dataDF.Throt.mean(),' avg speed: ',dataDF.Spd.mean())


TS = dataDF.TS.to_list()

## plots of command history
##plt.figure(1)
##plt.plot(TS, dataDF.Steer.to_list())
##plt.title("Steering cmd hist")
##plt.xlabel("Time (sec)")
##
##plt.figure(2)
##plt.plot(TS, dataDF.Throt.to_list())
##plt.title("Throttle cmd hist")
##plt.xlabel("Time (sec)")
##
##plt.show()


## live plot of lat long data
latMax = 40.0892
latMin = 40.0657
longMax = -105.2027#-105.1926
longMin = -105.2438#-105.2435
##latMax = 40.0802
##latMin = 40.0757
##longMax = -105.2227#-105.1926
##longMin = -105.23#-105.2435
BBox = (longMin,   longMax,      
         latMin, latMax)

mapImage = plt.imread('BoulderRes.png')

fig, ax = plt.subplots(figsize = (8,7))
##ax.scatter(dataDF.Lon, dataDF.Lat, zorder=1, alpha= 0.8, c='k', s=10)
ax.set_title('Plotting Waypoint Data')
buff = max(dataDF.Lon) - min(dataDF.Lon)
buff = 0.001
ax.set_xlim(min(dataDF.Lon)-buff,max(dataDF.Lon)+buff)
ax.set_ylim(min(dataDF.Lat)-buff,max(dataDF.Lat)+2*buff)
ax.imshow(mapImage, zorder=0, extent = BBox)
##plt.show()
# below plots those waypoints on the above map one at a time and updates
for i in range(0,len(dataDF),10):
    ax.scatter(dataDF.Lon[i], dataDF.Lat[i], zorder=1, alpha= 0.8, c='k', s=10)
    plt.pause(0.001)
    print(i,' of ',len(dataDF),' ', dataDF.Spd[i],' knts')
##plt.show()
