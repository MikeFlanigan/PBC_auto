import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

##data = np.load('Datalog_4.npy')
##data = np.load('Datalog_5.npy')
data = np.load('Datalog_6.npy')

# drop empty rows
data = data[data[:,0]>0]
rows = len(data)
print(rows ,' data points')

dataDF = pd.DataFrame(data, columns = ['TS','Steer','Throt','Lat','Lon','Spd','GPSAng','CompassAng'])

print('Elapsed time: ',round(dataDF.loc[rows-1].TS - dataDF.loc[0].TS,3),' seconds')

TS = dataDF.TS.to_list()

plt.figure(1)
plt.plot(TS, dataDF.Steer.to_list())
plt.title("Steering cmd hist")
plt.xlabel("Time (sec)")

plt.figure(2)
plt.plot(TS, dataDF.Throt.to_list())
plt.title("Throttle cmd hist")
plt.xlabel("Time (sec)")

plt.show()
