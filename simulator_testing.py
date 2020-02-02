import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from PBC_simulator import *


# list of waypoints in the north end of the boulder res
simulationWaypoints = [[40.085122, -105.216373],
                       [40.084301, -105.216620],
                       [40.083359, -105.217304],
                       [40.082407, -105.216414],
                       [40.084508, -105.215524]]
simulationWaypoints = pd.DataFrame(simulationWaypoints,columns=['Lats','Longs'])

boatSim = Simulator(simulationWaypoints.Lats[0],simulationWaypoints.Longs[0])
##ax.scatter(boatSim.trueLong, boatSim.trueLat, zorder=1, alpha= 0.8, c='k', s=10)
##ax.scatter(boatSim.originLong, boatSim.originLat, zorder=1, alpha= 0.8, c='r', s=10)
##plt.show()

speed_hist = []
x_hist = []
y_hist = []
omega_hist = []
theta_hist = []

dt = 0.01
simEnd = 2 # seconds
time = np.arange(0,simEnd,dt)
boatSim.theta = np.deg2rad(0)
for t in time:
    
    boatSim.throt = 100

    boatSim.theta = np.deg2rad(90)
    boatSim.steer = 0
    
##    if t < 1: boatSim.steer = 0
##    elif t < 5: boatSim.steer = 20 # hard right
##    elif t < 10: boatSim.steer = -20 # hard left
    
    boatSim.update(dt)

    speed_hist.append(boatSim.v)
    x_hist.append(boatSim.x)
    y_hist.append(boatSim.y)
    omega_hist.append(boatSim.omega)
    theta_hist.append(boatSim.theta)

plt.figure(1)
plt.plot(time,speed_hist)
plt.ylabel('Speed m/s')
plt.xlabel('Seconds')

plt.figure(2)
plt.plot(x_hist,y_hist,'ko')
plt.ylabel('y [m]')
plt.xlabel('x [m]')
plt.axis('equal')

plt.figure(3)
plt.plot(time,omega_hist)
plt.ylabel('Omega, rad/s')
plt.xlabel('Seconds')

plt.figure(4)
plt.plot(time,np.rad2deg(theta_hist))
plt.ylabel('theta, deg')
plt.xlabel('Seconds')

plt.show()
