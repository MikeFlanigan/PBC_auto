import math
import numpy as np

def newCoordFromDelta(lat1, lon1, distKm, bearing):
    # bearing is north 0, and goes 0-360
    R = 6378.1 #Radius of the Earth in km
    brng = np.deg2rad(bearing) #Bearing is 90 degrees converted to radians.

    lat1 = math.radians(lat1) #Current lat point converted to radians
    lon1 = math.radians(lon1) #Current long point converted to radians

    lat2 = math.asin( math.sin(lat1)*math.cos(distKm/R) +
         math.cos(lat1)*math.sin(distKm/R)*math.cos(brng))

    lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(distKm/R)*math.cos(lat1),
                 math.cos(distKm/R)-math.sin(lat1)*math.sin(lat2))

    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)

    return [lat2, lon2]

def knots2mps(knts):
    return knts/1.944


class Simulator:
    
    def __init__(self, lat, long):

        self.wind = [0,0] # speed and direction (rads)
        self.trueLat = lat
        self.trueLong = long

        originDist = 2.5 # km away
        originBearing = 225 # degrees to encourage a first quadrant simulator
        self.originLat = newCoordFromDelta(lat, long, originDist, originBearing)[0]
        self.originLong = newCoordFromDelta(lat, long, originDist, originBearing)[1]

        self.x = originDist*1000*np.cos(np.deg2rad(45)) # this is hardcoded since the origin is hardcoded
        self.xdot = 0
        self.xddot = 0
        self.y = originDist*1000*np.sin(np.deg2rad(45))
        self.ydot = 0
        self.yddot = 0

        self.v = (self.xdot**2+self.ydot**2)**0.5

        self.theta = 0 # radians -- heading of boat wrt x axis (east)
        self.omega = 0 # radians/second
        self.alpha = 0 # radians/sec^2

        self.throt = 0 # 0-100%
        self.steer = 0 # -20 (turns boat left) to 20 (turns boat right) degrees
        
        self.phi = 0 # radians of self.steer ... also could account for ramp up at some point

        self.mass = 2.5 # kgs
        self.I = self.mass*0.5**2 # seems some coefficient mr^2 is pretty close to the rotational moment of inertia for common shapes

    def Vupdate(self):
        # just combines xdot and ydot into a speed vector
        self.v = (self.xdot**2+self.ydot**2)**0.5
        
    def update(self, dt):
        self.phi = np.deg2rad(self.steer)

##        if self.theta < 0: self.theta = 2*np.pi + self.theta

        self.x = self.x + self.xdot*dt + 0.5*self.xddot*dt**2
        self.xdot = self.xdot + self.xddot*dt
        
        self.y = self.y + self.ydot*dt + 0.5*self.yddot*dt**2
        self.ydot = self.ydot + self.yddot*dt

        self.theta = self.theta + self.omega*dt + 0.5*self.alpha*dt**2
        self.omega = self.omega + self.alpha*dt
        
        self.Vupdate() # update the combined velocity vector
        
        C1 = 0.2 # coefficient converting throttle % to Newtons force
        C2 = 15 # coefficient capturing lots of complexity in the drag equation
        Cwind = 1 # coefficient capturing lots of complexity in the wind drag equation (note this doesn't account for apparent wind)
        C4 = 2 # coefficient capturing the complexity of rudder lift
        C5 = 2 # coefficient capturing the rotational drag correction torque
        
        # this is where numerical approximation like runga cutta would make sense rather than const accel assumption
##        self.xddot = (self.throt*C1*np.cos(self.theta+self.phi)- C2*self.xdot**2 - Cwind*self.wind[0]**2*np.cos(self.wind[1]))/self.mass
##        self.yddot = (self.throt*C1*np.sin(self.theta+self.phi)- C2*self.ydot**2 - Cwind*self.wind[0]**2*np.sin(self.wind[1]))/self.mass

        self.xddot = (self.throt*C1*np.cos(self.theta)- C2*np.cos(self.theta)*self.v**2 - Cwind*self.wind[0]**2*np.cos(self.wind[1]))/self.mass
        self.yddot = (self.throt*C1*np.sin(self.theta)- C2*np.sin(self.theta)*self.v**2 - Cwind*self.wind[0]**2*np.sin(self.wind[1]))/self.mass
        
##        self.alpha = (C4*(self.phi-self.omega)*self.v**2 - C5*self.omega**2)/self.I # note not including the thrust contribution to rotation, might be negligable
##        self.alpha = (C4*(self.phi-self.omega)*self.v**2 - C5*self.omega**2)/self.I
##        self.alpha = np.round(self.alpha,5)
        Cfake = 1.37
##        self.omega = -1*Cfake * self.v*self.phi
        self.omega = -1*Cfake*self.phi
        
        print('omega:',np.round(self.omega,3),' alpha:',np.round(self.alpha,3),' theta:',np.round(np.rad2deg(self.theta),3),
              ' xdot: ',np.round(self.xdot,3),' ydot: ',np.round(self.ydot,3),' speed: ',np.round(self.v**2,3),' xddot: ',
              np.round(self.xddot,3),' yddot: ', np.round(self.yddot,3))

        
