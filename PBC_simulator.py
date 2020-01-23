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


class Simulator:
    
    def __init__(self, lat, long):
        self.trueLat = lat
        self.trueLong = long

        originDist = 2.5 # km away
        originBearing = 225 # degrees to encourage a first quadrant simulator
        self.originLat = newCoordFromDelta(lat, long, originDist, originBearing)[0]
        self.originLong = newCoordFromDelta(lat, long, originDist, originBearing)[1]
        
        print('there')
