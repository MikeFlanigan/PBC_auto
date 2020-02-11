from math import *

def RangeAndBearing(lat1,lon1,lat2,lon2):
##    lat1 = 32.750330
##    lon1 = -117.202724

    ##lat2 = 32.750303
    ##lon2 = -117.197810

##    lat2 = 32.754964
##    lon2 = -117.202951

    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    Base = 6371 * c


    Bearing =atan2(cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1), sin(lon2-lon1)*cos(lat2)) 

    Bearing = degrees(Bearing)
##    print("")
##    print("")
##    print ("--------------------")
##    print ("Horizontal Distance:")
##    print(Base)
    Range = Base * 1000 # answer in meters
##    print("--------------------")
##    print("Bearing (degs ((East is 0)):")
##    print(Bearing)
##    print("--------------------")
##    Bearing = Bearing + 90
##    if Bearing > 360: Bearing = Bearing - 360
    return Range, Bearing

