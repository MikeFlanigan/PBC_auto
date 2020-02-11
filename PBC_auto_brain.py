import time
import datetime as dt
import serial
from queue import Queue, Empty
from threading import Thread
import numpy as np
import os, sys
import maestro
from haversine import RangeAndBearing


# serial setups
time.sleep(0.25)
RCSer = serial.Serial('/dev/ttyUSB0',baudrate = 115200) # varies on which is plugged first
time.sleep(0.25)
SensorSer = serial.Serial('/dev/ttyACM0',baudrate = 115200) # consistent
time.sleep(0.25)

# maestro setup
Maestro = maestro.Controller('/dev/ttyACM1')
SteerChannel = 6
MotorChannel = 8
# servo settings, the 4x mult is due to quarter microsecs 
microSecMin = 4*750 # -- turns boat left
microSecMax = 4*2500 # -- turns boat right
Maestro.setRange(SteerChannel, microSecMin, microSecMax)
Maestro.setAccel(SteerChannel,254) # basically max
Maestro.setSpeed(SteerChannel,100) # 0-255 close to max but slightly unkown, look in maestro code for more info
PWMlow = 0
PWMhigh = 127


# steering params
MAXRIGHT = 20
MAXLEFT = -20 

def ScaleFxn(x, fromLow, fromHigh, toLow, toHigh):
    x = (((x - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow)) + toLow
    return x

def ListenForRCData(RCStatusQueue):
    RC_data_loc = [0, 0, 0, 0] # [AUTO, WAYPT, SteerCmd, ThrotCmd]
    RC_data_send = [0, 0, 0, 0]
    RC_steer_hist = [0]*10 # moving filter
    while True:
        # listen for RC data for the Ardu and send it into the queue
        rawRCdata = RCSer.readline()
        try:
            rawRCdata = rawRCdata.decode("utf-8")
            

            if "Man" in rawRCdata:
                RC_data_loc[0] = 0
            elif "Aut" in rawRCdata:
                RC_data_loc[0] = 1
                
            if "WpLo" in rawRCdata:
                RC_data_loc[1] = 0
            elif "WpHi" in rawRCdata:
                RC_data_loc[1] = 1

            if len(rawRCdata.split("%")) > 1:
                RC_steer_hist.append(float(rawRCdata.split("%")[0])) # steer cmd
                RC_steer_hist.pop(0)

                RC_data_loc[2] = np.mean(RC_steer_hist)
                RC_data_loc[3] = float(rawRCdata.split("%")[1]) # throt cmd

##                print('rc data: ',RC_data_loc)
                RCSer.reset_input_buffer()
        except:
            print('error in rc data thread')
        
        # put in queue
        if RC_data_loc != RC_data_send:
            RC_data_send = RC_data_loc.copy()
            RCStatusQueue.put(RC_data_send)

def ListenForSensorData(GPSQueue, CompassQueue):
    GPS_data_loc = [0,0,0,0,0,0] # [fix, quality, lat, lon, spd, ang]
    GPS_data_send = [0,0,0,0,0,0]
    Compass_data_loc = [0]
    Compass_data_send = [0]
    while True:
        # listen for compass and gps data
        rawsensorData = SensorSer.readline()
        try:
            rawsensorData = rawsensorData.decode("utf-8")
            if "(deg)" in rawsensorData:
                Compass_data_loc[0] = float(rawsensorData.split(' ')[-1])
                
            elif "Fix:" in rawsensorData:
                s = rawsensorData.find(":")
                if rawsensorData[s+2].isdigit(): GPS_data_loc[0] = int(rawsensorData[s+2])

                s = rawsensorData.find(":",s+2)
                if rawsensorData[s+2].isdigit(): GPS_data_loc[1] = int(rawsensorData[s+2])

            elif "Loc" in rawsensorData:
                rawsensorData = rawsensorData[rawsensorData.find('Loc')+3:].split(',')
                GPS_data_loc[2] = float(rawsensorData[0])
                GPS_data_loc[3] = float(rawsensorData[1])
                
            elif "(knts)" in rawsensorData:
                GPS_data_loc[4] = float(rawsensorData.split(' ')[-1])
            elif "Ang " in rawsensorData:
                GPS_data_loc[5] = float(rawsensorData.split(' ')[-1])
                SensorSer.reset_input_buffer()
            else:
                pass
        except:
            print('error in sensor data thread')
            pass
        
        # put in queue
        if GPS_data_loc != GPS_data_send:
            GPS_data_send = GPS_data_loc.copy()
            GPSQueue.put(GPS_data_send)
        if Compass_data_loc != Compass_data_send:
            Compass_data_send = Compass_data_loc.copy()
            CompassQueue.put(Compass_data_send)

# start the listener threads
RCStatusQueue = Queue()
RCListenerThread = Thread(target=ListenForRCData, args=(RCStatusQueue,))
RCListenerThread.setDaemon(True)
RCListenerThread.start()
print('started RCListenerThread thread...')
GPSQueue = Queue()
CompassQueue = Queue()
SensorListenerThread = Thread(target=ListenForSensorData, args=(GPSQueue,CompassQueue,))
SensorListenerThread.setDaemon(True)
SensorListenerThread.start()
print('started SensorListenerThread thread...')


LoopTargetTime = 10 # milliseconds
Timer_Hz = dt.datetime.now()

over_count = 0
general_count = 0


###########


RC_data = []
AUTO = False
WAYPT = False
SteerInput = 0 # ~ input -100 - +100, but not actually
SteerCmd = 0 # output -20 - 20
ThrotInput = 0 # ~ input -100 - +100, but not actually
ThrotCmd = 0 # output, 0 - 100

AUTO_RisingEdge = False
AUTO_FallingEdge = False
WAYPT_RisingEdge = False

AutoFlag = False
WptFlag = False

IDLE = False # post race completion mode

GPSfix = 0
GPSquality = 0
GPSlat = 0.0
GPSlon = 0.0
GPSspd = 0.0
GPSang = 0.0

Compass = 0.0
CompassOffset = 90 # Believe the rotation is +90 to the heading

wptRadius = 5 # meters, ~ 15 feet

# try to load waypoint array
if os.path.isfile('WayPoints.npy'):
    WayPoints = np.load('WayPoints.npy')
else:
    WayPoints = np.zeros((20,2)) # 20 waypoints of Lat Long

wptInd = 0 # which waypoint 
try:
    while True:
        loop_start = dt.datetime.now() # update loop clock

        # ------------- DATA inputs -------------------
        # check RC queue for new data
        while not RCStatusQueue.empty():
            RC_data = RCStatusQueue.get()
            # parse this into
            AUTO = RC_data[0]
            WAYPT = RC_data[1]
            SteerInput = RC_data[2]
            ThrotInput = RC_data[3]
            
        # check GPS queue for new data
        while not GPSQueue.empty():
            GPS_data = GPSQueue.get()
            # parse this into
            GPSfix = GPS_data[0]
            GPSquality = GPS_data[1]
            GPSlat = GPS_data[2]
            GPSlon = GPS_data[3]
            GPSspd = GPS_data[4]
            GPSang = GPS_data[5]

        # check Compass queue for new data
        while not CompassQueue.empty():
            Compass_data = CompassQueue.get()
            # parse this into
            Compass = Compass_data[0] + CompassOffset
            if Compass > 360: Compass = Compass - 360

        # ------- edge detection -----------
        if AUTO and not AutoFlag:
            AUTO_RisingEdge = True
            AutoFlag = True
        else:
            AUTO_RisingEdge = False

        if not AUTO and AutoFlag:
            AUTO_FallingEdge = True
            AutoFlag = False
        else:
            AUTO_FallingEdge = False

        if WAYPT and not WptFlag:
            WAYPT_RisingEdge = True
            WptFlag = True
        else:
            WAYPT_RisingEdge = False
            
        if not WAYPT: WptFlag = False # reset the flag
        # -- END - edge detection -----------
        
        # ----- END ---- DATA inputs -------------------
            
        if not AUTO: # manual mode
            if WAYPT_RisingEdge:
                print('Logging waypoint to wpt array')
                #append gps
                WayPoints[wptInd,0] = GPSlat
                WayPoints[wptInd,1] = GPSlon
                WayPoints[wptInd+1:,:]=0 # zero the array past this point
                wptInd += 1 # uptick wpt array

                np.save('WayPoints.npy',WayPoints)
                print(WayPoints)

            # ---------- Calculate control signals -------------------
            SteerCmd = int(ScaleFxn(SteerInput,-100,100,-20,20))
            ThrotCmd = int(ScaleFxn(ThrotInput,-100,100,0,100))      
            # ---- END - Calculate control signals -------------------
            
        if AUTO_RisingEdge:
            wptInd = 0
            
        if AUTO_FallingEdge:
            wptInd = 0
            IDLE = False

        DEBUG = False
        if AUTO:
            if not GPSfix and not DEBUG: print('no GPS fix... waiting...')
            elif GPSfix or DEBUG:
                if IDLE:
                    SteerCmd = 0
                    ThrotCmd = 0
                else:
                    WptLat = WayPoints[wptInd,0]
                    WptLon = WayPoints[wptInd,1]

##                    GPSlat = 32.750330 
##                    GPSlon = -117.202724
##                    sims = [[32.750407,-117.193167],[32.756678,-117.195709],[32.754522,-117.216169],[32.743998,-117.223547],[32.733403,-117.202390]]
##                    WptLat = sims[0][0]
##                    WptLon = sims[0][1]
                    
                    Range, Bearing = RangeAndBearing(GPSlat,GPSlon,WptLat,WptLon)
##                    print('Distance: ', Range,' in meters')
##                    print('Bearing to next wpt: ',Bearing)

                    if Range < wptRadius:
                        print(' ')
                        print('Waypoint ',wptInd,' hit!!')
                        wptInd += 1
                        if WayPoints[wptInd,0] == 0 or WayPoints[wptInd,1] == 0:
                            IDLE = True
                            print(' ')
                            print('MISSION COMPLETE')

                    
                    if not IDLE:
                        # transform Bearing to be 0-360 with 0 being east
                        if 0 < Bearing <= 180: B = Bearing
                        elif Bearing < 0:
                            B = 360 - abs(Bearing)

                        # transform H to be 0 east
                        H = Compass - 90
                        if H < 0: H = 360+H # i think this resolves it
                        H = abs(360-H)

                        if 0 < B - H < 180:
                            # turn left
                            SteerCmd = -1*(B-H)
                            print('Bearing: ',B,' Heading: ',H,' turn left')
                        elif 180 <= B - H:
                            # turn right
                            SteerCmd = 360 - B - H
                            print('Bearing: ',B,' Heading: ',H,' turn right')
                        elif -180 < B - H < 0:
                            # turn right
                            SteerCmd = -1*(B-H)
                            print('Bearing: ',B,' Heading: ',H,' turn right')
                        elif B - H < -180:
                            # turn left
                            SteerCmd = -360 - B + H
                            print('Bearing: ',B,' Heading: ',H,' turn left')
                            
                        ThrotCmd = 100 # full speed ahead...
                

        if (dt.datetime.now() - Timer_Hz).microseconds >= LoopTargetTime*1000:
            Timer_Hz = dt.datetime.now()
            
            # ---------- write control signals -------------------
            if SteerCmd > 20: SteerCmd = 20 # clipping 
            elif SteerCmd < -20: SteerCmd = -20 # clipping 
            if -2 <= SteerCmd <= 2: SteerCmd = 0 # create a toleranced deadband
            Maestro.setTarget(SteerChannel,int(ScaleFxn(SteerCmd,-20,20,microSecMin,microSecMax)))

            if ThrotCmd > 85: ThrotCmd = 100 # clipping and also helping get full throttle out of the motor       
            if ThrotCmd < 4: ThrotCmd = 0 # create a toleranced deadband
            MotorSendValue = int(ScaleFxn(ThrotCmd,0,100,PWMlow,PWMhigh))
            if MotorSendValue < 80: MotorSendValue = 0 # clip since 80 is about the lowest to get motion due to friction and start torque
            Maestro.setPWM(MotorChannel,MotorSendValue) # 0 - 127
            # ---- END - write control signals -------------------

            

except KeyboardInterrupt:
    pass

