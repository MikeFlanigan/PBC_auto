import time
import datetime as dt
import serial
from queue import Queue, Empty
from threading import Thread
import numpy as np
import os, sys
import maestro


# serial setups
time.sleep(0.25)
RCSer = serial.Serial('/dev/ttyUSB0',baudrate = 115200) # varies on which is plugged first
time.sleep(0.25)
SensorSer = serial.Serial('/dev/ttyACM0',baudrate = 115200) # consistent
time.sleep(0.25)

### could speed both of these up by just not broadcasting them if they haven't changed
def ListenForRCData(RCStatusQueue):
    RC_data_loc = [0, 0, 0, 0] # [AUTO, WAYPT, SteerCmd, ThrotCmd]
    RC_data_send = [0, 0, 0, 0]
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
                RC_data_loc[2] = float(rawRCdata.split("%")[0]) # steer cmd
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

while True:
    loop_start = dt.datetime.now() # update loop clock

    while not RCStatusQueue.empty():
        RC_data = RCStatusQueue.get()
        # parse this into
        AUTO = RC_data[0]
        WAYPT = RC_data[1]
        SteerCmd = RC_data[2]
        ThrotCmd = RC_data[3]
##        print('rc data: ',RC_data)

    while not GPSQueue.empty():
        GPS_data = GPSQueue.get()
        # parse this into
        GPSfix = GPS_data[0]
        GPSquality = GPS_data[1]
        GPSlat = GPS_data[2]
        GPSlon = GPS_data[3]
        GPSspd = GPS_data[4]
        GPSang = GPS_data[5]
##        print(' ')
##        print('GPS data: ',GPS_data)
##        print(' ')

    while not CompassQueue.empty():
        Compass_data = CompassQueue.get()
        # parse this into
        Compass = Compass_data[0] + 90
        if Compass > 360: Compass = Compass - 360
        print('Compass: ',Compass)

##    print('Loop time in microsec: ',(dt.datetime.now() - loop_start).microseconds)
