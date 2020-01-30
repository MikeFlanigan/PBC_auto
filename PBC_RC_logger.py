import time
import datetime as dt
import serial
from queue import Queue, Empty
from threading import Thread
import numpy as np
import os, sys

# serial setups
time.sleep(0.25)
RCSer = serial.Serial('/dev/ttyUSB0',baudrate = 115200) # varies on which is plugged first
time.sleep(0.25)
SensorSer = serial.Serial('/dev/ttyACM0',baudrate = 115200) # consistent
time.sleep(0.25)
CmdSer = serial.Serial('/dev/ttyUSB1',baudrate = 115200) # varies on which is plugged first
##time.sleep(0.5)

def ScaleFxn(x, fromLow, fromHigh, toLow, toHigh):
    x = (((x - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow)) + toLow
    return x

def ListenForRCData(RCStatusQueue):
    RC_data_loc = [0, 0, 0, 0] # [AUTO, WAYPT, SteerCmd, ThrotCmd]
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
                RC_data_loc[2] = float(rawRCdata.split("%")[0])
                RC_data_loc[3] = float(rawRCdata.split("%")[1])
##                print('rc data: ',RC_data_loc)
                RCSer.reset_input_buffer()
        except:
            print('error in rc data thread')
        
        # put in queue
        RCStatusQueue.put(RC_data_loc)

def ListenForSensorData(GPSQueue, CompassQueue):
    GPS_data_loc = [0,0,0,0,0,0] # [fix, quality, lat, lon, spd, ang]
    Compass_data_loc = [0]
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
        GPSQueue.put(GPS_data_loc)
        CompassQueue.put(Compass_data_loc)

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


LoopTargetTime = 60 # milliseconds
Timer_15Hz = dt.datetime.now()

over_count = 0
general_count = 0

RC_data = []
AUTO = False
WAYPT = False
SteerCmd = 0 # ~ input -100 - +100, but not actually, output -20 - 20
ThrotCmd = 0 # ~ input -100 - +100, but not actually, output, 0 - 100

SteerCenter = 0 # will capture the center value
ThrotBottom = 0 # will capture the center value

LOGGING = False

GPSfix = 0
GPSquality = 0
GPSlat = 0.0
GPSlon = 0.0
GPSspd = 0.0
GPSang = 0.0

Compass = 0.0

logStart = False
logName = 'Datalog_'
logIndex = 0
logArray = np.zeros((180*60,8)) # ['TS','Steer','Throt','Lat','Lon','Spd','GPSAng','CompassAng']
DataRow = 0
logTime = dt.datetime.now()

DataOverflowed = False

warmingUp = True
warm_start = dt.datetime.now()
warm_start_dur = 10 # seconds
# main loop
try:
    while True:
        loop_start = dt.datetime.now() # update loop clock

        if (dt.datetime.now() - warm_start).seconds > warm_start_dur:
            warmingUp = False
        else:
            print('Warm start in process, dont move controls... ',round(SteerCenter,3),round(ThrotBottom,3))

        # check queues for new data
##        if RCStatusQueue.empty(): pass # could have a sleep here...
        while not RCStatusQueue.empty():
            RC_data = RCStatusQueue.get()
            # parse this into
            AUTO = RC_data[0]
            WAYPT = RC_data[1]
            SteerCmd = RC_data[2]
            ThrotCmd = RC_data[3]
            # need to warm start get data from these...
            if warmingUp:
                if SteerCmd != 0 and ThrotCmd != 0:
                    SteerCenter = 0.9*SteerCenter + 0.1*SteerCmd
                    ThrotBottom = 0.9*ThrotBottom + 0.1*ThrotCmd
            else:
                SteerCmd = SteerCmd - SteerCenter
                ThrotCmd = ThrotCmd - (100 + ThrotBottom)
                
            if WAYPT and not DataOverflowed:
                LOGGING = True
            elif not WAYPT:
                LOGGING = False
                DataOverflowed = False
                
        while not GPSQueue.empty():
            GPS_data = GPSQueue.get()
            # parse this into
            GPSfix = GPS_data[0]
            GPSquality = GPS_data[1]
            GPSlat = GPS_data[2]
            GPSlon = GPS_data[3]
            GPSspd = GPS_data[4]
            GPSang = GPS_data[5]
            


            
        while not CompassQueue.empty():
            Compass_data = CompassQueue.get()
            # parse this into
            Compass = Compass_data[0]
        
        
        # 15 Hz loop for control and logging
        if not warmingUp and (dt.datetime.now() - Timer_15Hz).microseconds >= LoopTargetTime*1000:
            Timer_15Hz = dt.datetime.now()
            general_count += 1
            
            # send a control command
            SteerCmd = int(ScaleFxn(SteerCmd,-100,100,-20,20))
            ThrotCmd = int(ScaleFxn(ThrotCmd,-100,100,0,100))
            
            if SteerCmd > 20: SteerCmd = 20 # clipping 
            elif SteerCmd < -20: SteerCmd = -20 # clipping 
            if -2 <= SteerCmd <= 2: SteerCmd = 0 # create a toleranced deadband

            if ThrotCmd > 85: ThrotCmd = 100 # clipping and also helping get full throttle out of the motor       
            if ThrotCmd < 4: ThrotCmd = 0 # create a toleranced deadband
            
            print("Steering: ",SteerCmd," Throttle: ",ThrotCmd)
            controlMsg = "s"+str(SteerCmd)+"e t"+str(ThrotCmd)+"n"
            CmdSer.write(controlMsg.encode())

            
            if LOGGING and not logStart:
                FileName = logName + str(logIndex) +".npy"
                # check if file exists
                while os.path.isfile(FileName):
                    logIndex += 1
                    FileName = logName + str(logIndex) +".npy"
                
                # empty the dataframe
                logArray = np.zeros((180*100,8)) # ['TS','Steer','Throt','Lat','Lon','Spd','GPSAng','CompassAng']
                DataRow = 0 # reset the row count
                logTime = dt.datetime.now() # reset the log timer
                
                logStart = True
                
            if LOGGING:
                # add data to the log
                logArray[DataRow,0] = (dt.datetime.now()-logTime).seconds + (dt.datetime.now()-logTime).microseconds/1000000.0
                logArray[DataRow,1] = SteerCmd
                logArray[DataRow,2] = ThrotCmd
                logArray[DataRow,3] = GPSlat
                logArray[DataRow,4] = GPSlon
                logArray[DataRow,5] = GPSspd
                logArray[DataRow,6] = GPSang
                logArray[DataRow,7] = Compass

                DataRow += 1

            if (not LOGGING and logStart) or DataRow > 180*100 - 2:
                # save the file
                np.save(FileName,logArray)
                
                logStart = False
                LOGGING = False # in case the row overflow happened

                if (DataRow > 180*100 - 2): DataOverflowed = True
                


        # warning flag if the loop takes longer then 10 milliseconds
    ##    print((dt.datetime.now() - loop_start).microseconds)
        if (dt.datetime.now() - loop_start).microseconds > (LoopTargetTime+1)*1000:
##            print(' ')
##            print((dt.datetime.now() - loop_start).microseconds)
##            print("Warning loop time has exceeded 10 milliseconds!!")
##            print(' ')
            over_count += 1
            pass
except KeyboardInterrupt:
    pass

except:
    print('Exception:')
    e = sys.exc_info()
    print(e)
    pass
print(general_count)
print(over_count)

RCSer.close()
SensorSer.close()
CmdSer.close()
