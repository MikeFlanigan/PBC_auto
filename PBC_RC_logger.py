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


LoopTargetTime = 1 # milliseconds
Timer_15Hz = dt.datetime.now()

over_count = 0
general_count = 0

RC_data = []
AUTO = False
WAYPT = False
SteerInput = 0 # ~ input -100 - +100, but not actually
SteerCmd = 0 # output -20 - 20
ThrotInput = 0 # ~ input -100 - +100, but not actually
ThrotCmd = 0 # output, 0 - 100

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
##            print('Warm start in process, dont move controls... ',round(SteerCenter,3),round(ThrotBottom,3))
            pass
        
        # check queues for new data
        while not RCStatusQueue.empty():
            RC_data = RCStatusQueue.get()
            # parse this into
            AUTO = RC_data[0]
            WAYPT = RC_data[1]
            SteerInput = RC_data[2]
            ThrotInput = RC_data[3]
##            print('rc data: ',RC_data)

            
##            # need to warm start get data from these...
##            if warmingUp:
##                if SteerInput != 0:
##                    SteerCenter = 0.9*SteerCenter + 0.1*SteerInput
##                if ThrotInput != 0:
##                    ThrotBottom = 0.9*ThrotBottom + 0.1*ThrotInput
##                print(SteerCenter,ThrotBottom)
##            else:
##                SteerCmd = SteerInput - SteerCenter
##                ThrotCmd = ThrotInput - (100 + ThrotBottom)
##                print(RC_data,' Steer: ',SteerCmd,' Throt: ',ThrotCmd)
                
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
        if (dt.datetime.now() - Timer_15Hz).microseconds >= LoopTargetTime*1000:
            Timer_15Hz = dt.datetime.now()
            general_count += 1
            
            # send a control command
            SteerCmd = int(ScaleFxn(SteerInput,-100,100,-20,20))
            ThrotCmd = int(ScaleFxn(ThrotInput,-100,100,0,100))
            
            if SteerCmd > 20: SteerCmd = 20 # clipping 
            elif SteerCmd < -20: SteerCmd = -20 # clipping 
            if -2 <= SteerCmd <= 2: SteerCmd = 0 # create a toleranced deadband

            if ThrotCmd > 85: ThrotCmd = 100 # clipping and also helping get full throttle out of the motor       
            if ThrotCmd < 4: ThrotCmd = 0 # create a toleranced deadband
            
            print("Steering: ",SteerCmd," Throttle: ",ThrotCmd)

            Maestro.setTarget(SteerChannel,int(ScaleFxn(SteerCmd,-20,20,microSecMin,microSecMax)))


            MotorSendValue = int(ScaleFxn(ThrotCmd,0,100,PWMlow,PWMhigh))
            if MotorSendValue < 80: MotorSendValue = 0 # clip since 80 is about the lowest to get motion due to friction and start torque
            Maestro.setPWM(MotorChannel,MotorSendValue) # 0 - 127
        
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
##        print('Loop time in microsec: ',(dt.datetime.now() - loop_start).microseconds)
        if (dt.datetime.now() - loop_start).microseconds > (LoopTargetTime+1)*1000:
##            print(' ')
##            print((dt.datetime.now() - loop_start).microseconds)
##            print("Warning loop time has exceeded 10 milliseconds!!")
##            print(' ')
            over_count += 1
            
except KeyboardInterrupt:
    pass

except:
    print('Exception:')
    e = sys.exc_info()
    print(e)
    pass
print('Number of loops within target cycle time: ',general_count)
print('Number of loops over target cycle time: ',over_count)

RCSer.close()
SensorSer.close()
