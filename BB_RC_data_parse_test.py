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
                RC_data_loc[2] = float(rawRCdata.split("%")[0]) # steer cmd
                RC_data_loc[3] = float(rawRCdata.split("%")[1]) # throt cmd
##                print('rc data: ',RC_data_loc)
                RCSer.reset_input_buffer()
        except:
            print('error in rc data thread')
        
        # put in queue
        RCStatusQueue.put(RC_data_loc)


# start the listener threads
RCStatusQueue = Queue()
RCListenerThread = Thread(target=ListenForRCData, args=(RCStatusQueue,))
RCListenerThread.setDaemon(True)
RCListenerThread.start()
print('started RCListenerThread thread...')


while True:
    loop_start = dt.datetime.now() # update loop clock

    # check queues for new data
##        if RCStatusQueue.empty(): time.sleep(0.002) 
    while not RCStatusQueue.empty():
        RC_data = RCStatusQueue.get()
        # parse this into
        AUTO = RC_data[0]
        WAYPT = RC_data[1]
        SteerCmd = RC_data[2]
        ThrotCmd = RC_data[3]
        print('rc data: ',RC_data)
