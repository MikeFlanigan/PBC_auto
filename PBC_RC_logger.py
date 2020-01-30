import time
import datetime as dt
import serial
from queue import Queue
from queue import Empty
from threading import Thread
import pandas as pd
import numpy as np
import sys





def ListenForRCData(RCStatusQueue):

    while True:
        # listen for RC data for the Ardu and send it into the queue
        pass


def ListenForSensorData(GPSQueue, CompassQueue):

    while True:
        # listen for compass and gps data
        pass

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


LoopMaxTime = 10 # milliseconds
Timer_100Hz = dt.datetime.now()

over_count = 0
general_count = 0


# main loop
try:
    while True:
        loop_start = dt.datetime.now() # update loop clock

        
        # 100 Hz loop for control and logging
        if (dt.datetime.now() - Timer_100Hz).microseconds >= 10*1000:
            Timer_100Hz = dt.datetime.now()
            general_count += 1
    ##        print('thing')
            # send a control command

            # add data to the log


        # warning flag if the loop takes longer then 10 milliseconds
    ##    print((dt.datetime.now() - loop_start).microseconds)
        if (dt.datetime.now() - loop_start).microseconds > LoopMaxTime*1000:
            print(' ')
            print((dt.datetime.now() - loop_start).microseconds)
            print("Warning loop time has exceeded 10 milliseconds!!")
            print(' ')
            over_count += 1
            pass
except KeyboardInterrupt:
    pass
print(general_count)
print(over_count)
