import time
import datetime as dt
import serial
from queue import Queue, Empty
from threading import Thread
import numpy as np
import os, sys
import maestro

# maestro setup
Maestro = maestro.Controller('/dev/ttyACM1')
SteerChannel = 6
MotorChannel = 8
# servo settings, the 4x mult is due to quarter microsecs 
microSecMin = 4*750 # -- turns boat left
microSecMax = 4*2500 # -- turns boat right
Maestro.setRange(SteerChannel, microSecMin, microSecMax)
Maestro.setAccel(SteerChannel,254) # basically max
Maestro.setSpeed(SteerChannel,150) # 0-255

def ScaleFxn(x, fromLow, fromHigh, toLow, toHigh):
    x = (((x - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow)) + toLow
    return x

LoopTargetTime = 500 # milliseconds
Timer_15Hz = dt.datetime.now()

MSteerCmd = 0
SteerCmd = 0
SteerDelta = 10
MThrotCmd = -100
ThrotCmd = 0
ThrotDelta = 10
PWMlow = 0
PWMhigh = 127
while True:
    
    # 15 Hz loop for control and logging
    if (dt.datetime.now() - Timer_15Hz).microseconds >= LoopTargetTime*1000:
        Timer_15Hz = dt.datetime.now()

        if MSteerCmd + SteerDelta > 100 or MSteerCmd + SteerDelta < -100: SteerDelta = -1*SteerDelta
        MSteerCmd += SteerDelta

        if MThrotCmd + ThrotDelta > 100 or MThrotCmd + ThrotDelta < -100: ThrotDelta = -1*ThrotDelta
        MThrotCmd += ThrotDelta
        
        # send a control command
        SteerCmd = int(ScaleFxn(MSteerCmd,-100,100,-20,20))
        ThrotCmd = int(ScaleFxn(MThrotCmd,-100,100,0,100))
        
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
        

                
