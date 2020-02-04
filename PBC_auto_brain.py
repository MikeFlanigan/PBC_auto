import time
import datetime as dt
import serial
from queue import Queue, Empty
from threading import Thread
import numpy as np
import os, sys
import maestro

AUTO = False
WAYPT = False
SteerCmd = 0 # ~ input -100 - +100, but not actually, output -20 - 20
ThrotCmd = 0 # ~ input -100 - +100, but not actually, output, 0 - 100

SteerCenter = 0 # will capture the center value
ThrotBottom = 0 # will capture the center value

IDLE = True
try:
    while True:

        if not AUTO:
            if WAYPT_RisingEdge: 
                append gps
            if WAYPT_FallingEdge:
                save waypoint array

            get manual controls
            write control signals
            
        if AUTO_RisingEdge:
            load waypoint array

        if AUTO:
            update sensor readings
            check if current waypoint is within radius distance
                if yes, then load next waypoint
                if no more to load, then go into idle mode
            navigate to current waypoint
            write control signals

        control loop speed
        send controls
            

except KeyboardInterrupt:
    pass

