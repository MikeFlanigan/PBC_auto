import serial

ardu_ser = serial.Serial('/dev/ttyUSB0',baudrate = 115200)
GPS_ser = serial.Serial('/dev/ttyACM0',baudrate = 115200)

while True:
    heard = ardu_ser.readline()
    print('heard:',heard)
    print(' ')

    # definitely need to thread and then queue this
    GPS = GPS_ser.readline() 
    print('GPS: ',GPS)
