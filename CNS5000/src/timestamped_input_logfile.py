#!/usr/bin/python
import time
import serial
from os.path import expanduser


# configure the serial connections 
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600, #8N1
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

if (ser.isOpen()):
    ser.close()
ser.open()

# Create a log file
home = expanduser("~")
fName = home+"/catkin_ws/rosbags/timestamped_log.txt"
outFile = open(fName, "wb")
# Send commands to CNS-5000 to start the logs
ser.write('unlogall\r\n')

ser.write('LOG COM1 INSPVAA ONTIME 0.2\r\n')
timenow = time.time()

try:
        while ser.inWaiting() > 0:
            # While data is in the buffer
            timethen = timenow
            timenow = time.time()
            speed = timenow - timethen
            kvh5000_output = timethen + ": " + ser.readline() # Read data a line of data from buffer
            
            outFile.write(kvh5000_output) # Option to log data to file
            print(kvh5000_output)

         
                     
except KeyboardInterrupt:
    ser.write('unlogall\r\n') # Send a message to CNS-5000 to stop sending logs
    outFile.close() 
    ser.close()
    raise


