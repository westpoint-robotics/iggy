#!/usr/bin/python
import time
import serial
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from usma_novatel_parser import *
from os.path import expanduser
from std_msgs.msg import String

# configure the serial connections
#defualt after an FRESET command is sent is at buadrate 9600.
#to change baud rate, enter minicom at 9600 and send command "serialconfig com1 115200 n 8 1 n on"
#then exit minicom, re-enter at 115200 baudrate and send command "saveconfig" so that it will stay at that buadrate after turning off
#other settings saved:
#FLIPPED front and back of robot, rotated 180 about z axis
#TODO make this rotation and imu/antenna offset more accurate. the INS not perfectly alligned with robot (a little crooked) and the
#ser.write('VEHICLEBODYROTATION 0 0 180\r\n')
#APPLYVEHICLEBODYROTATION ENABLE
#SETIMUTOANTOFFSET 0.0 0.6096 0.8636 0.05 0.05 0.05
#SETALIGNMENTVEL 1.15
ser = serial.Serial(
    port='/dev/ttyUSB1',
    baudrate=115200, #8N1
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

if (ser.isOpen()):
    ser.close()
ser.open()

# Create a log file
home = expanduser("~")
fName = home+"/catkin_ws/rosbags/novatelLog.txt"
outFile = open(fName, "wb")
# Send commands to CNS-5000 to start the logs
ser.write('unlogall\r\n')


#TODO write code that checks if INS_SOLUTION_GOOD or INS_HIGH_VARIANCE, if high variance maybe pause navigation and wait till solution good (might need a cap on how long you wait since sometimes the wait is over 2 minutes)

#these lines are no longer used, better to do kinematic alignment (drive around)
#setinitaz = input("type 1 to SETINITAZIMUTH, and 2 to skip it:")
#if (setinitaz == 1):
#  align = input("What directions  in degrees is the robot facing:")
#  command = 'SETINITAZIMUTH ' + str(align) + ' 10\r\n'
#  ser.write(command)

ser.write('LOG COM1 INSPVAA ONTIME 0.2\r\n')
#ser.write('LOG COM1 RAWIMUSA ONTIME 0.2\r\n')
#ser.write('LOG COM1 BESTGPSPOSA ONTIME 0.2\r\n')

# Start the ROS node and create the ROS publisher    
gpsPub = rospy.Publisher('gps/fix', NavSatFix, queue_size=1)
imuPub = rospy.Publisher('imu_data', Imu, queue_size=1)
novaPub = rospy.Publisher('raw_data', String, queue_size=1)
rospy.init_node('novatel_CNS5000', anonymous=True)
rate = rospy.Rate(15) # 10hz
try:
    while not rospy.is_shutdown(): 
 
        #ser.write('LOG COM1 INSPVAA ONCE \r\n')
        while ser.inWaiting() > 0:
            # While data is in the buffer
            kvh5000_output = ser.readline() # Read data a line of data from buffer
            outFile.write(kvh5000_output) # Option to log data to file
            #print(kvh5000_output)
            #novaPub = kvh5000_output
            #TODO print once when gets into different mode like initializing, finesteering, etc
                
            if (kvh5000_output.split(",")[0] == "#BESTGPSPOSA"): # check if this the gps message
                #print "Inside best gps"
                nova_Data = kvh5000_output.split(';')[1] # split the header and message body
                nova_Data = nova_Data.split(',') # split the message body into fields
                gps_out = NavSatFix()
                gps_out = parse_novatelGPS(nova_Data) # returns a NavSatFix message
                gpsPub.publish(gps_out) # publish the ROS message

            elif (kvh5000_output.split(",")[0] == "%RAWIMUSA"): # check if this the IMU message
                nova_Data = kvh5000_output.split(';')[1] # split the header and message body
                nova_Data = nova_Data.split(',') # split the message body into fields
                imu_out = parse_novatelIMU(nova_Data) 
                imuPub.publish(imu_out)

            elif (kvh5000_output.split(",")[0] == "#INSPVAA"): # check if this the INSPVA message
                nova_Header = kvh5000_output.split(';')[0]
                nova_Header = kvh5000_output.split(',')
                nova_Data = kvh5000_output.split(';')[1] # split the header and message body
                nova_Data = nova_Data.split(',') # split the message body into fields
                inspva_out = parse_novatelINSPVA(nova_Header, nova_Data) 
                gpsPub.publish(inspva_out[1])
                imuPub.publish(inspva_out[0])
                novaPub.publish(kvh5000_output)            
                     
except KeyboardInterrupt:
    ser.write('unlogall\r\n') # Send a message to CNS-5000 to stop sending logs
    outFile.close() 
    ser.close()
    raise


