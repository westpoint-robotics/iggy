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

'''
udev rule settings for the CNS 5000 and flexpak6
SUBSYSTEM=="tty", ATTRS{idProduct}=="2303", ATTRS{idVendor}=="067b", ATTRS{product}=="USB-Serial Controller", SYMLINK+="raw_imu"
SUBSYSTEM=="tty", ATTRS{idProduct}=="2303", ATTRS{idVendor}=="067b", ATTRS{product}=="USB-Serial Controller D", SYMLINK+="raw_gps"
SUBSYSTEM=="tty", ATTRS{idProduct}=="0100", ATTRS{idVendor}=="09d7", SYMLINK+="flex6_gps"
'''
# TODO rename these devices. raw_gps is not accurate name, this the cns5000 ins device.
ser = serial.Serial(
    port='/dev/raw_gps',
    #baudrate=115200, #8N1
    baudrate=9600, #8N1
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

if (ser.isOpen()):
    ser.close()
ser.open()

timestr = time.strftime("%Y_%m_%d_%H_%M_%S")
# Create a log file
home = expanduser("~")
fName = home+"/Data/novatelLog"+timestr+".txt"
outFile = open(fName, "wb")

# Send commands to CNS-5000 to start the logs
ser.write('unlogall\r\n')
time.sleep(0.03)
ser.write('ASSIGNLBANDBEAM AUTO\r\n')
time.sleep(0.03)
#ser.write('VEHICLEBODYROTATION 0 0 180\r\n')
#time.sleep(0.03)
#ser.write('APPLYVEHICLEBODYROTATION ENABLE\r\n')
#time.sleep(0.03)
ser.write('LOG COM1 INSPVAA ONTIME 0.5\r\n')
time.sleep(0.03)
ser.write('LOG COM1 BESTPOSA ONTIME 1\r\n')
time.sleep(0.03)
ser.write('LOG COM1 PPPPOSA ONTIME 1\r\n')
time.sleep(0.03)
ser.write('LOG COM1 GPGSA ONTIME 5\r\n')
time.sleep(0.03)
ser.write('LOG COM1 GPGSV ONTIME 5\r\n')
time.sleep(0.03)

#TODO write code that checks if INS_SOLUTION_GOOD or INS_HIGH_VARIANCE, if high variance pause navigation and wait till solution good (might need a cap on how long you wait since sometimes the wait is over 2 minutes)

#these lines are no longer used, better to do kinematic alignment (drive around)
#setinitaz = input("type 1 to SETINITAZIMUTH, and 2 to skip it:")
#if (setinitaz == 1):
#  align = input("What directions  in degrees is the robot facing:")
#  command = 'SETINITAZIMUTH ' + str(align) + ' 10\r\n'
#  ser.write(command)

# Start the ROS node and create the ROS publisher    
gpsPub = rospy.Publisher('fix', NavSatFix, queue_size=1)
# The below line is not needed in current config. We are using raw imu from another serial connection.
imuPub = rospy.Publisher('inspvaa_imu_data', Imu, queue_size=1)
novaPub = rospy.Publisher('raw_data', String, queue_size=1)
inspva_gpsPub = rospy.Publisher('inspva_fix', NavSatFix, queue_size=1)
rospy.init_node('kvh_cns5000', anonymous=True)
rate = rospy.Rate(2) 
try:
    while not rospy.is_shutdown():  
        while ser.inWaiting() > 0:
            # While data is in the buffer
            kvh5000_output = ser.readline() # Read data a line of data from buffer
            outFile.write(kvh5000_output) # Option to log data to file
            #print(kvh5000_output)
            novaPub.publish(kvh5000_output)            
            #TODO print once when gets into different mode like initializing, finesteering, etc
            if (kvh5000_output.split(",")[0] == "#BESTPOSA"): # check if this the gps message
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
                inspva_gpsPub.publish(inspva_out[1])
                #imuPub.publish(inspva_out[0])

	rate.sleep()
                     
except KeyboardInterrupt:
    ser.write('unlogall\r\n') # Send a message to CNS-5000 to stop sending logs
    outFile.close() 
    ser.close()
    raise



