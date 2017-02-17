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



# Start the ROS node and create the ROS publisher    
gpsPub = rospy.Publisher('gps/fix', NavSatFix, queue_size=1)
imuPub = rospy.Publisher('imu_data', Imu, queue_size=1)
#novaPub = rospy.Publisher(???,???, queue_size=1)
rospy.init_node('novatel_CNS5000', anonymous=True)
rate = rospy.Rate(5) # 10hz
with open("/home/user1/box_inspvaa_raw.csv") as insData:
    try:
        while not rospy.is_shutdown():                  
            kvh5000_output = insData.readline() # Read data a line of data from buffer
            #outFile.write(kvh5000_output) # Option to log data to file
            print(kvh5000_output)
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

            elif (kvh5000_output.split(",")[0] == "[COM1]#INSPVAA"): # check if this the INSPVA message
                nova_Data = kvh5000_output.split(';')[1] # split the header and message body
                nova_Data = nova_Data.split(',') # split the message body into fields
                inspva_out = parse_novatelINSPVA(nova_Data) 
                gpsPub.publish(inspva_out[1])
                imuPub.publish(inspva_out[0])            
            rate.sleep()
    except KeyboardInterrupt:
        ser.write('unlogall\r\n') # Send a message to CNS-5000 to stop sending logs
        #outFile.close() 
        ser.close()
        raise


