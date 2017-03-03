#!/usr/bin/python

'''

SYMBOLS USED FOR SERIAL COMMO:
~ Read configuration operation 
^ Write configuration operation 
! Run time commands
? Run time queries
% Maintenance Commans
_ Carriage return

+ Command acknowledgement
- Command not recognized or accepted

Commands are terminated by carriage return (Hex 0x0d, ‘eschar r’)

'''
import serial
import rospy
import time
import os
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import Vector3

# global variables
lastSwitchVal = 0
switchValue = 0
RCmode = 2

# Create a log file
#outFile = open("roboteqLog.txt", "wb")

# While data is in the buffer read it, then return it.
def getdata():
    info = ''
    while ser.inWaiting() > 0: 
        info += str(ser.read())
    return info

def getRCInput():
    try:
        getdata()   #clear buffer   
        # PI - Read Pulse Inputs  
        ser.write('?PI 4\r') #pulse input of channel 4 (rc4 button) (switch)
        switch = getdata()

    except Exception as e: # catch *all* exceptions
        print e
        print( "Error: getRCInput" )
        switch = 1900
    return switch

def moveCallback(data):

    global RCmode
    global lastSwitchVal
    global switchValue

    # 
    pub_covariance = TwistWithCovarianceStamped()
    pub_covariance.twist.twist.linear.x = data.linear.x
    pub_covariance.twist.twist.angular.z = data.angular.z
    pub2.publish(pub_covariance)

    #print('im here')
    if (switchValue > 1500):  #switch in RC mode
            RCmode = 1
    else:
        if (abs(data.linear.x) > 0.001 or abs(data.angular.z) > 0.001):
            #rospy.loginfo("I heard %f %f",data.linear.x,data.angular.z)
            speed = data.linear.x *1000 #linear.x is value between -1 and 1 and input to wheels is between -1000 and 1000
                                        #1000 would give full speed range, but setting to lower value to better control robot
            turn = (data.angular.z + 0.009)*500*-1 
            # G - Go to Speed or to Relative Position
            cmd = '!G 1 ' + str(speed) + '\r'
            ser.write(cmd)
            #print(cmd)
            cmd = '!G 2 ' + str(turn) + '\r'
            ser.write(cmd)
            #print(cmd)
            RCmode = 0

# Configures the roboteq motor controller to work with Iggy. Relying on the EEPROM has proven unreliable.
def initalizeController():
    # Below are the initial configurations for the Roboteq motor controller required by Iggy. See RoboteqIggySettings.pdf.
    configCmds=['^CPRI 1 1\r',      '^CPRI 2 0\r',      '^OVL 350\r',       '^UVL 180\r',
                '^MAC 1 20000\r',   '^MAC 2 20000\r',   '^MXRPM 1 3500\r',  '^MXRPM 2 3500\r',
                '^MXMD 1\r',        '^PMOD 0 1\r',      '^PMOD 5 0\r']

    # Send commands to Roboteq and exit if any fail
    for cmd in configCmds:
        ser.write(cmd)
        time.sleep(.01)
        result = getdata()
        if (result != '+\r'):
            print "ERROR: ROBOTEQ DRIVER FAILED TO SET CONFIGURATION WITH: ", cmd,"\n"
            print "ERROR: ROBOTEQ DRIVER CONFIGURATION FAILED. NOW EXITING ROBOTEQ DRIVER\n\n"
            exit()
        print "SUCCESSFULLY CONFIGURED THE ROBOTEQ MOTOR CONTROLLER\n"
    


    pass

if __name__ == "__main__":

    lastSwitchVal = 0
    switchValue = 0

    # configure the serial connections 
    try:
        ser = serial.Serial(
            port='/dev/roboteq',
            baudrate=115200, #8N1
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
    except:
        raise
        try:
            ser = serial.Serial(
                port='/dev/ttyACM1',
                baudrate=115200, #8N1
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
        except:
            raise

    if (ser.isOpen()):
        ser.close()
    ser.open()

    #TODO rename to iggy_roboteq and align the namespace
    rospy.init_node('igvc_roboteq', anonymous=True)
    pub2 = rospy.Publisher("roboteq_driver/cmd_with_covariance", TwistWithCovarianceStamped, queue_size=1) 
    lights = rospy.Publisher("/lights", Bool, queue_size=1)
    auto = rospy.Publisher("/autonomous", Bool, queue_size=1)
    rospy.Subscriber("roboteq_driver/cmd", Twist, moveCallback)

    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        try:
            switchValue = getRCInput()

            if (lastSwitchVal != switchValue/100):
                lastSwitchVal = switchValue/100
                if (switchValue > 1500):  #switch in RC mode
                    lights.publish(False)
                else:    
                    lights.publish(True) 
            else:
                if (switchValue > 1500):
                    auto.publish(False)
                else:
                    auto.publish(True)             

        except KeyboardInterrupt:
            ser.close()
            raise
        except: # catch *all* exceptions
            print( "Error in roboteq Driver: Some other exception, RESTART THE WHOLE ROBOT IF THIS MESSAGE KEEPS APPEARING" )
        rate.sleep()
