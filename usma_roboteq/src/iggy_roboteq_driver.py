#!/usr/bin/python

'''
The controller uses a simple communication protocol based on ASCII characters. Com-
mands are not case sensitive. ?a is the same as ?A. Commands are terminated by car-
riage return (Hex 0x0d, ‘eschar r’).
The underscore ‘_’ character is interpreted by the controller as a carriage return. This alter-
nate character is provided so that multiple commands can be easily concatenated inside a
single string.
All other characters lower than 0x20 (space) have no effect.
The controller will echo back to the PC or Microcontroller every valid character it has re-
ceived. If no echo is received, one of the following is occurring
The controller will acknowledge commands in one of the two ways:
For commands that cause a reply, such as a configuration read or a speed or amps que-
ries, the reply to the query must be considered as the command acknowledgment.
For commands where no reply is expected, such as speed setting, the controller will
issue a “plus” character (+) followed by a Carriage Return after every command as an ac-
knowledgment.
If a command or query has been received, but is not recognized or accepted for any rea-
son, the controller will issue a “minus” character (-) to indicate the error.

The general format for setting a parameter is the “^” character followed by the command
name followed by parameter(s) for that command. These will set the parameter in the con-
troller’s RAM and this parameter becomes immediately active for use. The parameter can
also be permanently saved in EEPROM by sending the %EESAV maintenance command.


'''
import serial
import rospy
import time
import os
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovarianceStamped
#from nav_msgs.msg import Odometry
#from std_msgs.msg import Int16
from geometry_msgs.msg import Vector3

# global variables
lastSwitchVal = 0
switchValue = 0
RCmode = 2
estopCount = False

# Create a log file
#outFile = open("roboteqLog.txt", "wb")

def getdata():
    info = ''
    while ser.inWaiting() > 0: # While data is in the buffer
        info += str(ser.read())
    return info

def makeCleanMsgTwoLetters(message):
    cleanmsg = ''
    try:    
        for i in range(3,15):
            #print (message[i])
            if (message[i]== '\r'):
                cleanmsg = int(cleanmsg)
                return cleanmsg
            cleanmsg += message[i]
    except:
                #return 'error'                
        return int(10000000)

def getRCInput():
    try:
        time.sleep(.01)
        getdata()   #clear buffer   
        # PI - Read Pulse Inputs  
        ser.write('?PI 3\r') #pulse input of channel 3 (rc3 button) (estop)
        time.sleep(.005)
        estopVal = getdata()
        ser.write('?PI 4\r') #pulse input of channel 4 (rc4 button) (switch)
        time.sleep(.005)
        switch = getdata()
        estopVal = makeCleanMsgTwoLetters(estopVal)
        switch = makeCleanMsgTwoLetters(switch)
    except Exception as e: # catch *all* exceptions
        print e
        print( "Error: getRCInput" )
        estopVal = 1000
        switch = 1900
    return estopVal, switch

def moveWheels(speed):  #not currently in use
    try:
        for i in range(1000):
            # G - Go to Speed or to Relative Position
            ser.write('!G 1 1000\r') ##only says go forward full speed, doesnt use speed input
            time.sleep(.010)
    except: # catch *all* exceptions
        print( "Error: moveWheels" )

def moveCallback(data):
    global estopCount
    global RCmode
    global lastSwitchVal
    global switchValue

    pub_covariance = TwistWithCovarianceStamped()
    pub_covariance.twist.twist.linear.x = data.linear.x
    pub_covariance.twist.twist.angular.z = data.angular.z
    pub2.publish(pub_covariance)
    estopValue = 0
    #print('im here')
    if (estopValue > 1500):  #estop button pushed
        # EX - Emergency Stop
        ser.write('!EX\r')
        estopCount = True
        RCmode = 3
        #print(estopValue)

    elif (estopCount == True):
        # MG - Emergency Stop Release
        ser.write('!MG\r')
        estopCount = False
        #print('switch back on')
        RCmode = 4
    else:
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

def initalizeController():
    #TODO Set priority to 2 to prefer RC Pulse over serail
    # READ ?FID to get firmware and controller information
    # ^ECHOF nn
    # ?V GET BATTERY VOLTS  internal:battery:fivevolts
    # Check if Pulse In 1 to 4 are enabled
    cmd = '~PMOD\r' 
    pmod = ser.write(cmd)
    


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

    rospy.init_node('igvc_roboteq', anonymous=True)
    pub2 = rospy.Publisher("roboteq_driver/cmd_with_covariance", TwistWithCovarianceStamped, queue_size=1) 
    lights = rospy.Publisher("/lights", Bool, queue_size=1)
    auto = rospy.Publisher("/autonomous", Bool, queue_size=1)
    rospy.Subscriber("roboteq_driver/cmd", Twist, moveCallback)
   
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        try:
            RCVals = getRCInput()
            estopValue = RCVals[0]
            switchValue = RCVals[1]

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
