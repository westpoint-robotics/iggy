#!/usr/bin/python
import serial
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
#from std_msgs.msg import Int16
from geometry_msgs.msg import Vector3


RCmode = 2

#print("here1")
# configure the serial connections 
try:
    ser = serial.Serial(
        port='/dev/ttyACM0',
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

# Create a log file
#outFile = open("roboteqLog.txt", "wb")
#print("here2")



estopCount = False


def getdata():
    info = ''
    while ser.inWaiting() > 0: # While data is in the buffer
        info += str(ser.read())
    return info

def makeCleanMsgOneLetter(message):
    cleanmsg = ''
    try:    
        for i in range(2,15):
            #print (message[i])
            if (message[i]== '\r'):
                cleanmsg = int(cleanmsg)
                return cleanmsg
            cleanmsg += message[i]
    except:
                #return 'error'                
        return int(10000000)
        
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
        
    #if cleanmsg.split 

def getEncoders():
    #print("here") 
    try:
        time.sleep(.01)
        getdata()   #clear buffer     
        ser.write('?C 1\r')  #enc left wheel
        time.sleep(.005)
        leftWheel = getdata()
        ser.write('?C 2\r')  #enc right wheel
        time.sleep(.005)
        rightWheel = getdata()
        print("enc = ", leftWheel, rightWheel) 
        leftWheel = makeCleanMsgOneLetter(leftWheel)
        rightWheel = makeCleanMsgOneLetter(rightWheel)
        #print("enc = ", leftWheel, rightWheel) 
    except: # catch *all* exceptions
        leftWheel = 10000000
        rightWheel = 10000000 
        print( "Error: getEncoders" )
    #print(leftWheel)
    return leftWheel, rightWheel

def getRCInput():
    try:
        time.sleep(.01)
        getdata()   #clear buffer     
        ser.write('?PI 3\r') #pulse input of channel 3 (rc3 button) (estop)
        time.sleep(.005)
        estopVal = getdata()
        ser.write('?PI 4\r') #pulse input of channel 4 (rc4 button) (switch)
        time.sleep(.005)
        switch = getdata()
        estopVal = makeCleanMsgTwoLetters(estopVal)
        switch = makeCleanMsgTwoLetters(switch)
    except: # catch *all* exceptions
        print( "Error: getRCInput" )
        estopVal = 1000
        switch = 1900

    return estopVal, switch


def moveWheels(speed):  #not currently in use
    try:
        for i in range(1000):
            ser.write('!G 1 1000\r') ##only says go forward full speed, doesnt use speed input
            time.sleep(.010)
    except: # catch *all* exceptions
        print( "Error: moveWheels" )


def moveCallback(data):
    global estopCount
    global RCmode
    #print('im here')
    RCVals = getRCInput()
    estopValue = RCVals[0]
    switchValue = RCVals[1]
    if (estopValue > 1500):  #estop button pushed
        ser.write('!EX\r')
        estopCount = True
        RCmode = 3
        #print(estopValue)
    elif (estopCount == True):
        ser.write('!MG\r')
        estopCount = False
        #print('switch back on')
        RCmode = 4
    else:
        if (switchValue > 1500):  #switch in RC mode
            #do RC commands
            #print ('RC things')
            RCmode = 1
        else:                     
            #print('sending command')
            #print('comp things')
            #print(switchValue)
            if (abs(data.linear.x) > 0.001 or abs(data.angular.z) > 0.001):
                #rospy.loginfo("I heard %f %f",data.linear.x,data.angular.z)
                speed = data.linear.x *500 #linear.x is value between -1 and 1 and input to wheels is between -1000 and 1000
                                            #1000 would give full speed range, but setting to lower value to better control robot
                turn = (data.angular.z + 0.009)*400  #angular.z value is between -0.5 and 0.5 and input to wheels is between -1000 and 1000
                                                     #2000 would give full speed range, but setting to lower value to better control robot
                                                     #through testing found 0.009 to be the approximate offset in the wheels
                                                     #the wheels are balanced improperly and so a true number is hard to find
                                                     #since based on the incline the robot is facing, the value changes dramatically
           #     if (turn > 100):
           #         turn = 100
           #     elif (turn < -100):
           #         turn = -100
           #     if (speed > 500):
           #         speed = 500
           #     elif (speed < -500):
           #         speed = -500
                #print(speed,turn)
                cmd = '!G 1 ' + str(speed) + '\r'
                ser.write(cmd)
                #getdata()
                #print(cmd)
                cmd = '!G 2 ' + str(turn) + '\r'
                ser.write(cmd)
                #getdata()
                #print(cmd)
                RCmode = 0
       


#def valsToOdom(encVals):
#    leftenc = encVals[0]
#    rightenc = encVals[1]
#    odom_mes = Odometry()
#    return odom_mes


if __name__ == '__main__':
    rospy.init_node('igvc_roboteq', anonymous=True)
    #print('hello world')
    pub = rospy.Publisher("enc_raw", Vector3, queue_size=1) 
    rospy.Subscriber("roboteq_driver/cmd", Twist, moveCallback)

    try:
        #print('try.. try again')
        rate = rospy.Rate(3)
        #encodermsg = Vector3()
        while not rospy.is_shutdown():
            #print('here234')
            #enclist = getEncoders()
            #if (enclist[0] == 10000000 or enclist[1] == 10000000 or enclist[0] == None or enclist[1] == None):
                #print ('error happened')
            #    pass
           # else:
                #odom_msg = valsToOdom(encoders)
                
                #encodermsg.x = enclist[0]
                #encodermsg.y = enclist[1]
                #encodermsg.z = RCmode
                #pub.publish(encodermsg)
                #print(odom_msg)            
                #print (encoders)
                #moveCallback()
                #look at move subcriber, if not empty, move = true
            rate.sleep()



    except KeyboardInterrupt:
        ser.close()
        raise











