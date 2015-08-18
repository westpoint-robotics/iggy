#!/usr/bin/python
import serial
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from geometry_msgs.msg import Vector3



#print("here1")
# configure the serial connections 
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200, #8N1
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

if (ser.isOpen()):
    ser.close()
ser.open()

# Create a log file
#outFile = open("roboteqLog.txt", "wb")
#print("here2")

def getdata():
    info = ''
    while ser.inWaiting() > 0: # While data is in the buffer
        info += str(ser.read())
    return info

def makeCleanMsg(message):
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
        

    #if cleanmsg.split 

def getEncoder():
    try:
        time.sleep(.01)
        getdata()   #clear buffer     
        ser.write('?C 1\r')
        time.sleep(.005)
        leftWheel = getdata() 
        ser.write('?C 2\r')      
        time.sleep(.005)
        rightWheel = getdata()
    except: # catch *all* exceptions
        e = sys.exc_info()[0]
        print( "<p>Error: %s</p>" % e )
    leftWheel = makeCleanMsg(leftWheel)
    rightWheel = makeCleanMsg(rightWheel)
    #print(leftWheel)
    return leftWheel, rightWheel

def moveWheels(speed):
    try:
        for i in range(1000):
            ser.write('!G 1 1000\r')
            time.sleep(.010)
    except: # catch *all* exceptions
        e = sys.exc_info()[0]
        print( "<p>Error: %s</p>" % e )
    #return leftWheel, rightWheel


def moveCallback(data):
    if (abs(data.linear.x) > 0.01 or abs(data.angular.z) > 0.01):
        #rospy.loginfo("I heard %f %f",data.linear.x,data.angular.z)
        speed = data.linear.x *2000
        turn = data.angular.z *1000
        #print(speed,turn)
        cmd = '!G 1 ' + str(speed) + '\r'
        ser.write(cmd)
        #getdata()
        #print(cmd)
        cmd = '!G 2 ' + str(turn) + '\r'
        ser.write(cmd)
        #getdata()
        #print(cmd)

#def valsToOdom(encVals):
#    leftenc = encVals[0]
#    rightenc = encVals[1]
#    odom_mes = Odometry()
#    return odom_mes


if __name__ == '__main__':
    rospy.init_node('igvc_roboteq', anonymous=True)
    rospy.Subscriber("roboteq_driver/cmd", Twist, moveCallback)
    pub = rospy.Publisher("enc_raw", Vector3, queue_size=1) 
    try:
        rate = rospy.Rate(1)
        encodermsg = Vector3()        
        while not rospy.is_shutdown():
            enclist = getEncoder()
            if (enclist[0] == 10000000 or enclist[1] == 10000000 ):
                #print ('error happened')
                pass
            else:
                #odom_msg = valsToOdom(encoders)
                encodermsg.x = enclist[0]
                encodermsg.y = enclist[1]
                pub.publish(encodermsg)
                #print(odom_msg)            
                #print (encoders)
                #moveCallback()
                #look at move subcriber, if not empty, move = true
            rate.sleep()



    except KeyboardInterrupt:
        outFile.close() 
        ser.close()
        raise


