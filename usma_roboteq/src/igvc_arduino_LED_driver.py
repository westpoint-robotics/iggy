#!/usr/bin/python
import serial
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
#from std_msgs.msg import Int16
from geometry_msgs.msg import Vector3

#lastTime = rospy.Time() # secs=nsecs=0
#lastTime = rospy.get_rostime()

#print("here1")
# configure the serial connections 
try:
    ser = serial.Serial(
        port='/dev/ttyACM1',
        baudrate=9600, #8N1
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
except:
    raise
#    try:
#        ser = serial.Serial(
#            port='/dev/ttyACM1',
#            baudrate=9600, #8N1
#            parity=serial.PARITY_NONE,
#            stopbits=serial.STOPBITS_ONE,
#            bytesize=serial.EIGHTBITS
#        )
#    except:
#        raise

if (ser.isOpen()):
    ser.close()
ser.open()


lastTime = rospy.Time()

def moveCallback(data):
    global lastTime
    #print(data.z)
    #print(data.y)
    timenow = rospy.get_rostime()
#    global lastTime #
    if (timenow - lastTime > rospy.Duration(.0001)):
        lastTime = timenow    
        switch = data.z
        if (switch == 1): 
            ser.write('a')
            #print('a')
        elif (switch ==0):
            ser.write('b')
            #print('b')
        else:
            ser.write('c')
            print("error, switch val recieved from roboteq_driver: ", switch, "no val sent arduino")

    

if __name__ == '__main__':
    try:
        rospy.init_node('igvc_arduino', anonymous=True)
        #print('hello world')
        lstTime = rospy.get_time()
        rospy.Subscriber("enc_raw", Vector3, moveCallback)
        rospy.spin()

    except KeyboardInterrupt:
        ser.close()
        raise











