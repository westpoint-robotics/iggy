#!/usr/bin/python
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
#from std_msgs.msg import Int16
from geometry_msgs.msg import Vector3



if __name__ == '__main__':
    rospy.init_node('igvc_roboteq', anonymous=True)
    #print('hello world')
    pub = rospy.Publisher("enc_raw", Vector3, queue_size=1) 
    flipcount = 0

    try:
        #print('try.. try again')
        rate = rospy.Rate(1)
        encodermsg = Vector3()
        encodermsg.x = 0  #left
        encodermsg.y = 0  #right
        while not rospy.is_shutdown():
            flipcount += 1
            if (flipcount > 29):  #turn left
                encodermsg.x += 6.0
                encodermsg.y += 13.0
                #print ("x + 6, y + 13")
                flipcount = 0
            elif (flipcount > 20):  #turn left
                encodermsg.x += 6.0
                encodermsg.y += 13.0
                #print ("x + 6, y + 13")
            elif (flipcount > 10): #go straight 
                encodermsg.x += 9.0
                encodermsg.y += 9.0
                #print ("x + 9, y + 9")
            else:                  #turn right
                encodermsg.x += 13.0
                encodermsg.y += 6.0
                #print ("x + 13, y + 6")
            #print('here234')
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











