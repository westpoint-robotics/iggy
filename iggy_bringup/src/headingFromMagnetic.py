#!/usr/bin/env python

""" 


"""
import math
import rospy
import tf
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Twist, Vector3Stamped
from nav_msgs.msg import Odometry

class MagHeading():
    def __init__(self):
        # Give the node a name
        rospy.init_node('magnetic_heading', anonymous=False)

        # Publisher of type nav_msgs/Odometry
        self.magHdg_pub = rospy.Publisher('/imu/mag', Float32, queue_size=1)
        
        # Subscribe to the gps positions
        rospy.Subscriber('/phidget/imu/mag', Vector3Stamped, self.update_magnetic_callback)
        self.mHeading=Vector3Stamped()

    def update_magnetic_callback(self, msg):
        self.mHeading = msg
        x=msg.vector.x
        y=msg.vector.y
        heading=Float32()
        # TODO add eqaution here
        heading=math.atan2(y,x)*(180/math.pi)
        if heading<0:
            heading = heading+360
        heading=heading+180
	heading=(51.1*math.exp(0.0057*heading))%360
        self.magHdg_pub.publish(heading)
        
if __name__ == '__main__':
        mHead=MagHeading()
        rospy.spin()

        

