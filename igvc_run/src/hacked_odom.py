#!/usr/bin/python

import rospy
import csv
import roslib
import actionlib
import time
import tf
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from random import sample
from math import pow, sqrt
from LatLongUTMconversion import LLtoUTM, UTMtoLL
from std_msgs.msg import Float32
import math

global finalOdom
global initialPose

def gps_callback(gps):
	global initialPose
	if (initialPose == -1):
		initialPose = LLtoUTM(23, gps.latitude, gps.longitude)
	else:
		thisPose = LLtoUTM(23, gps.latitude, gps.longitude)
		finalOdom.pose.pose.position.x = thisPose[1] - initialPose[1]
		finalOdom.pose.pose.position.y = thisPose[2] - initialPose[2]
		finalOdom.pose.pose.position.z = 0
		
def virtual_callback(virtual):
	global finalOdom
	finalOdom.twist.twist.linear.x = virtual.twist.twist.linear.x
	
def imu_callback(imu):
	global finalOdom
	finalOdom.twist.twist.angular.z = imu.angular_velocity.z

def heading_callback(heading):
	global finalOdom
	heading.data = ((heading.data+360)-90)%360
	q = tf.transformations.quaternion_from_euler(0, 0, math.radians(heading.data))
	finalOdom.pose.pose.orientation = Quaternion(*q)

if __name__ == "__main__":
	global finalOdom
	global initialPose

	initialPose = -1
	finalOdom = Odometry()
	finalOdom.header.frame_id = "odom"
	finalOdom.child_frame_id = "base_footprint"

	rospy.init_node('hacked_odom', anonymous=False)

	rospy.Subscriber('/gps/fix', NavSatFix, gps_callback)
        rospy.Subscriber('/odom/virtual', Odometry, virtual_callback)
	rospy.Subscriber('/imu/raw', Imu, imu_callback)
	rospy.Subscriber('/xsens/heading', Float32, heading_callback)
        pub = rospy.Publisher('/odom', Odometry, queue_size=1)
	tfSend = tf.TransformBroadcaster()

	while not rospy.is_shutdown():
		finalOdom.header.stamp = rospy.Time.now()
		tfSend.sendTransform((finalOdom.pose.pose.position.x, finalOdom.pose.pose.position.y,0),
			tf.transformations.quaternion_from_euler(0, 0, finalOdom.pose.pose.orientation.z),
			rospy.Time.now(),
			"base_footprint",
			"odom")
		pub.publish(finalOdom)

	rospy.spin()
