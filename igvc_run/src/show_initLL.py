#!/usr/bin/env python
""" Code modified by: Dominic Larkin 5FEB2016
    
	Get initial pose using the igvc 2016 averaging solution
"""
import csv
import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry 
from random import sample
from math import pow, sqrt
from LatLongUTMconversion import LLtoUTM
from goal_pub_IGVC import NavTest

global nav
nav = NavTest()
utm=nav.setInitialPose()
print UTMtoLL(23, utm[2], utm[1], utm[0])
