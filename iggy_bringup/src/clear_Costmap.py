#!/usr/bin/env python
import roslib
import rospy
import actionlib
import time
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Empty
import std_srvs.srv

rospy.init_node('cosmap_clearer', anonymous=True)
clear_msg = Empty()
# THIS DOES NOT WORK. Still need to figure out what this next line should be.
# You can use command line: rosservice call /move_base/clear_costmap
move_base = actionlib.SimpleActionClient("/move_base/clear_costmaps", std_srvs.srv.Empty)
move_base.wait_for_server(rospy.Duration(60))
move_base.clear_costmaps()
