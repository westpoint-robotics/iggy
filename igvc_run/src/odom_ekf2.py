#!/usr/bin/env python

""" odom_ekf.py - Version 0.1 2012-07-08

    Republish the /robot_pose_ekf/odom_combined topic which is of type 
    geometry_msgs/PoseWithCovarianceStamped as an equivalent message of
    type nav_msgs/Odometry so we can view it in RViz.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovariance

#globals



class OdomEKF():


    def __init__(self):
        # Give the node a name
        rospy.init_node('odom_ekf', anonymous=False)
        self.timeNow = rospy.get_rostime()
        self.data2 = TwistWithCovariance()

        # Publisher of type nav_msgs/Odometry
        self.ekf_pub = rospy.Publisher('odometry_final', Odometry, queue_size=1)
        
        # Wait for the /odom topic to become available
        rospy.wait_for_message('robot_pose_ekf/odom_combined', PoseWithCovarianceStamped)
        
        # Subscribe to the /robot_pose_ekf/odom_combined topic
        rospy.Subscriber('imu/data', PoseWithCovarianceStamped, self.callback)
        rospy.Subscriber('robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.pub_ekf_odom)
        

        #rospy.loginfo("Publishing combined odometry on /odom_ekf")
        
    def pub_ekf_odom(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = 'utm'
        odom.pose = msg.pose
        odom.twist = self.data2
        odom.twist.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        
        self.ekf_pub.publish(odom)
        
    def callback(self, data):
        lastTime = self.timeNow
        self.timeNow = rospy.get_rostime()
        deltaTime = Duration(self.timeNow - lastTime)
        self.data2.linear.x = data.linear_acceleration.x*deltaTime
        self.data2.linear.y = data.linear_acceleration.y*deltaTime
        self.data2.linear.z = data.linear_acceleration.z*deltaTime
        self.data2.angular = data.angular_velocity




if __name__ == '__main__':
    #try:
        OdomEKF()
        rospy.spin()
    #except:
    #    print('error')
    #    pass
        

