#!/usr/bin/env python

""" Code modified by: Dominic Larkin 5FEB2016
    
    Command a robot to move autonomously among a number of goal locations defined in the map frame.
    Attempt to move to each location in succession.  Keep track of success rate, time elapsed, and 
    total distance traveled. The goal locations are determined by adding distances to the current
    location of the robot on the map.
    based on nav_test.py - Version 0.1 2012-ub_IGVC.py01-10. Modified for use with West Point Robotics.
    Originaly created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    Original License:
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
"""

import roslib;
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry 
from random import sample
from math import pow, sqrt

class NavTest():
    def __init__(self):
        # A list of goal coordinates to move to. This assumes (0,0) is the coordinate
        # that the robot started at.
        deltas = [(-0.5,0),(0.5,0)]

        rospy.init_node('nav_test', anonymous=True)        
        rospy.on_shutdown(self.shutdown)        

        # A variable to hold the initial pose of the robot
        self.initial_pose = Odometry()
        self.current_pose = Odometry()
        
        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 10)
        
        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']        
        
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))        
        rospy.loginfo("Connected to move base server")        
        # Make sure we have the initial pose
        rospy.loginfo("Waiting for initial pose")
        rospy.Subscriber('/odometry/filtered', Odometry, self.update_current_pose)
        while self.initial_pose.header.stamp == "":
            # Get the initial pose from the robot
            rospy.loginfo("Trying to subscribe...")
            rospy.wait_for_message('/odometry/filtered', Odometry)
            
            rospy.sleep(1) 
        rospy.sleep(.25) 
        self.initial_pose = self.current_pose      
        rospy.loginfo("Initial pose found at (%.4f,%.4f)" %(self.initial_pose.pose.pose.position.x,self.initial_pose.pose.pose.position.y))

        # Calculating goal positions based on initial position.
        locations = []        
        for delta_x,delta_y in deltas:
            goal_pose = Pose()
            goal_pose.position.x = self.initial_pose.pose.pose.position.x + delta_x
            goal_pose.position.y = self.initial_pose.pose.pose.position.y + delta_y
            goal_pose.orientation = self.initial_pose.pose.pose.orientation                        
            locations.append(goal_pose)

        # Variables to keep track of success rate, running time,
        # and distance traveled
        n_locations = len(locations)
        n_goals = 0
        n_successes = 0
        i = n_locations
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0

        # Begin the main loop and run through a sequence of locations
        i = 0
        while not rospy.is_shutdown():
                        
            # Keep track of the distance traveled.
            if i != 0:
                distance = sqrt(pow(locations[i].position.x - 
                                    locations[i-1].position.x, 2) +
                                pow(locations[i].position.y - 
                                    locations[i-1].position.y, 2))
            else:
                distance = sqrt(pow(locations[i].position.x - 
                                    self.initial_pose.pose.pose.position.x, 2) +
                                pow(locations[i].position.y - 
                                    self.initial_pose.pose.pose.position.y, 2))         
        
            # Set up the next goal location
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = locations[i]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            
            # Let the user know where the robot is going next
            rospy.loginfo("Going to: (%.4f,%.4f)" %(self.goal.target_pose.pose.position.x,self.goal.target_pose.pose.position.y))
            
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)
            
            # Allow 5 minutes to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 
            
            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    n_successes += 1
                    distance_traveled += distance
                    rospy.loginfo("State:" + str(state))
                else:
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
            
            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0
            
            # Print a summary success/failure, distance traveled and time elapsed
            n_goals += 1
            rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
                          str(n_goals) + " = " + 
                          str(100 * n_successes/n_goals) + "%")
            rospy.loginfo("Running time: %.2f min Distance: %.3f"  % (running_time,distance_traveled))
               
            # Increment the counters
            i += 1
            if (i >= len(locations)):
                rospy.signal_shutdown("NO MORE GOALS TO ACHIEVE") 

            rospy.sleep(self.rest_time)
            
    def update_current_pose(self, current_pose):
        rospy.loginfo("Current Pose is: (%.4f,%.4f)" %((current_pose.pose.pose.position.x),(current_pose.pose.pose.position.y)))        
        self.current_pose = current_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

