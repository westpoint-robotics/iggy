#!/usr/bin/env python

""" Code modified by: Dominic Larkin 21APR2017    

example of usage: rosrun igvc_run goal_pub_IGVC.py _waypoint_file:=igvc_run/src/waypoints3.csv

"""
import csv
import roslib
import rospy
import actionlib
import time
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry 
from random import sample
from math import pow, sqrt
from LatLongUTMconversion import LLtoUTM, UTMtoLL
from marker_pub import make_waypoint_viz
from visualization_msgs.msg import Marker
from copy import deepcopy

class Odom_gps():

    def __init__(self):     
        rospy.init_node('odom_gps', anonymous=True)        

        rospy.Subscriber('fix', NavSatFix, self.update_utm_cb)
        self.odom_pub = rospy.Publisher('odom/gps', Odometry, queue_size=1) 
        self.current_utm = [] # return (UTMZone, UTMEasting, UTMNorthing)
        self.initial_utm = [] # return (UTMZone, UTMEasting, UTMNorthing)    

    def setInitialPose(self):
        # Get the initial pose from the robot
        rospy.loginfo("Establishing initial position wait 10 seconds. Do not move the robot.")
        easts=[]
        nrths=[]
        i=0
        #time.sleep(2)
        if (len(self.current_utm)==3):
            for i in range(3): #TODO move back to 10
                utm_c=self.current_utm
                easts.append(utm_c[1]) #TODO add check for empty array
                nrths.append(utm_c[2])
                rospy.sleep(1.0)
            easting = sum(easts) / float(len(easts))
            northing = sum(nrths) / float(len(nrths))
            self.initial_utm = [utm_c[0],easting,northing]
            rospy.loginfo("done establishing initial position.It is: %f, %f",self.initial_utm[1],self.initial_utm[2])
        else: 
            print "WAITING FOR INITIAL LOCATION"

    def mainloop(self):

        frame_id = rospy.get_param("~frame_id", "odom")
        child_frame = rospy.get_param("~child_frame_id", "base_link")
        rospy.loginfo("GPS to Odom frame ID is: %s and Child frame ID is: %s",frame_id,child_frame)
        rate = rospy.Rate(30) # 30hz
        # Begin the main loop 
        odom_msg=Odometry()
        while not rospy.is_shutdown():
            if (len(self.initial_utm) < 3):
                self.setInitialPose() # In the odom frame. 
            else:
                odom_msg.header.frame_id=frame_id
                odom_msg.header.stamp= rospy.Time.now()
                odom_msg.child_frame_id = child_frame
                odom_msg.pose.pose.position.x=self.current_utm[1]-self.initial_utm[1]
                odom_msg.pose.pose.position.y=self.current_utm[2]-self.initial_utm[2]
                #rospy.loginfo("Odom is: %f, %f -- Initial is: %f, %f ---  Current is: %f, %f ",odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y,self.initial_utm[1],self.initial_utm[2],self.current_utm[1],self.current_utm[2])
                odom_msg.pose.pose.orientation.x=0
                odom_msg.pose.pose.orientation.y=0
                odom_msg.pose.pose.orientation.z=0
                odom_msg.pose.pose.orientation.w=1.0
                odom_msg.pose.covariance=[1e-10,       0,       0,       0,       0,       0,
                                             0,    1e-10,       0,       0,       0,       0,
                                             0,        0,   99999,       0,       0,       0,
                                             0,        0,       0,   99999,       0,       0,
                                             0,        0,       0,       0,   99999,       0,
                                             0,        0,       0,       0,       0,   99999]
                self.odom_pub.publish(odom_msg)
            rate.sleep()

    def update_utm_cb(self, current_latlong):
        #rospy.loginfo("Current Lat, Long is: (%f,%f)" %((current_latlong.latitude),(current_latlong.longitude)))        
        self.current_utm = LLtoUTM(23, current_latlong.latitude,current_latlong.longitude)

if __name__ == '__main__':
        od_gps = Odom_gps()
        od_gps.mainloop()


'''
user1@ros20:~/catkin_ws/src/iggy/iggy_navigation$ rosmsg show nav_msgs/Odometry 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
'''
