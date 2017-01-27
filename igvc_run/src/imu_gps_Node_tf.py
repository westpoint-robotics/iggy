#!/usr/bin/env python  

import rospy
import tf
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import std_msgs.msg
from LatLongUTMconversion import LLtoUTM

class igvc_localizer:


    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.location_pub = rospy.Publisher("/dml/odom_combined", Odometry,  queue_size = 1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/imu_data", Imu, self.callback_imu,  queue_size = 1)    
        self.subscriber = rospy.Subscriber("/gps/fix", NavSatFix, self.callback_gps,  queue_size = 1)   
        self.imuData = Imu() 
        self.gpsData = NavSatFix()
        self.isIMU = False
        self.isGPS = False

    def callback_imu(self, data):
        '''Callback function of subscribed topic. 
        Retrieve current imu data and update the current pose'''
        self.imuData = data
        self.isIMU = True


    def callback_gps(self, data):
        '''Callback function of subscribed topic. 
        Retrieve current gps dat and update current location relative to start location'''
        self.gpsData = data
        self.isGPS = True

    def get_absolute_odom(self):
        ''' Uses the current sensor readings to generate odometry
            Converts latitude and longitude to UTM assuming WGS84(23)
        '''
        odomData = Odometry()
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        odomData.header = h
        odomData.header.frame_id = "odom"
        odomData.child_frame_id = ""
        utm_zone, easting, northing = LLtoUTM(23, self.gpsData.latitude, self.gpsData.longitude) 
        #rospy.loginfo("ABS e: %f n: %f" % (easting, northing))
    
        odomData.pose.pose.position.x = easting
        odomData.pose.pose.position.y = northing
        odomData.pose.pose.position.z = 0
        odomData.pose.pose.orientation.x = self.imuData.orientation.x
        odomData.pose.pose.orientation.y = self.imuData.orientation.y
        odomData.pose.pose.orientation.z = self.imuData.orientation.z
        odomData.pose.pose.orientation.w = self.imuData.orientation.w
        odomData.pose.covariance = [0.01       ,0        ,0        ,0        ,0        ,0
                                     ,0        ,0.01     ,0        ,0        ,0        ,0
                                     ,0        ,0        ,0.01     ,0        ,0        ,0
                                     ,0        ,0        ,0        ,0.01     ,0        ,0
                                     ,0        ,0        ,0        ,0        ,0.01     ,0
                                     ,0        ,0        ,0        ,0        ,0        ,0.01]
        return odomData

    def get_relative_odom(self, initPos):
        ''' Uses the current sensor readings to generate odometry and then 
            makes the UTM coords relative to the start position
        '''
        odomData = self.get_absolute_odom() 
        #rospy.loginfo("omd e: %f n: %f" % (odomData.pose.pose.position.x, odomData.pose.pose.position.y))       
        odomData.pose.pose.position.x -= initPos.pose.pose.position.x
        odomData.pose.pose.position.y -= initPos.pose.pose.position.y
        return odomData
        
    #TODO move main code into functions

if __name__ == '__main__':
    localizer = igvc_localizer()
    rospy.init_node('igvc_localizer', anonymous=True)
    rate = rospy.Rate(15) # 10hz
    init_position = None
    while not rospy.is_shutdown():
        if (init_position is None) and localizer.isIMU and localizer.isGPS:
            init_position = localizer.get_absolute_odom()
            rospy.loginfo("Starting location utm east: %f north: %f" % (init_position.pose.pose.position.x, init_position.pose.pose.position.y))
        elif not(init_position is None):            
            current_odom = localizer.get_relative_odom(init_position)
            localizer.location_pub.publish(current_odom)
            br = tf.TransformBroadcaster()
            br.sendTransform((current_odom.pose.pose.position.x, current_odom.pose.pose.position.y, 0), (current_odom.pose.pose.orientation.x,current_odom.pose.pose.orientation.y,current_odom.pose.pose.orientation.z,current_odom.pose.pose.orientation.w), rospy.Time.now(), "base_footprint", "odom")
        rate.sleep()
    

'''
user1@user1:~/catkin_ws/rosbags/weMadeIt2$ rosmsg show nav_msgs/Odometry
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
