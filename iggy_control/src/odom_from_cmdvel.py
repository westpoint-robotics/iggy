#!/usr/bin/env python

""" Code written by: Dominic Larkin 04APR2017
    
    Subscribe to cmd_vel and assume the robot executes these commands and generate odometry from
    the cmd_vel.    
    
"""
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 
import math
import tf

class VirtualOdom():

    def __init__(self):     
        rospy.init_node('virtual_odom', anonymous=True)        
        self.br = tf.TransformBroadcaster()
        self.odom_pub = rospy.Publisher("cmd_odom", Odometry, queue_size=1 ) 
        rospy.Subscriber('/cmd_vel', Twist, self.update_cmd_vel)
        self.last_time = rospy.Time.now() 
        self.x = 0
        self.y = 0
        self.th = 0

    
    def update_cmd_vel(self, cmd_vel):
        pub_tf=rospy.get_param('~pub_tf',False)
        vx = cmd_vel.linear.x
        vy = cmd_vel.linear.y
        vth = cmd_vel.angular.z
        odom_msg = Odometry()
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        odom_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.th)
        odom_msg.pose.pose.orientation.x = odom_quat[0]
        odom_msg.pose.pose.orientation.y = odom_quat[1]
        odom_msg.pose.pose.orientation.z = odom_quat[2]
        odom_msg.pose.pose.orientation.w = odom_quat[3]
        odom_msg.pose.covariance = [   5.0, 0.0,       0.0,        0.0,        0.0,        0.0, 
                                       0.0, 99999.0,   0.0,        0.0,        0.0,        0.0, 
                                       0.0, 0.0,       99999.0,    0.0,        0.0,        0.0,   
                                       0.0, 0.0,       0.0,        99999.0,    0.0,        0.0, 
                                       0.0, 0.0,       0.0,        0.0,        99999.0,    0.0, 
                                       0.0, 0.0,       0.0,        0.0,        0.0,        99999.0] 
        
        # Build the odometry message to publish
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.child_frame_id = "base_link"
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = vth
        odom_msg.twist.covariance = [  5.0, 0.0,       0.0,        0.0,        0.0,        0.0, 
                                       0.0, 99999.0,   0.0,        0.0,        0.0,        0.0, 
                                       0.0, 0.0,       99999.0,    0.0,        0.0,        0.0,   
                                       0.0, 0.0,       0.0,        99999.0,    0.0,        0.0, 
                                       0.0, 0.0,       0.0,        0.0,        99999.0,    0.0, 
                                       0.0, 0.0,       0.0,        0.0,        0.0,        99999.0] 
        if((vx+vth)<0.10): #if stopped or near stopped believe this more.
            odom_msg.twist.covariance[0]=0.0001
            odom_msg.twist.covariance[7]=0.0001
        print "PUBTF",pub_tf
        if (pub_tf):
           self.br.sendTransform((self.x, self.y, 0),
                     tf.transformations.quaternion_from_euler(0.0, 0.0, self.th),
                     rospy.Time.now(),
                     odom_msg.child_frame_id,
                     odom_msg.header.frame_id)
        #Publish the odometry message
        self.last_time = current_time
        self.odom_pub.publish(odom_msg)
          

if __name__ == '__main__':
    global nav
    try:
        v_odom = VirtualOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Command Velocity to Odometry finished.")


'''
rosmsg show nav_msgs/Odometry 
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
