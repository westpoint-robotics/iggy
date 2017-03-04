#!/usr/bin/env python
# the "correct" way to do this is using http://wiki.ros.org/actionlib
# this is a short term solution








import rospy

#additions from Royal
import roslib
import actionlib

#end of additions


from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
#from sensor_msgs.msg import Imu



def gps_callback(gps):
    return gps.pose

#def callback(imu):
#    return imu


if __name__ == '__main__':
    try:
        rospy.init_node('igvc_gps_goals', anonymous=True)
        pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        rate = rospy.Rate(15) # 10hz
        rospy.Subscriber("gps/fix", NavSatFix, gps_callback)
        #rospy.Subscriber("imu_data", Imu, callback)
        seqnum = 0
    while not rospy.is_shutdown():        
        gpsGoal = PoseStamped()
        gpsGoal.header.seq = seqnum
        gpsGoal.header.stamp = rospy.get_rostime()
        gpsGoal.header.frame_id = 'map'
        gpsGoal.pose.position
        
        pub.publish(gpsGoal)
        seqnum += 1


        rate.sleep()
    except rospy.ROSInterruptException:
        pass



