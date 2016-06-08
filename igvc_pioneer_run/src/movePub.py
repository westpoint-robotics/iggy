#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

def talker():
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('movePub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():        
        #rospy.loginfo(inputCmd)        
        #rate.sleep()
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        inputCmd = PoseStamped()
        inputCmd.header.frame_id = "map"
        rospy.sleep(1.)
        inputCmd.header.stamp = rospy.get_rostime()
        print (rospy.get_rostime())
        inputCmd.pose.position.x = 4.50876379013
        inputCmd.pose.position.y = 3.73679423332
        inputCmd.pose.orientation.z = 0.0214622427325
        inputCmd.pose.orientation.w = 0.99976965954
        pub.publish(inputCmd)
        rate.sleep()
       
    # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    try:
            
      talker()
       
    except rospy.ROSInterruptException:
        pass
