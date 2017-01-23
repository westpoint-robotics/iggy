#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('roboteq_driver/cmd', Twist, queue_size=1)
    rospy.init_node('send_roboteq_cmd', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        cmd = Twist()
        cmd.linear.x = 1.0
        cmd.angular.z = 0.009 #0.009   #through testing found this to be the oofset in the wheels


        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
