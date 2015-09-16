#!/usr/bin/env python
# license removed for brevity
### joystick for Aria
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def talker():
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
    pub1 = rospy.Publisher('navigation_velocity_smoother/raw_cmd_vel', Twist, queue_size=10)
    #rospy.init_node('joyPub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #inputCmd.axes: [-0.0, -0.0, 0.0, -0.0, -0.0, 0.0, 0.0, 0.0]
        #=inputCmd.axis[0] 
        #=inputCmd.axis[1] 
        rospy.loginfo(inputCmd)
        
        rate.sleep()



def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
    pub1 = rospy.Publisher('navigation_velocity_smoother/raw_cmd_vel', Twist, queue_size=10)
    inputCmd = Twist()
    inputCmd.linear.x = data.axes[1]
    inputCmd.angular.z = data.axes[0]
    pub.publish(inputCmd)
    pub1.publish(inputCmd)
    #return data
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('joylistener', anonymous=True)

    rospy.Subscriber("joy", Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
      listener()        
      talker()
       
    except rospy.ROSInterruptException:
        pass
