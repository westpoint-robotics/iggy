#!/usr/bin/env python  

import rospy
import tf
from sensor_msgs.msg import Imu

def callbackIMU(data):
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 2), (data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w), rospy.Time.now(), "cns5000_frame", "map")

    
    '''
    br2 = tf.TransformBroadcaster()
    r,p,y = tf.transformations.euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
    br2.sendTransform((0, 0, 1), tf.transformations.quaternion_from_euler(y,p,r), rospy.Time.now(), "imu_frame2", "map")
    '''

if __name__ == '__main__':
    rospy.init_node('imu_tf_broadcaster')
    rospy.Subscriber('/imu_data', Imu, callbackIMU)
    rospy.spin()
