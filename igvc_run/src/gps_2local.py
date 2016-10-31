#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

class republish:

    def __init__(self):
        rospy.init_node("gps2local")
        rospy.Subscriber("/gps_rawFix", NavSatFix, self.coordCb)
        self.scanPub = rospy.Publisher('/gps/fix', NavSatFix)
        rospy.spin()

    def coordCb(self, msg):

        msg.latitude = msg.latitude - 32.0153551135
        msg.longitude = msg.longitude - 47.2741644959
        msg.altitude = msg.altitude - 247.196
        self.scanPub.publish(msg)

if __name__ == "__main__":
    a = republish()


## 320153.551135	4727416.44959	247.196

