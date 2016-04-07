#!/usr/bin/env python 

#SHAMELESSLY STOLEN FROM:
#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.leftRed = rospy.Publisher("/camera/left/redFilter",Image,queue_size=1)
    self.leftBlue = rospy.Publisher("/camera/left/blueFilter",Image,queue_size=1)
    self.rightRed = rospy.Publisher("/camera/right/redFilter",Image,queue_size=1)
    self.rightBlue = rospy.Publisher("/camera/right/blueFilter",Image,queue_size=1)

    self.bridge = CvBridge()
    self.leftSub = rospy.Subscriber("/camera/left/rgb",Image,self.callbackLeft)
    self.rightSub = rospy.Subscriber("/camera/right/rgb",Image,self.callbackRight)

  def callbackLeft(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    #print cv_image

    red = np.array([0,0,50,50,50,255])
    blue = np.array([86,31,4,220,88,50])
    
    maskRed = cv2.inRange(cv_image, red[0:3], red[3:6])
    maskBlue = cv2.inRange(cv_image, blue[0:3], blue[3:6])

    self.leftRed.publish(self.bridge.cv2_to_imgmsg(maskRed, "mono8"))
    self.leftBlue.publish(self.bridge.cv2_to_imgmsg(maskBlue, "mono8"))

  def callbackRight(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    red = np.array([0,0,50,50,50,255])
    blue = np.array([86,31,4,220,88,50])

    maskRed = cv2.inRange(cv_image, red[0:3], red[3:6])
    maskBlue = cv2.inRange(cv_image, blue[0:3], blue[3:6])

    self.rightRed.publish(self.bridge.cv2_to_imgmsg(maskRed, "mono8"))
    self.rightBlue.publish(self.bridge.cv2_to_imgmsg(maskBlue, "mono8"))

def main(args):
  ic = image_converter()
  rospy.init_node('Hansing_converter', anonymous=True)
  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    r.sleep()

if __name__ == '__main__':
    main(sys.argv)
