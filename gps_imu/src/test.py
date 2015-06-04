#!/usr/bin/python
import time
import serial
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from usma_novatel_parser import *



align = input ("thing:")
print align
