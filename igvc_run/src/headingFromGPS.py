#!/usr/bin/env python

""" 


"""
import math
import rospy
import tf
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class GpsHeading():
    def __init__(self):
        # Give the node a name
        rospy.init_node('gps_heading', anonymous=False)

        # Publisher of type nav_msgs/Odometry
        self.hdg_pub = rospy.Publisher('/novatel/heading', PoseWithCovarianceStamped, queue_size=1)
        self.points = []
	

        # Subscribe to the gps positions
        rospy.Subscriber('/gps/fix', NavSatFix, self.update_gps_callback)
        # Subscribe to the gps positions
        rospy.Subscriber('/odom/virtual', Odometry, self.update_odom_callback)
        self.lastHdg=Float32()
	self.lastHdg=0.0
        self.coord=NavSatFix()
        self.prevCoord = NavSatFix()
        self.odom = Odometry()

    def update_odom_callback(self, odom):
        self.odom = odom


    def update_gps_callback(self, current_latlong):
            if (len(self.points) < 4) :
            	self.points = [current_latlong] + self.points
		return
            else:
		self.points = [current_latlong] + self.points[0:3]
            self.prevCoord = self.points[0] #store prev gps data into separate data structure
            prevLat = self.prevCoord.latitude #extract previous lat
            prevLon = self.prevCoord.longitude #extract previous lon        
            self.coord=self.points[3] #update current gps data
            lat=self.coord.latitude #extract current lat
            lon=self.coord.longitude #extract current lon
            hdg=Float32()
            hdg=self.findBearing(prevLat,prevLon,lat,lon)
            msg = PoseWithCovarianceStamped()
            q = tf.transformations.quaternion_from_euler(0,0,math.radians(hdg))
            print hdg
            msg.pose.pose.orientation = Quaternion(*q)
            covariance = [0.00]*36
            covariance[35] = 0.001
            msg.pose.covariance = covariance
	    msg.header.frame_id="base_link"
            msg.header.stamp=rospy.Time.now()
            if ((180 - abs(abs(self.lastHdg-hdg)-180))<15):
                self.hdg_pub.publish(msg)
                self.lastHdg=hdg
        
    def findBearing(self,lat1,lon1,lat2,lon2): 
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)
        diffLong = math.radians(lon2 - lon1)
        x = math.sin(diffLong) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)* math.cos(lat2) * math.cos(diffLong))
        initial_bearing = math.atan2(x, -y)
        # Now we have the initial bearing but math.atan2 return values
        # from -180 to + 180 which is not what we want for a compass bearing
        # The solution is to normalize the initial bearing as shown below
        initial_bearing = 90 + math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360
        return compass_bearing #as float
        
if __name__ == '__main__':
        gHead=GpsHeading()
        rospy.spin()

        

