#!/usr/bin/env python

import csv
import roslib
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

def make_waypoint_viz(pos,txt,ident):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "my_namespace"
    marker.id = ident
    #marker.type = Marker.SPHERE
    marker.type = Marker.TEXT_VIEW_FACING
    marker.text = "HERENOW!!"
    #marker.lifetime = 30.0
    marker.action = Marker.ADD
    marker.pose = pos
    #marker.pose.position.x = 1
    #marker.pose.position.y = 1
    #marker.pose.position.z = 1
    #marker.pose.orientation.x = 0.0
    #marker.pose.orientation.y = 0.0
    #marker.pose.orientation.z = 0.0
    #marker.pose.orientation.w = 1.0
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1.0 # Don't forget to set the alpha!
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    #only if using a MESH_RESOURCE marker type:
    #marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae"
    return marker

#Test Code below
if __name__ == '__main__':
    rospy.init_node('waypoint_visualization', anonymous=True)
    vis_pub = rospy.Publisher( "visualization_marker", Marker, queue_size=1 )
    rate = rospy.Rate(1)
    p = Pose()
    x=0
    while not rospy.is_shutdown():
        t = "HERENOW!"
        p.position.x = 1+x
        p.position.y = 1
        p.position.z = 1
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0    
        vis_pub.publish( make_waypoint_viz(p,t,x ))
        x +=1
        rate.sleep()


