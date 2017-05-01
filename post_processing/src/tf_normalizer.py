#!/usr/bin/python
import rosbag
import tf

with rosbag.Bag('2017-04-21-15-54-05.bag_NEW', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('2017-04-21-15-54-05.bag', 'w'):
        if topic != '/tf':
            outbag.write(topic, newMsg, t)



'''

rosmsg show tf2_msgs/TFMessage 
geometry_msgs/TransformStamped[] transforms
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string child_frame_id
  geometry_msgs/Transform transform
    geometry_msgs/Vector3 translation
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion rotation
      float64 x
      float64 y
      float64 z
      float64 w

'''
