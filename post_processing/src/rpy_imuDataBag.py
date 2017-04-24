#!/usr/bin/python
import rosbag
import tf

with open('imu_rpy.csv', 'w') as oFile:
    outString = "secs.nsecs,orientation.x,orientation.y,orientation.z,orientation.w,"
    outString += "ang_vel.x,ang_vel.y,ang_vel.z,lin_accel.x,lin_accel.y,lin_accel.z,"
    outString += "roll,pitch,yaw\n"
    oFile.write(outString)
    with rosbag.Bag('imu_rpy.bag', 'w') as outbag:
        for topic, msg, t in rosbag.Bag('imudata.bag').read_messages():
            if topic == '/imu_data':
                yaw, pitch, roll = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
                newMsg = msg
                newMsg.orientation.x = yaw
                newMsg.orientation.y = yaw
                newMsg.orientation.z = yaw
                newMsg.orientation.w = 0
                outbag.write(topic, newMsg, t)

                outString = str(msg.header.stamp.secs)+"."+str(msg.header.stamp.nsecs)+","
                outString += str(msg.orientation.x)+","+str(msg.orientation.y)+","+str(msg.orientation.z)+","+str(msg.orientation.w)+","
                outString += str(msg.angular_velocity.x)+","+str(msg.angular_velocity.y)+","+str(msg.angular_velocity.z)+","
                outString += str(msg.linear_acceleration.x)+","+str(msg.linear_acceleration.y)+","+str(msg.linear_acceleration.z)+","
                outString += str(roll)+","+str(pitch)+","+str(yaw)+"\n"
                oFile.write(outString)
