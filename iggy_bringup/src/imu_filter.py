#!/usr/bin/env python

""" 
imu_complementary_filter
source: http://www.phidgets.com/docs/Compass_Primer
"""
import math
import rospy
import tf
import sys
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Twist, Vector3Stamped
from nav_msgs.msg import Odometry

class MagHeading():
    def __init__(self):
        self.mMagnetic=Vector3Stamped()
        self.mImu=Imu()
        self.compassBearingFilter = []
        self.compassBearingFilterSize = 1 #10 TODO return to a deeper filter
        self.lastAngles= []

        # Give the node a name
        rospy.init_node('imu_filtered', anonymous=False)

        # Publisher of type nav_msgs/Odometry
        self.magHdg_pub = rospy.Publisher('imu/compass', Float32, queue_size=1)
        self.world_imu_pub = rospy.Publisher ('imu/global', Imu, queue_size=1)
        #self.br = tf.TransformBroadcaster()

        # Subscribe to the gps positions
        rospy.Subscriber('magnetic', Vector3Stamped, self.update_magnetic_callback)
        rospy.Subscriber('imu/raw', Imu, self.update_imu_callback)

    def update_magnetic_callback(self, msg):
        self.mMagnetic = msg

    def update_imu_callback(self, msg):
        self.mImu = msg
          
    def calculateCompassBearing(self,sd,md):
        gravity=[sd.linear_acceleration.x,sd.linear_acceleration.y,sd.linear_acceleration.z]
        magField=[md.vector.x,md.vector.y,md.vector.z]
        compassBearing = pitchAngleDeg=rollAngleDeg=yawAngle=pitchAngle=rollAngle=0
        
        try:     
            lastAngles=self.lastAngles
            compassBearingFilter=self.compassBearingFilter
        
            #Roll Angle - about axis 0
            #  tan(roll angle) = gy/gz
            #  Use atan2 so we have an output os (-180 - 180) degrees
            rollAngle = math.atan2(gravity[1], gravity[2])
         
            #Pitch Angle - about axis 1
            #  tan(pitch angle) = -gx / ((gy * sin(roll angle)) + (gz * cos(roll angle)))
            #  Pitch angle range is (-90 - 90) degrees
            pitchAngle = math.atan(
                -gravity[0] / ((gravity[1] * math.sin(rollAngle)) + (gravity[2] * math.cos(rollAngle)))
            )
         
            #Yaw Angle - about axis 2
            #  tan(yaw angle) = (mz * sin(roll) - my * cos(roll)) / 
            #                   (mx * cos(pitch) + my * sin(pitch) * sin(roll) + mz * sin(pitch) * cos(roll))
            #  Use atan2 to get our range in (-180 - 180)
            #
            #  Yaw angle == 0 degrees when axis 0 is pointing at magnetic north
            yawAngle = math.atan2(
                   (magField[2] * math.sin(rollAngle))
                 - (magField[1] * math.cos(rollAngle))
                ,
                   (magField[0] * math.cos(pitchAngle))
                 + (magField[1] * math.sin(pitchAngle) * math.sin(rollAngle))
                 + (magField[2] * math.sin(pitchAngle) * math.cos(rollAngle)))
            angles = [rollAngle, pitchAngle, yawAngle]

            #we low-pass filter the angle data so that it looks nicer on-screen
        
            #make sure the filter buffer doesn't have values passing the -180<->180 mark
            #Only for Roll and Yaw - Pitch will never have a sudden switch like that
            if len(lastAngles) == 3:
                for i in [0,2]:
                    if (abs(angles[i] - lastAngles[i]) > 3):
                        for stuff in compassBearingFilter:
                            if (angles[i] > lastAngles[i]):
                                stuff[i] += 360 * math.pi / 180.0
                            else:
                                stuff[i] -= 360 * math.pi / 180.0

            self.lastAngles = angles           

            compassBearingFilter.append(angles)
            if (len(compassBearingFilter) > self.compassBearingFilterSize):
                compassBearingFilter.pop(0)

            yawAngle = pitchAngle = rollimuAngle = 0
            for stuff in compassBearingFilter:
                rollAngle += stuff[0]
                pitchAngle += stuff[1]
                yawAngle += stuff[2]
            yawAngle /= len(compassBearingFilter)
            pitchAngle /= len(compassBearingFilter)
            rollAngle /= len(compassBearingFilter)

            #Convert radians to degrees for display
            compassBearing = yawAngle * (180.0 / math.pi)
            pitchAngleDeg = (pitchAngle * (180.0 / math.pi))
            rollAngleDeg = (rollAngle * (180.0 / math.pi))    
        except ZeroDivisionError:
            e = sys.exc_info()[0]
            print("Exception computing bearing: %s " % e)
        return [compassBearing,pitchAngleDeg,rollAngleDeg,yawAngle,pitchAngle,rollAngle]
    
    def mainLoop(self):
        worldImu= Imu()
        rate = rospy.Rate(20) # 10hz
        pub_tf=rospy.get_param('~pub_tf',"false")
        imu_frame=rospy.get_param('~imu_frame','imu')
        world_frame=rospy.get_param('world_frame','odom')
        while not rospy.is_shutdown():
            if(self.mImu.header.seq!=0):
                angles =self.calculateCompassBearing(self.mImu,self.mMagnetic)
                #print("PHIDG: Compass Heading> compassBearing: %9.6f  pitchAngleDeg: %9.6f  rollAngleDeg: %9.6f" % (angles[0],angles[1],angles[2]))   
                self.magHdg_pub.publish(angles[0])
                worldImu= self.mImu
                worldImu.orientation.x,worldImu.orientation.y,worldImu.orientation.z,worldImu.orientation.w = \
                                            tf.transformations.quaternion_from_euler(angles[5], angles[4], angles[3])
                #if (pub_tf):
                    #self.br.sendTransform((0,0,0),(worldImu.orientation.x, worldImu.orientation.y, worldImu.orientation.z,
                                            #worldImu.orientation.w), rospy.Time.now(), imu_frame, world_frame)
                self.world_imu_pub.publish(worldImu)
            rate.sleep()    

if __name__ == '__main__':
        mHead=MagHeading()
        mHead.mainLoop()

'''
rosmsg show sensor_msgs/Imu
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Quaternion orientation
  float64 x
  float64 y
  float64 z
  float64 w
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
  float64 x
  float64 y
  float64 z
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
  float64 x
  float64 y
  float64 z
float64[9] linear_acceleration_covariance

'''
        

