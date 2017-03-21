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
        # Give the node a name
        rospy.init_node('magnetic_heading', anonymous=False)

        # Publisher of type nav_msgs/Odometry
        self.magHdg_pub = rospy.Publisher('/imu/mag', Float32, queue_size=1)
        
        # Subscribe to the gps positions
        rospy.Subscriber('/phidget/imu/mag', Vector3Stamped, self.update_magnetic_callback)
        rospy.Subscriber('/phidget/imu/data_raw', Imu, self.update_imu_callback)
        rospy.Subscriber('/magnetic', Vector3Stamped, self.update_xsensM_callback)
        rospy.Subscriber('/imu/raw', Imu, self.update_xsensG_callback)
        self.mMagnetic=Vector3Stamped()
        self.mXsensM=Vector3Stamped()
        self.mImu=Imu()
        self.mXsensG=Imu()
        self.compassBearingFilterX = []
        self.compassBearingFilterP = []
        self.compassBearingFilterSize = 1 #10
        self.lastAnglesP= []
        self.lastAnglesX= []

    def update_magnetic_callback(self, msg):
        self.mMagnetic = msg

    def update_imu_callback(self, msg):
        self.mImu = msg
        
    def update_xsensM_callback(self, msg):
        self.mXsensM = msg
        
    def update_xsensG_callback(self, msg):
        self.mXsensG = msg
                
    def update_heading(self):
        heading = Float32()
        compassBearing = 0.0
        gravity0=self.mImu.linear_acceleration.x
        gravity1=self.mImu.linear_acceleration.y
        gravity2=self.mImu.linear_acceleration.z
        magField0 = self.mMagnetic.vector.x
        magField1 = self.mMagnetic.vector.y
        magField2 = self.mMagnetic.vector.z
        if ((gravity0 + gravity1 + gravity2) != 0.0 or (magField0 + magField1 + magField2) != 0.0): 
            try:
                # Compute roll(about axis 0) angle = atan2( gy/gz) 
                rollAngle = math.atan2(gravity1, gravity2)

                # Compute pitch(about axis 1) angle = -gx / ((gy * sin(roll angle)) + (gz * cos(roll angle)))
                pitchAngle = math.atan(-1*(gravity0)/((gravity1*math.sin(rollAngle)) + (gravity2*math.cos(rollAngle))))

                # Compute yaw(about axis 2) angle = = (mz * sin(roll) - my * cos(roll))/(mx * cos(pitch) + my * sin(pitch) * sin(roll) + mz * sin(pitch) * cos(roll))         
                yawAngle = math.atan2((magField2*math.sin(rollAngle)) - (magField1*math.cos(rollAngle)),
                                      (magField0*math.cos(pitchAngle))+(magField1*math.sin(pitchAngle)*math.sin(rollAngle))+
                                      (magField2*math.sin(pitchAngle)*math.cos(rollAngle)))
                yawBearing = magField0 * (180/math.pi)   
                pitchBearing = magField1 * (180/math.pi)   
                rollBearing = magField2 * (180/math.pi)   
                print "ROLLANGLE:",rollBearing,"PITCHANGLE:",pitchBearing,"yawAngle:",yawBearing                                                  
            except:
                print("exception caught")
                raise
        return compassBearing
        
    def calculateCompassBearing(self,sd,md,isXsens):
        gravity=[sd.linear_acceleration.x,sd.linear_acceleration.y,sd.linear_acceleration.z]
        magField=[md.vector.x,md.vector.y,md.vector.z]
        compassBearing = pitchAngleDeg=rollAngleDeg=yawAngle=pitchAngle=rollAngle=0
        
        try:     
            if isXsens:
                lastAngles=self.lastAnglesX
                compassBearingFilter=self.compassBearingFilterX
            else:
                lastAngles=self.lastAnglesP
                compassBearingFilter=self.compassBearingFilterP
        
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

            if isXsens:
                self.lastAnglesX = angles
            else:
                self.lastAnglesP = angles           

            compassBearingFilter.append(angles)
            if (len(compassBearingFilter) > self.compassBearingFilterSize):
                compassBearingFilter.pop(0)

            yawAngle = pitchAngle = rollAngle = 0
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
        rate = rospy.Rate(5) # 10hz
        while not rospy.is_shutdown():
            #print self.mImu
            if(self.mImu.header.seq!=0 and self.mXsensG.header.seq!=0):
                #self.magHdg_pub.publish(self.update_heading())
                angles =self.calculateCompassBearing(self.mImu,self.mMagnetic,False)
                print("PHIDG: Compass Heading> compassBearing: %9.6f  pitchAngleDeg: %9.6f  rollAngleDeg: %9.6f" % (angles[0],angles[1],angles[2]))   
                angles =self.calculateCompassBearing(self.mXsensG,self.mXsensM,True)
                print("XSENS: Compass Heading> compassBearing: %9.6f  pitchAngleDeg: %9.6f  rollAngleDeg: %9.6f" % (angles[0],angles[1],angles[2]))     
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
        

