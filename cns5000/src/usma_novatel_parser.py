#!/usr/bin/python
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf
import numpy

import warnings
import math

curTime = rospy.Time()
curRoll = float(0.0)
curPitch = float(0.0)
curYaw = float(0.0)
lastYaw = float(0.0)
lastPitch = float(0.0)
lastRoll = float(0.0)

curvelcnsX = float(0.0)
curvelcnsY = float(0.0)
curvelcnsZ = float(0.0)
lastvelcnsX = float(0.0)
lastvelcnsY = float(0.0)
lastvelcnsZ = float(0.0)

def parse_novatelGPS(gpsString):    
    # ----- parse the data string from fields to variables -----
    solutionStatus = gpsString[0] # Solution status
    positionType = gpsString[1] # Position type
    latitude = gpsString[2] # Latitude
    longitude = gpsString[3] # Longitude
    heightMSL = gpsString[4] # Height above mean sea level
    undulation = gpsString[5] # Undulation
    datumID = gpsString[6] # Datum ID
    latitudeDev = gpsString[7] # Latitude standard deviation
    longitudeDev = gpsString[8] # Longitude standard deviation
    heightDev = gpsString[9] # Height standard deviation
    baseStationID = gpsString[10] # Base station ID
    differentialAge = gpsString[11] # Differential age
    solutionAge = gpsString[12] # Solution age in seconds
    observationsTracked = gpsString[13] # Number of observations tracked
    usedL1Ranges = gpsString[14] # Number of satellite solutions used in computation
    aboveMaskL1Ranges = gpsString[15] # Number of GPS and GLONASS L1 ranges above the RTK mask angle
    aboveMaskL2Ranges = gpsString[16] # Number of GPS and GLONASS L2 ranges above the RTK mask angle
    # ----- Format the gps data into ros msg -----
    fix_msg = NavSatFix()
    fix_msg.header.stamp = rospy.get_rostime()
    fix_msg.header.frame_id = 'cns5000_frame'
    fix_msg.latitude = float(latitude)
    fix_msg.longitude = float(longitude)
    fix_msg.altitude = float(heightMSL)
    fix_msg.position_covariance = [0.01,0.0,0.0,0.0,0.01,0.0,0.0,0.0,999.0]
    fix_msg.position_covariance_type = 1


    #fix_msg.position_covariance_type = 0
    #fix_msg.position_covariance = 0
    #fix_msg.status = 0 # Fix
    # TODO add a check for a valid position. Currently all data is marked as valid
    # TODO use the checksum to check the message
    #fix_msg.status = -1 # No Fix
    #print "Solution Status:", solutionStatus
    return fix_msg
'''
uint8 COVARIANCE_TYPE_UNKNOWN=0
uint8 COVARIANCE_TYPE_APPROXIMATED=1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
uint8 COVARIANCE_TYPE_KNOWN=3
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_idkvh5000
sensor_msgs/NavSatStatus status
  int8 STATUS_NO_FIX=-1
  int8 STATUS_FIX=0
  int8 STATUS_SBAS_FIX=1
  int8 STATUS_GBAS_FIX=2
  uint16 SERVICE_GPS=1
  uint16 SERVICE_GLONASS=2
  uint16 SERVICE_COMPASS=4
  uint16 SERVICE_GALILEO=8
  int8 status
  uint16 service
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type

'''

##TODO fix this imustring so that imu accel we dont care, but orientation is correct and coovarience matrix says to listen to it.
def parse_novatelIMU(imuString):
    gnssWeek = imuString[0] # GNSS week
    secondsFromWeek = imuString[1] # Seconds from week start
    imuStatus = imuString[2] # IMU status
    deltaAccelZ = imuString[3] # Change in accel. velocity count along the Z-axis
    deltaAccelY = imuString[4] # - Change in accel. velocity count along the Y-axis
    deltaAccelX = imuString[5] # Change in accel. velocity count along the X-axis
    deltaGyroZ = imuString[6] # Change in gyro angle count around the Z-axis, right-handed
    deltaGyroY = imuString[7] # - Change in gyro angle count around the Y-axis), right-handed
    deltaGyroX = imuString[8].split("*")[0] # Change in gyro angle count around the X-axis, right-handed
    degGyroZ = float(deltaGyroZ)*5.72957795 # *0.1 to radians and then * 180/pi for degrees (yaw)
    degGyroY = float(deltaGyroY)*5.72957795 # *0.1 to radians and then * 180/pi for degrees (pitch)
    degGyroX = float(deltaGyroX)*5.72957795 # *0.1 to radians and then * 180/pi for degrees (roll)
    #------IMU data formatting--------
    velx = float(deltaAccelX)*.05/pow(2,15)
    vely = float(deltaAccelY)*.05/pow(2,15)
    velz = float(deltaAccelZ)*.05/pow(2,15)
    global curTime
    global curRoll
    global curYaw
    global curPitch
    global lastRoll
    global lastYaw
    global lastPitch

    #cns to imu coordinate sytem conversion --> cnsX = -imuY, cnsY = imuX, cnsZ = imuZ
    #imuY comes in as negative of value --> cancels out negative conversion
    imuRoll = degGyroY
    imuPitch = degGyroX
    imuYaw = -1*degGyroZ   
    
    lastYaw = curYaw
    curYaw = imuYaw
    lastPitch = curPitch
    curPitch = imuPitch
    lastRoll = curRoll
    curRoll   = imuRoll

    lastTime = curTime    
    curTime = rospy.Time.now()
    deltime = (curTime-lastTime).to_sec()
    if (deltime == 0):
        deltime = 0.00000001

    imu_msg = Imu()
    imu_msg.header.stamp = curTime
    imu_msg.header.frame_id = 'cns5000_frame'
    #TODO linear acceleration is completely off but is being ignored, fix it for more accuracy
    imu_msg.linear_acceleration.x = float(deltaAccelY)/1000#*.05/pow(2,15)
    imu_msg.linear_acceleration.y = float(deltaAccelX)/1000#*-1#*.05/pow(2,15)
    imu_msg.linear_acceleration.z = float(deltaAccelZ)/10000#*.05/pow(2,15)
    imu_msg.linear_acceleration_covariance = [9999,0.0,0.0,0.0,9999,0.0,0.0,0.0,9999]
    #imu_msg.orientation_covariance.x = float(angcnsY)
    #euler = Vector3(curRoll, curPitch, curYaw)
    #euler = vector_norm(euler)
    quaternion = tf.transformations.quaternion_from_euler(curRoll, curPitch, curYaw)
    #type(pose) = geometry_msgs.msg.Pose

    # DML Next two lines additions to normalize the quaternion
    quat = numpy.array([quaternion[0],quaternion[1],quaternion[2],quaternion[3]])                    
    quaternion[0],quaternion[1],quaternion[2],quaternion[3] = quat / numpy.sqrt(numpy.dot(quat, quat))

    imu_msg.orientation.x = quaternion[0]
    imu_msg.orientation.y = quaternion[1]
    imu_msg.orientation.z = quaternion[2]
    imu_msg.orientation.w = quaternion[3]
    imu_msg.orientation_covariance = [0.1,0.0,0.0,0.0,0.1,0.0,0.0,0.0,0.1]
    imu_msg.angular_velocity.x = (curPitch-lastPitch)/deltime
    imu_msg.angular_velocity.y = (curRoll-lastRoll)/deltime
    imu_msg.angular_velocity.z = (curYaw-lastYaw)/deltime
    imu_msg.angular_velocity_covariance = [9999,0.0,0.0,0.0,9999,0.0,0.0,0.0,9999]
    
    return imu_msg
   
def parse_novatelINSPVA(insHeader, insString):
    #print "++++instring:",insString
    gnssWeek = insString[0] # GNSS week
    secondsFromWeek = insString[1] # Seconds from week start
    latitude = insString[2] # Latitude (WGS84)
    longitude = insString[3] # Longitude (WGS84)
    heightMSL = insString[4] # Ellipsoidal Height (WGS84) [m]
    velcnsY = float(insString[5]) # Velocity in northerly direction [m/s] (negative for south)
    velcnsX = float(insString[6]) # Velocity in easterly direction [m/s] (negative for west)
    velcnsZ = float(insString[7]) # Velocity in upward direction [m/s]
    cnsYaw = float(insString[8])*(3.14159265/180) # yaw/azimuth - Right-handed rotation from local level around Z-axis -- changed from degress to radians (pi/180 deg)
    cnsRoll = float(insString[9])*(3.14159265/180)  # roll - (neg) Right-handed rotation from local level around y-axis  -- changed from degress to radians (pi/180 deg)
    cnsPitch = float(insString[10])*(3.14159265/180)  # pitch -Right-handed rotation from local level around x-axis  -- changed from degress to radians (pi/180 deg)
    inertialStatus = insString[11].split('*')[0] # Inertial status
   
    #print "inertialStatus",inertialStatus
    fix_msg = NavSatFix()
    fix_msg.header.stamp = rospy.get_rostime()
    fix_msg.header.frame_id = 'cns5000_frame'
    fix_msg.latitude = float(latitude)
    fix_msg.longitude = float(longitude)
    fix_msg.altitude = float(heightMSL)

    fix_msg.position_covariance = [0.01,0.0,0.0,0.0,0.01,0.0,0.0,0.0,999.0]
    
    if (insHeader[4] == "FINESTEERING"):
        fix_msg.position_covariance_type = 2
    else:
        fix_msg.position_covariance_type = 0

    #print "lat, long, alt:" + str(fix_msg.latitude)+ " , "+ str(fix_msg.longitude)+" , " + str(fix_msg.altitude)
    global curTime

    global curRoll
    global curYaw
    global curPitch
    global lastRoll
    global lastYaw
    global lastPitch

    global curvelcnsX
    global curvelcnsY
    global curvelcnsZ
    global lastvelcnsX
    global lastvelcnsY
    global lastvelcnsZ

    #cns to imu coordinate sytem conversion --> cnsX = -imuY, cnsY = imuX, cnsZ = imuZ
    #imuY comes in as negative of value --> cancels out negative conversion
    imuYaw = -1*cnsPitch + 1.57079632679 #rotate 90 deg, or pi/2 radians
    #print(imuYaw)
    imuPitch = cnsRoll
    imuRoll = cnsYaw   
    
    lastYaw = curYaw
    curYaw = imuYaw
    lastPitch = curPitch
    curPitch = imuPitch
    lastRoll = curRoll
    curRoll   = imuRoll

    lastvelcnsX = curvelcnsX
    curvelcnsX = velcnsX
    lastvelcnsY = curvelcnsY
    curvelcnsY = velcnsY
    lastvelcnsZ = curvelcnsZ
    curvelcnsZ = velcnsZ


    lastTime = curTime    
    curTime = rospy.Time.now()
    deltime = (curTime-lastTime).to_sec()
    if (deltime == 0):
        deltime = 0.00000001

    imu_msg = Imu()
    imu_msg.header.stamp = curTime
    imu_msg.header.frame_id = 'cns5000_frame'
    imu_msg.linear_acceleration.x = (curvelcnsY-lastvelcnsY)/deltime #*.05/pow(2,15)
    imu_msg.linear_acceleration.y = (curvelcnsY-lastvelcnsY)/deltime #*-1#*.05/pow(2,15)
    imu_msg.linear_acceleration.z = (curvelcnsY-lastvelcnsY)/deltime #*.05/pow(2,15)
    imu_msg.linear_acceleration_covariance = [0.1,0.0,0.0,0.0,0.1,0.0,0.0,0.0,0.1]
    #imu_msg.orientation_covariance.x = float(angcnsY)
    #euler = Vector3(curRoll, curPitch, curYaw)
    #euler = vector_norm(euler)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, curYaw)
    #print(curRoll, curPitch, curYaw)
    #type(pose) = geometry_msgs.msg.Pose

    # DML Next two lines additions to normalize the quaternion
    quat = numpy.array([quaternion[0],quaternion[1],quaternion[2],quaternion[3]])                    
    quaternion[0],quaternion[1],quaternion[2],quaternion[3] = quat / numpy.sqrt(numpy.dot(quat, quat))

    imu_msg.orientation.x = quaternion[0]
    imu_msg.orientation.y = quaternion[1]
    imu_msg.orientation.z = quaternion[2]
    imu_msg.orientation.w = quaternion[3]
    imu_msg.orientation_covariance = [0.1,0.0,0.0,0.0,0.1,0.0,0.0,0.0,0.1]
    imu_msg.angular_velocity.x = (curPitch-lastPitch)/deltime
    imu_msg.angular_velocity.y = (curRoll-lastRoll)/deltime
    imu_msg.angular_velocity.z = (curYaw-lastYaw)/deltime
    imu_msg.angular_velocity_covariance = [0.1,0.0,0.0,0.0,0.1,0.0,0.0,0.0,0.1]
    #velx = float(velEast)*.05/pow(2,15)
    #vely = float(velNorth)*.05/pow(2,15)
    #velz = float(velUp)*.05/pow(2,15)
    #imu_msg = Vector3(velx,vely,velz,pitch,roll)
    #imu_msg.orientation.w = 0.01


    retrnList = [imu_msg, fix_msg]
    return retrnList
    #TODO put data into a ROS MSG and return it

# Below is the test code
if __name__ == "__main__":
    rawGPSstring = '#BESTGPSPOSA,COM1,0,85.5,FINESTEERING,1815,408756.000,00000000,8505,11526;\
    SOL_COMPUTED,SINGLE,41.39183389839,-73.95306269372,32.9551,-32.2000,WGS84,1.7972,1.3207,3.3414,\
    "",0.000,0.000,10,10,10,0,0,02,00,01*1904fda5'
    rawINSPVstring = '#INSPVAA,COM1,0,86.0,FINESTEERING,1815,408756.500,00000000,54e2,11526;1815,\
    408756.500000000,-90.00000000000,0.00000000000,-6356752.3142,0.0000,0.0000,0.0000,0.000000000,\
    0.000000000,0.000000000,INS_ALIGNING*6666667b'
    rawIMUString = '%RAWIMUSA,1815,407772.000;1815,407771.985103333,00000077,63811,-6290,6253,7,\
    -31,14*2d2a4cff'
    
    gpsPub = rospy.Publisher('fix', NavSatFix, queue_size=1)
    rospy.init_node('novatel_CNS5000', anonymous=True)
    rate = rospy.Rate(5) # 10hz
        
    def novatel_publisher(inString):  
        if (inString.split(",")[0] == "#BESTGPSPOSA"): # check if this the gps message
            gpsData = rawGPSstring.split(';')[1] # split the header and message
            gpsData = gpsData.split(',') # split on message fields
            out_msg = parse_novatelGPS(gpsData)
            gpsPub.publish(out_msg)

        elif (inString.split(",")[0] == "%RAWIMUSA"): # check if this the IMU message     
            imuData = rawIMUString.split(';')[1] # split the header and message
            imuData = imuData.split(',') # split on message fields
            parse_novatelIMU(imuData)
            #TODO make a publisher for rawimu

        elif (inString.split(",")[0] == "#INSPVAA"): # check if this the INSPVA message    
            insData = rawINSPVstring.split(';')[1] # split the header and message
            insData = insData.split(',') # split on message fields
            parse_novatelINSPVA(insData)
            #TODO make a publisher for INSPVA
    
    while not rospy.is_shutdown():  
        novatel_publisher(rawGPSstring)
        novatel_publisher(rawINSPVstring)
        novatel_publisher(rawIMUString)

    

