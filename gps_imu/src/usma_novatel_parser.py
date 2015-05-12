#!/usr/bin/python
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

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
    fix_msg.latitude = float(latitude)
    fix_msg.longitude = float(longitude)
    fix_msg.altitude = float(heightMSL)
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
  string frame_idvelodyne
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
    #------IMU data formatting--------
    velx = float(deltaAccelX)*.05/pow(2,15)
    vely = float(deltaAccelY)*.05/pow(2,15)
    velz = float(deltaAccelZ)*.05/pow(2,15)
    imu_msg = Vector3(velx,vely,velz)
    return imu_msg
   

def parse_novatelINSPVA(insString):
    #print "++++instring:",insString
    gnssWeek = insString[0] # GNSS week
    secondsFromWeek = insString[1] # Seconds from week start
    latitude = insString[2] # Latitude (WGS84)
    longitude = insString[3] # Longitude (WGS84)
    heightMSL = insString[4] # Ellipsoidal Height (WGS84) [m]
    velNorth = insString[5] # Velocity in northerly direction [m/s] (negative for south)
    velEast = insString[6] # Velocity in easterly direction [m/s] (negative for west)
    velUp = insString[7] # Velocity in upward direction [m/s]
    roll = insString[8] # Roll - Right-handed rotation from local level around Y-axis in degrees
    pitch = insString[9] # Pitch - Right-handed rotation from local level around X-axis in degrees
    azimuth = insString[10] # Azimuth - Left-handed rotation around Z-axis in degrees clockwise from North
    inertialStatus = insString[11].split('*')[0] # Inertial status
    #print "inertialStatus",inertialStatus
    fix_msg = NavSatFix()
    fix_msg.latitude = float(latitude)
    fix_msg.longitude = float(longitude)
    fix_msg.altitude = float(heightMSL)

    #print "lat, long, alt:" + str(fix_msg.latitude)+ " , "+ str(fix_msg.longitude)+" , " + str(fix_msg.altitude)
    imu_msg = Twist()
    imu_msg.linear.x = float(velEast)*.05/pow(2,15)
    imu_msg.linear.y = float(velNorth)*.05/pow(2,15)
    imu_msg.linear.z = float(velUp)*.05/pow(2,15)
    imu_msg.angular.x = float(pitch)
    imu_msg.angular.y = float(roll)
    #velx = float(velEast)*.05/pow(2,15)
    #vely = float(velNorth)*.05/pow(2,15)
    #velz = float(velUp)*.05/pow(2,15)
    #imu_msg = Vector3(velx,vely,velz,pitch,roll)
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

    
