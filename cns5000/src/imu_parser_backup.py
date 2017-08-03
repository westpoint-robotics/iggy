#!/usr/bin/env python
# Software License Agreement (BSD License)

# KVH CG-5100 IMU ROS Driver
# Copyright (c) 2013, Cheng-Lung Lee, University of Detroit Mercy.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# KVH CG-5100 IMU ROS Driver
# Copyright (c) 2013, Cheng-Lung Lee, University of Detroit Mercy.

# Changelog

# 
# 2013.03.19 Initial release.
#
#
#
# KVH CG-5100 IMU data format
# RS232/RS422 115200 BPS 8N1, all binary data , 36 Bytes/Package , Rate: 100 Hz 
# 
# Here are the example data set (in C format)... 36 byte each ( 4 Header + 30 Data + 2 CRC )
# H0   H1   H2   H3   1    2    3    4    5    6    7    8    9    10   11   12   13   14   15
# 0xfe 0x81 0xff 0x55 0x35 0xc3 0x74 0xeb 0x35 0x8b 0xe4 0xc9 0x35 0xca 0xb2 0x39 0xb8 0x62 0xd2
# 16   17   18   19   20   21   22   23   24   25   26   27   28   29   30   C0   C1
# 0xf0 0x3a 0x30 0xc4 0x52 0x3d 0xc9 0x33 0xb2 0x00 0x00 0x00 0x00 0x77 0x22 0x0d 0x8e
#
# Decode
# H0~3 Header   : Always 0xFE81FF55; this value will never occur anywhere else
# 1,2,3,4       : X angle,SPFP(Single-precision floating point) +/-0.66 radians
# 5,6,7,8       : Y angle, SPFP( Single-precision floating point) +/-0.66 radians
# 9,10,11,12    : Z angle, SPFP( Single-precision floating point) +/-0.66 radians
# 13,14,15,16   : X velocity,  SPFP +/-1 m/sec Assumes 100 Hz TOV
# 17,18,19,20   : Y velocity,  SPFP +/-1 m/sec Assumes 100 Hz TOV
# 21,22,23,24   : Z velocity,  SPFP +/-1 m/sec Assumes 100 Hz TOV
# 25,26,27,28   : Odometer pulses,  SPFP, 45 kHz
# 29            : Status, DISC 1=valid;0=invalid , Value=119 == All sensor OK.
# 30            : Sequence ,UINT8 0-127 Increments for each message and resets to 0 after 127
# C0,C1         : Computed as follows: Each byte in the message data is treated as an
#                 unsigned, 8-bit integer; the checksum is the least significant two
#                 bytes of the accumulated data

# Decode example 1 ( c style )
# X: 0x35 0xc3 0x74 0xeb = 0x35c374eb
#       struct.unpack('!f', '35c374eb'.decode('hex'))[0]
#       1.4562659771399922e-06
# Y: 0x35 0x8b 0xe4 0xc9 =0x358be4c9
#       struct.unpack('!f', '358be4c9'.decode('hex'))[0]
#       1.0422892273709294e-06
# Z: 0x35 0xca 0xb2 0x39 =0x35cab239
#       struct.unpack('!f', '35cab239'.decode('hex'))[0]
#       1.510204242549662e-06
# Vx: 0xb8 0x62 0xd2 0xf0=0xb862d2f0
#       struct.unpack('!f', 'b862d2f0'.decode('hex'))[0]
#       -5.4079049732536077e-05
# Vy: 0x3a 0x30 0xc4 0x52=0x3a30c452
#       struct.unpack('!f', '3a30c452'.decode('hex'))[0]
#       0.00067431211937218904
# Vz: 0x3d 0xc9 0x33 0xb2=0x3dc933b2
#       struct.unpack('!f', '3dc933b2'.decode('hex'))[0]
#       0.09824313223361969
# encoder: 0x00 0x00 0x00 0x00
# Status: 0x77
# Sequence: 0x22
# Checksum: 0x0d 0x8e 

# Decode example2 ( python style )
# reference: http://docs.python.org/2/library/struct.html#struct.unpack
#   >>> data="\xfe\x81\xff\x55\x35\x80\x66\xd8\xb5\xe1\xc8\xbe\x35\x91\xdd\x76\x38\xe4\x92\x24\x39\xfc\x05\xd7\x3d\xc8\xfb\x66\x00\x00\x00\x00\x77\x4c\x0e\x34"
#   len(data)
#   36
#   >>> import struct  
# decode all data
#   >>> all=struct.unpack(">fffffffBBH",data[4:36])
# all=(9.5666746346978471e-07, -1.6822230008983752e-06, 1.0867795481317444e-06, 0.00010899108019657433, 0.00048069536569528282, 0.098135754466056824, 0.0, 119, 76, 3636)
# Status =119 = Binary %1110111 = all sensor status Ok
# cehck CRC , read all data (30 byte) as unsigned char (B) , and CRC as unsigned short int (H)
#   >>> Acrc=struct.unpack(">BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBH",data[4:36])
#    if this is true CRC ok 
#       sum(Acrc[0:30])==Acrc[30]

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

import serial, string, math, time, calendar
import struct
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
import numpy


cov = 0.0001#1e-6

def wrapTo2PI(theta):
    '''Normalize an angle in radians to [0, 2*pi]
    '''
    return theta % (2.*math.pi)

def wrapToPI(theta):
    '''Normalize an angle in radians to [-pi, pi]
    '''
    return (wrapTo2PI(theta+math.pi) - math.pi)

def KVHCG5100shutdownhook():
    global KVH_IMU
    print "KVH CG-5100 IMU shutdown time!"
    rospy.loginfo('Closing IMU Serial port')
    KVH_IMU.close() #Close KVH_IMU serial port
    
    
def CehckCRC(data):
    # in this function cehck CRC
    # data="\xfe\x81\xff\x55\x35\x80\x66\xd8\xb5\xe1\xc8\xbe\x35\x91\xdd\x76\x38\xe4\x92\x24\x39\xfc\x05\xd7\x3d\xc8\xfb\x66\x00\x00\x00\x00\x77\x4c\x0e\x34"
    # cehck CRC , read all data (30 byte) as unsigned char (B) , and CRC as unsigned short int (H)
    if (len(data)==36)&(data[0:4]=="\xfe\x81\xff\x55"):
        Acrc=struct.unpack(">BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBH",data[4:36])
        return sum(Acrc[0:30])==Acrc[30]
    else:
        return False
        
if __name__ == '__main__':
    global KVH_IMU
    global SerialData
    rospy.init_node('kvhCG5100imu')
    #Pos_pub = rospy.Publisher('imu/raw/heading', Pose2D, queue_size=1)
    Imu_pub = rospy.Publisher('imu/raw', Imu, queue_size=1)
    #Twist_pub = rospy.Publisher('imu/speed', Twist)
    # TODO add publisher for IMU_Status here
    # TODO add publisher for IMU velocity here
    KVHCG5100Pose2D=Pose2D()
    KVHCG5100Pose2D.x=float(0.0)
    KVHCG5100Pose2D.y=float(0.0)
    #Init KVH_IMU port
    #KVH_IMUport = rospy.get_param('~port','/dev/ttyUSB0')
    KVH_IMUport = rospy.get_param('~port','/dev/raw_imu')
    KVH_IMUrate = rospy.get_param('~baud',115200)
    # printmodulus set to 1 is 100 Hz. 2 : 50Hz 
    #KVH_IMUprintmodulus = rospy.get_param('~printmodulus',1)
    #IMU heading offset in degree
    #KVH_IMU_offset = rospy.get_param('~offset',0.)

    CRC_error_limits   =rospy.get_param('~CRC_error_limits', 10.)
    CRC_errorcounter=0


    imu_data = Imu()
    imu_data = Imu(header=rospy.Header(frame_id="cns5000_frame"))
    
    #TODO find a right way to convert imu acceleration/angularvel./orientation accuracy to covariance
    imu_data.orientation_covariance = [cov, 0, 0, 
                                       0, cov, 0, 
                                       0, 0, cov]
    
    imu_data.angular_velocity_covariance = [cov, 0, 0,
                                            0, cov, 0, 
                                            0, 0, cov]
    
    imu_data.linear_acceleration_covariance = [cov, 0, 0, 
                                               0, cov, 0, 
                                               0, 0, cov]

    #twist_data = Twist()
    #twist_data = Twist(header=rospy.Header(frame_id="KVH_CG5100_IMU"))
    rospy.on_shutdown(KVHCG5100shutdownhook) 

    try:
        #talker()
        #ReadCompass()
        #Setup Compass serial port
        KVH_IMU = serial.Serial(port=KVH_IMUport, baudrate=KVH_IMUrate, timeout=.5)
        
        ### start of loop back test ###
        #testdata="\xfe\x81\xff\x55\x35\x80\x66\xd8\xb5\xe1\xc8\xbe\x35\x91\xdd\x76\x38\xe4\x92\x24\x39\xfc\x05\xd7\x3d\xc8\xfb\x66\x00\x00\x00\x00\x77\x4c\x0e\x34"
        #KVH_IMU.write(testdata)
        ### end of loop back test ###
        
        time.sleep(0.1)
        # readout all data, if any
        rospy.loginfo("Test reading data from IMU. Got bytes %i" % KVH_IMU.inWaiting() ) # should got some data
        if (KVH_IMU.inWaiting() >0):
                #read out all datas, the response shuldbe OK
                data=KVH_IMU.read(KVH_IMU.inWaiting())
                if ("\xfe\x81\xff\x55" in data) :
                    rospy.loginfo("Got Header from IMU") 
                else :
                    rospy.logerr("Got No Header from IMU data. Please check serial port!") 
                    rospy.logerr('[0]Received No IMU header from KVH CG-5100 IMU. Please check IMU serial port and IMU Power. Shutdown!')
                    rospy.signal_shutdown('Received No IMU header from KVH CG-5100 IMU')
        else:
                #send error, if no data in buffer
                rospy.logerr('[1]Received No data from KVH CG-5100 IMU. Please check IMU serial port and IMU Power. Shutdown!')
                rospy.signal_shutdown('Received No data from KVH CG-5100 IMU')

        dataSynced=False
        data=""
        X=0.
        Y=0.
        Z=0.
        Ex_old=0.0
        Ey_old=0.0
        Ez_old=0.0
        Vx_old=0.0
        Vy_old=0.0
        Vz_old=0.0
        Time_step=0.1
        while not rospy.is_shutdown():  
            ### start of loop back test ###
            #testdata="\xfe\x81\xff\x55\x35\x80\x66\xd8\xb5\xe1\xc8\xbe\x35\x91\xdd\x76\x38\xe4\x92\x24\x39\xfc\x05\xd7\x3d\xc8\xfb\x66\x00\x00\x00\x00\x77\x4c\x0e\x34"
            #KVH_IMU.write(testdata)
            ### end of loop back test ### 
                 
            if (dataSynced) :
                data = KVH_IMU.read(36)
                DataTimeSec = rospy.get_time()
            #    if (KVH_IMU.inWaiting()>=36) : # if we still have data in buffer .... we process data too slow show error and re-sync
            #        rospy.logerr(" Seems we have too much IMU data in buffer or too slow in processing data , must re-sync") # 
            #        rospy.loginfo("Data in buffer %i" % KVH_IMU.inWaiting() ) # should got some data
            #        dataSynced=False
            else : # if not synced , look for header
                data += KVH_IMU.read(KVH_IMU.inWaiting())
                while (len(data)>=36):
                    if ("\xfe\x81\xff\x55" == data[0:4]):
                        if ( len(data)%36 >0 ): # still has unread data , so read them until we have full package
                            data += KVH_IMU.read(36-len(data)%36)
                            # we have 36*N data in buffer
                            # only keep the lastest data
                        data=data[len(data)-36:]
                        if (len(data)==36):
                            dataSynced=True
                            DataTimeSec = rospy.get_time() 
                            break       
                    else :
                        data=data[1:] # drop 1 byte if not header
                        
            try:
                if (dataSynced) :
                            if CehckCRC(data):

                                fields=struct.unpack(">fffffffBB",data[4:34])
                                # 1,2,3,4       : X angle, SPFP( Single-precision floating point) +/-0.66 radians
                                # 5,6,7,8       : Y angle, SPFP( Single-precision floating point) +/-0.66 radians
                                # 9,10,11,12    : Z angle, SPFP( Single-precision floating point) +/-0.66 radians
                                # 13,14,15,16   : X velocity,  SPFP +/-1 m/sec Assumes 100 Hz TOV
                                # 17,18,19,20   : Y velocity,  SPFP +/-1 m/sec Assumes 100 Hz TOV
                                # 21,22,23,24   : Z velocity,  SPFP +/-1 m/sec Assumes 100 Hz TOV
                                # 25,26,27,28   : Odometer pulses,  SPFP, 45 kHz
                                # 29            : Status, DISC 1=valid;0=invalid , Value=119 == All sensor OK.
                                # 30            : Sequence ,UINT8 0-127 Increments for each message and resets to 0 after 127
                                
                                # Note KVH CG5100 IMU use ENU system , same as ROS
                                Ex =fields[0] # X angle +/-0.66 radians
                                Ey =fields[1] # Y angle +/-0.66 radians
                                Ez =fields[2] # Z angle +/-0.66 radians
                                Vy=fields[3] # X velocity, +/-1 m/sec
                                Vx=fields[4] # Y velocity, +/-1 m/sec
                                Vz=fields[5] # Z velocity, +/-1 m/sec
                                Odometer=fields[6]  # Odometer pulses
                                Status  =fields[7]    # Status, Value=119 == All sensor OK.
                                Sequence=fields[8] # Sequence UINT8 0-127 0 after 127
                                
                                #imu_data.header.stamp = rospy.Time.now() # Should add an offset here
                                imu_data.header.stamp = rospy.Time.from_sec(DataTimeSec-len(data)/11520.) # this is timestamp with a bit time offset 10bit per byte @115200bps
                                # IMU outputs [x,y,z] ENU , convert Euler angle to Quaternion 
                                #imu_data.orientation = Quaternion()
                                X+=Ex
                                Y+=Ey
                                Z+=Ez
                                
                                #print(Z,Y,X,Ex,Ey,Ez,Vx,Vy,Vz,imu_data.orientation.x,imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w, imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z, imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z)
 

                                #http://answers.ros.org/question/53688/euler-angle-convention-in-tf/
                                # ???--> #imu_quaternion=quaternion_from_euler(X,Y,Z) # Euler's roll, pitch and yaw angles
                                #q = tf.transformations.quaternion_from_euler(yaw, pitch, roll, 'rzyx')
                                imu_quaternion=tf.transformations.quaternion_from_euler(Z, Y, X, 'rzyx')
                                #imu_quaternion=quaternion_from_euler(Ex,Ey,Ez) # Euler's roll, pitch and yaw angles

                                # DML Next two lines additions to normalize the quaternion
                                quat = numpy.array([imu_quaternion[0],imu_quaternion[1],imu_quaternion[2],imu_quaternion[3]])                    
                                imu_quaternion[0],imu_quaternion[1],imu_quaternion[2],imu_quaternion[3] = quat / numpy.sqrt(numpy.dot(quat, quat))

                                imu_data.orientation.x=imu_quaternion[0]
                                imu_data.orientation.y=imu_quaternion[1]
                                imu_data.orientation.z=imu_quaternion[2]
                                imu_data.orientation.w=imu_quaternion[3]
                                # calculate angular/rate 
                                
                                # ( new - old ) / time_step ( 0.01 sec )
                                #imu_data.angular_velocity.x = (Ex-Ex_old) / Time_step
                                #imu_data.angular_velocity.y = (Ey-Ey_old) / Time_step
                                #imu_data.angular_velocity.z = (Ez-Ez_old) / Time_step
                                imu_data.angular_velocity.x = Ex / Time_step
                                imu_data.angular_velocity.y = Ey / Time_step
                                imu_data.angular_velocity.z = Ez / Time_step

                                # calculate acceleration 
                                # newV -oldV / time_step ( 0.01sec )
                                #imu_data.linear_acceleration.x = (Vx-Vx_old) / Time_step
                                #imu_data.linear_acceleration.y = (Vy-Vy_old) / Time_step
                                #imu_data.linear_acceleration.z = (Vz-Vz_old) / Time_step
                                imu_data.linear_acceleration.x = Vx / Time_step
                                imu_data.linear_acceleration.y = Vy / Time_step
                                imu_data.linear_acceleration.z = Vz / Time_step

                                Imu_pub.publish(imu_data)
                                
                                # TODO publish IMU Velocity data here
                                
                                #twist_data.linear.x=Vx
                                #twist_data.linear.y=Vy
                                #twist_data.linear.z=Vz
                                #twist_data.angular.x=imu_data.angular_velocity.x
                                #twist_data.angular.y=imu_data.angular_velocity.y
                                #twist_data.angular.z=imu_data.angular_velocity.z
                                #Twist_pub.publish(twist_data)
                                # save the current data for next loop
                                Ex_old=Ex
                                Ey_old=Ey
                                Ez_old=Ez
                                Vx_old=Vx
                                Vy_old=Vy
                                Vz_old=Vz 
                                                               
                                KVHCG5100Pose2D.y=Status # put update rate here for debug the update rate
                                KVHCG5100Pose2D.x=Sequence # put mSec tick here for debug the speed
                                KVHCG5100Pose2D.theta = wrapToPI(Z)

                                #Pos_pub.publish(KVHCG5100Pose2D)
                                
                                # reset counter once you have good data
                                CRC_errorcounter=0

                            else:
                                rospy.logerr("[3] CRC error. Sentence was: %s" % ':'.join(x.encode('hex') for x in data))
                                rospy.logerr("[3] CRC error, must re-sync") # 
                                dataSynced=False
                                CRC_errorcounter=CRC_errorcounter+1
                                if (CRC_errorcounter>CRC_error_limits):
                                        CRC_errorcounter=0
                                        rospy.logfatal('Too Much Back-to-Back CRC error ,Closing KVH IMU Serial port')
                                        KVH_IMU.close() #Close KVH_IMU serial port
                                        KVH_IMU = serial.Serial(port=KVH_IMUport, baudrate=KVH_IMUrate, timeout=.5)
                                        time.sleep(0.01)
                                        rospy.loginfo('Try to re-open IMU Serial port')
                                        KVH_IMU.open() #Close KVH_IMU serial port


                                
                #else:
                        #rospy.logerr("[4]Received a sentence but not correct. Sentence was: %s" % ':'.join(x.encode('hex') for x in data))

            except ValueError as e:
                rospy.logwarn("Value error, likely due to missing fields in the data messages.Sentence was: %s" % ':'.join(x.encode('hex') for x in data) )

            # no loop, delay, ROSspin() here, we try to read all the data asap

        rospy.loginfo('Closing IMU Serial port')
        KVH_IMU.close() #Close KVH_IMU serial port
    except rospy.ROSInterruptException:
        pass
