#!/usr/bin/env python

"""Copyright 2010 Phidgets Inc.
This work is licensed under the Creative Commons Attribution 2.5 Canada License. 
To view a copy of this license, visit http:#creativecommons.org/licenses/by/2.5/ca/
"""

__author__ = 'Adam Stelmack'
__version__ = '2.1.8'
__date__ = 'May 17 2010'

#Basic imports
from ctypes import *
import sys
import math
import time
#Phidget specific imports
from Phidgets.Phidget import Phidget
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import SpatialDataEventArgs, AttachEventArgs, DetachEventArgs, ErrorEventArgs
from Phidgets.Devices.Spatial import Spatial, SpatialEventData, TimeSpan
from Phidgets.Phidget import PhidgetLogLevel

compassBearingFilter = []
compassBearingFilterSize = 10
lastAngles = []
#Create an accelerometer object
try:
    spatial = Spatial()
except RuntimeError as e:
    print("Runtime Exception: %s" % e.details)
    print("Exiting....")
    exit(1)

#Information Display Function
def DisplayDeviceInfo():
    print("|------------|----------------------------------|--------------|------------|")
    print("|- Attached -|-              Type              -|- Serial No. -|-  Version -|")
    print("|------------|----------------------------------|--------------|------------|")
    print("|- %8s -|- %30s -|- %10d -|- %8d -|" % (spatial.isAttached(), spatial.getDeviceName(), spatial.getSerialNum(), spatial.getDeviceVersion()))
    print("|------------|----------------------------------|--------------|------------|")
    print("Number of Acceleration Axes: %i" % (spatial.getAccelerationAxisCount()))
    print("Number of Gyro Axes: %i" % (spatial.getGyroAxisCount()))
    print("Number of Compass Axes: %i" % (spatial.getCompassAxisCount()))

#Event Handler Callback Functions
def SpatialAttached(e):
    attached = e.device
    print("Spatial %i Attached!" % (attached.getSerialNum()))

def SpatialDetached(e):
    detached = e.device
    print("Spatial %i Detached!" % (detached.getSerialNum()))

def SpatialError(e):
    try:
        source = e.device
        print("Spatial %i: Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        
def calculateCompassBearing(sd):
    global compassBearingFilter
    global compassBearingFilterSize
    global lastAngles
    gravity=sd.Acceleration
    magField=sd.MagneticField
    compassBearing = pitchAngleDeg = rollAngleDeg =0
    
 
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
    try:
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
        lastAngles = angles

        compassBearingFilter.append(angles)
        if (len(compassBearingFilter) > compassBearingFilterSize):
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
    except:
        e = sys.exc_info()[0]
        print("Exception computing bearing: %s " % e)
    return [compassBearing,pitchAngleDeg,rollAngleDeg,yawAngle,pitchAngle,rollAngle]

     
def SpatialData(e):
    source = e.device
    #print("Spatial %i: Amount of data %i" % (source.getSerialNum(), len(e.spatialData)))
    for index, spatialData in enumerate(e.spatialData):
        '''
        print("=== Data Set: %i ===" % (index))
        if len(spatialData.Acceleration) > 0:
            print("Acceleration> x: %6f  y: %6f  z: %6f" % (spatialData.Acceleration[0], spatialData.Acceleration[1], spatialData.Acceleration[2]))
        if len(spatialData.AngularRate) > 0:
            print("Angular Rate> x: %6f  y: %6f  z: %6f" % (spatialData.AngularRate[0], spatialData.AngularRate[1], spatialData.AngularRate[2]))
        if len(spatialData.MagneticField) > 0:
            print("Magnetic Field> x: %6f  y: %6f  z: %6f" % (spatialData.MagneticField[0], spatialData.MagneticField[1], spatialData.MagneticField[2]))
        '''
        angles =calculateCompassBearing(spatialData)
        print("Compass Heading> compassBearing: %6f  pitchAngleDeg: %6f  rollAngleDeg: %6f" % (angles[0],angles[1],angles[2]))            
        #print("Time Span> Seconds Elapsed: %i  microseconds since last packet: %i" % (spatialData.Timestamp.seconds, spatialData.Timestamp.microSeconds))
        time.sleep(0.1)
    
    #print("------------------------------------------")

#Main Program Code
try:
    #logging example, uncomment to generate a log file
    #spatial.enableLogging(PhidgetLogLevel.PHIDGET_LOG_VERBOSE, "phidgetlog.log")

    spatial.setOnAttachHandler(SpatialAttached)
    spatial.setOnDetachHandler(SpatialDetached)
    spatial.setOnErrorhandler(SpatialError)
    spatial.setOnSpatialDataHandler(SpatialData)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

print("Opening phidget object....")

try:
    spatial.openPhidget()
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

print("Waiting for attach....")

try:
    spatial.waitForAttach(10000)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    try:
        spatial.closePhidget()
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)
    print("Exiting....")
    exit(1)
else:
    spatial.setDataRate(4)
    DisplayDeviceInfo()

print("Press Enter to quit....")

chr = sys.stdin.read(1)

print("Closing...")

try:
    spatial.closePhidget()
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

print("Done.")
exit(0)
