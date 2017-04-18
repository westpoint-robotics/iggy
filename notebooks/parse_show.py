#!/usr/bin/python
#Primary Jupyter python script for analysis of BAG file data from IGVC runs. Made by 2LT John Oberholtzer
#Updated 03JUN2016

#Checklist for operation:
#1. Change "Suffix" to appropriate ending from runs.
#2. Change the booleans on the FLAG list below to correspond with which BAG files are available. Adjust as necessary.
#3. Comment or otherwise disable any tests you do not wish to run in their corresponding cells.
#4. Click Cells>Run All to execute

#This file will require updating if rostopics are moved from one bag file to another as the references are hardcoded.
import matplotlib
import csv
import sys
import ast
import os
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from datetime import datetime
from struct import *
from mpl_toolkits.mplot3d import Axes3D
from LatLongUTMconversion import LLtoUTM
csv.field_size_limit(sys.maxsize)
matplotlib.use('Agg')
#%matplotlib nbagg
pylab.rcParams['figure.figsize'] = 15, 8  # that's default image size for this interactive session

#The folder the BAG files are in should be named as RUN + the suffix. Example: RUN_2016-05-26-10-34-51
#bagfolder = "RUN" + suffix + "/"
datafolder = "Data/circle05/"

#Load INSPVAA messages from the KVH CSV files.
#If aware that .CSV files are not present, you can optionally manually uncheck the flags from the above cell,
#although the code will automatically check and set the flags to False where .CSV files are not present.
inspvafile = "kvh_INSPVAA.csv"
FLAG_inspvafile = True
InspvaArray = []
if (os.path.isfile(datafolder+inspvafile) == False):
    FLAG_inspvafile = False
    print("File not found: " + datafolder+inspvafile)
else:    
    with open((datafolder+inspvafile),'rb') as infile:
        content = infile.readlines()
        for row in content:
            #if ('\0' not in row) and (('BESTGNSSPOSA'in row) or ('INSPVAA'in row)): # remove unparsed bytes found when in SBAS mode.
            if (('BESTGNSSPOSA'in row) or ('INSPVAA'in row)): # remove unparsed bytes found when in SBAS mode.
                if ('\0' not in row):
                    InspvaArray.append(row.split(','))
                else:
                    row=row.split('#')[-1].split(',') 
                    row[0]='#'+row[0]
                    InspvaArray.append(row)             
    print("Found file: " + datafolder+inspvafile + " with " + str(len(InspvaArray))+ " rows of data.")
    print InspvaArray[4]

#INSPVAA Lat and Long PROCESSING (RUN IF USING ANALYSIS)
inspva_latitudes= []
inspva_longitudes= []
inspva_altitudes= [] 
gnss_latitudes= []
gnss_longitudes= []
gnss_altitudes= [] 
if FLAG_inspvafile == True:
    TFcolumnnames = InspvaArray.pop(0)
    TFdataex = InspvaArray[0]    
   
    for row in InspvaArray:
        if ('#INSPVAA'==row[0]):
            if ('FINESTEERING'==row[4]) and ('INS_SOLUTION_GOOD'==row[20]):
                inspva_latitudes.append(row[11])
                inspva_longitudes.append(row[12])
                inspva_altitudes.append(row[13])  
        if ('#BESTGNSSPOSA'==row[0]):
            if ('FINESTEERING'==row[4]):
                gnss_latitudes.append(row[11])
                gnss_longitudes.append(row[12])
                gnss_altitudes.append(row[13])                  
    print row[0],row[1]

    print len(InspvaArray),len(inspva_latitudes),TFcolumnnames,'/n',TFdataex

#Load FLEX6 messages from the FLEX6 GPS CSV files.

flex6_gpsfile = "flex6_gps.csv"
FLAG_flex6_gps = True
flex6_gpsArray = []
if (os.path.isfile(datafolder+flex6_gpsfile) == False):
    FLAG_inspvafile = False
    print("File not found: " + datafolder+flex6_gpsfile)
else:    
    with open((datafolder+flex6_gpsfile),'rb') as infile:
        content = infile.readlines()
        for row in content:
            #if ('\0' not in row) and (('BESTGNSSPOSA'in row) or ('INSPVAA'in row)): # remove unparsed bytes found when in SBAS mode.
            if (('BESTPOSA'in row)): # remove unparsed bytes found when in SBAS mode.
                    #row=row.split('#')[-1].split(',') 
                    #row[0]='#'+row[0]
                    flex6_gpsArray.append(row.split(','))             
    print("Found file: " + datafolder+flex6_gpsfile + " with " + str(len(flex6_gpsArray))+ " rows of data.")
    print flex6_gpsArray[4]


#INSPVAA Lat and Long PROCESSING (RUN IF USING ANALYSIS)
flex6_latitudes= []
flex6_longitudes= []
flex6_altitudes= [] 
if FLAG_flex6_gps == True:
    TFcolumnnames = flex6_gpsArray.pop(0)
    TFdataex = flex6_gpsArray[0]    
   
    for row in flex6_gpsArray:
                flex6_latitudes.append(row[11])
                flex6_longitudes.append(row[12])
                flex6_altitudes.append(row[13])                 
    print row[0],row[1]

    print len(flex6_gpsArray),len(flex6_latitudes),flex6_latitudes[9],flex6_longitudes[9],'/n',TFcolumnnames

#Load mti-7000 messages from the Xsens CSV files.
mti700file = "mti700.csv"
FLAG_mti700file = True
mti700Array = []
newDict={}
mti700_latitudes= []
mti700_longitudes= []
mti700_altitudes= [] 
if (os.path.isfile(datafolder+mti700file) == False):
    FLAG_mti700file = False
    print("File not found: " + datafolder+mti700file)
else:    
    with open((datafolder+mti700file),'rb') as dictfile:        
        for line in dictfile:
            if 'Lat' in line:
                mti700Array.append(eval(line))
                mti700_latitudes.append((eval(line))['GNSS']['Lat'])
                mti700_longitudes.append((eval(line))['GNSS']['Lon'])
    print("Found file: " + datafolder+mti700file + " with " + str(len(mti700_latitudes))+ " rows of data.")
    print mti700Array[9]
    print mti700_latitudes[9]
    print mti700_longitudes[9]

#GPS AND ODOMGPS GRAPHING FOR LOCATION
plt.ion()
if FLAG_inspvafile == True:
#if False:
    #plt.plot(inspva_longitudes,inspva_latitudes, 'ro')
    #plt.plot(mti700_longitudes,mti700_latitudes, 'b^')
    #plt.plot(gnss_longitudes,gnss_latitudes, 'g^')
    #plt.plot(flex6_longitudes,flex6_latitudes, 'b*')
    #plt.plot(-73.97545914722222,41.38231066666667,'b*') #Survey Point on Stony2
    plt.show(block=False)
    for i in range(len(flex6_longitudes)):
        plt.plot(float(flex6_longitudes[i]),float(flex6_latitudes[i]),'r*')
        plt.show(block=False)
        plt.draw()
        print i
        time.sleep(0.1)
    plt.show()
    time.sleep(5)































