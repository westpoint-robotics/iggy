#!/usr/bin/python

'''
The controller uses a simple communication protocol based on ASCII characters. Com-
mands are not case sensitive. ?a is the same as ?A. Commands are terminated by car-
riage return (Hex 0x0d, 'eschar r').
The underscore '_' character is interpreted by the controller as a carriage return. This alter-
nate character is provided so that multiple commands can be easily concatenated inside a
single string.
All other characters lower than 0x20 (space) have no effect.
The controller will echo back to the PC or Microcontroller every valid character it has re-
ceived. If no echo is received, one of the following is occurring
The controller will acknowledge commands in one of the two ways:
For commands that cause a reply, such as a configuration read or a speed or amps que-
ries, the reply to the query must be considered as the command acknowledgment.
For commands where no reply is expected, such as speed setting, the controller will
issue a "plus" character (+) followed by a Carriage Return after every command as an ac-
knowledgment.
If a command or query has been received, but is not recognized or accepted for any rea-
son, the controller will issue a "minus" character (-) to indicate the error.

The general format for setting a parameter is the "^" character followed by the command
name followed by parameter(s) for that command. These will set the parameter in the con-
troller's RAM and this parameter becomes immediately active for use. The parameter can
also be permanently saved in EEPROM by sending the %EESAV maintenance command.

~ Read configuration operation 
^ Write configuration operation 
! Run time commands
? Run time queries
% Maintenance Commans

'''
import serial
import time
import os

# global variables
lastSwitchVal = 0
switchValue = 0
RCmode = 2
estopCount = False

def getdata():
    info = ''
    while ser.inWaiting() > 0: # While data is in the buffer
        info += str(ser.read())
    return info

# configure the serial connections 
try:
    ser = serial.Serial(
        port='/dev/roboteq',
        baudrate=115200, #8N1
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
except:
    #raise
    try:
        ser = serial.Serial(
            port='/dev/ttyACM0',
            baudrate=115200, #8N1
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
    except:
        raise

if (ser.isOpen()):
    ser.close()
ser.open()

# Below are the initial configurations for the Roboteq motor controller required by Iggy. See RoboteqIggySettings.pdf.
configCmds=['^CPRI 1 1\r',      '^CPRI 2 0\r',      '^OVL 350\r',       '^UVL 180\r',
            '^MAC 1 20000\r',   '^MAC 2 20000\r',   '^MXRPM 1 3500\r',  '^MXRPM 2 3500\r',
            '^MXMD 1\r',        '^PMOD 0 1\r']

# Send commands to Roboteq and exit if any fail
for cmd in configCmds:
    ser.write(cmd)
    time.sleep(.01)
    result = getdata()
    if (result != '+\r'):
        print "ERROR: ROBOTEQ DRIVER FAILED TO SET CONFIGURATION WITH: ", cmd,"\n"
        print "ERROR: ROBOTEQ DRIVER CONFIGURATION FAILED. NOW EXITING ROBOTEQ DRIVER\n\n"
        exit()
    print "SUCCESSFULLY CONFIGURED THE ROBOTEQ MOTOR CONTROLLER\n"
    


