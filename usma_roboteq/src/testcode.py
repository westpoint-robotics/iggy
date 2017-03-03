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


'''
import serial
import time
import os

# global variables
lastSwitchVal = 0
switchValue = 0
RCmode = 2
estopCount = False

# Set Command Priorities
cmd = '^CPRI 1 1\r' # Pulse In as first priority
time.sleep(0.05)
cmd = '^CPRI 2 0\r' # Serial is second priority
time.sleep(0.05)
cmd = '^OVL 350\r' # Set over voltage to 35.0 volts
time.sleep(0.05)
cmd = '^UVL 180\r' # Set under voltage to 18.0 volts
time.sleep(0.05)
cmd = '^MAC 1 20000\r' # Set acceleration rate for motor 1
time.sleep(0.05) 
cmd = '^MAC 2 20000\r' # Set acceleration rate for motor 2
time.sleep(0.05)
cmd = '^MXRPM 1 3500\r' # Set maximum rpm for motor 1
time.sleep(0.05)
cmd = '^MXRPM 2 3500\r' # Set maximum rpm for motor 2
time.sleep(0.05)
cmd = '^MXMD 1\r' # Set mixed to Mode 1
time.sleep(0.05)

# Check if Pulse In is enabled for PIn 1 to 4
cmd = '~PMOD\r' 
pmod = '1:0:1:1:0' #ser.write(cmd)
pmod = pmod.split(':')
on =[0,1,2,4]
for i in on:
    if pmod[i] == '0':
        cmd = '^PMOD ' + str(i) + '\r'
        pmod[i] = '1' 
if pmod[3] == '1':
    cmd = '^PMOD 3\r'
    pmod[3] = '0'
print pmod


