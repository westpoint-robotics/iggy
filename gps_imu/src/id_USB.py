#!/usr/bin/env python  

# AFter applying this patch: https://acassis.wordpress.com/2015/04/14/nameerror-global-name-base-is-not-defined/

import serial.tools.list_ports
import time




if __name__ == '__main__':
    #print serial.tools.list_ports.comports()
    #print [port for port in serial.tools.list_ports.comports().ListPortInfo() if port[2] != 'n/a']

    #for port in serial.tools.list_ports.comports():
    #    if port[2] != 'n/a':
    #        print port.ListPortInfo() 

    ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=115200, #8N1
        #baudrate=9600, #8N1
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )

    if (ser.isOpen()):
        ser.close()
    ser.open()
    # Send commands to CNS-5000 to start the logs
    ser.write('unlogall\r\n')
    ser.write('LOG COM1 INSPVAA ONTIME 0.2\r\n')
    #ser.write('LOG COM1 RAWIMUSA ONTIME 0.2\r\n')
    #ser.write('LOG COM1 BESTGPSPOSA ONTIME 0.2\r\n')
    time.sleep(1)
    while ser.inWaiting() > 0:
                # While data is in the buffer
                velodyne_output = ser.readline()
                print velodyne_output,'\n'
