#!/usr/bin/python
import time
import serial

ser = serial.Serial(
    port='/dev/raw_gps',
    #baudrate=115200, #8N1
    baudrate=9600, #8N1
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

if (ser.isOpen()):
    ser.close()
ser.open()

# Send commands to CNS-5000 to start the logs
ser.write('UNLOGALL\r\n')
time.sleep(0.05)
ser.write('RESET\r\n')
time.sleep(0.05)
ser.write('FRESET\r\n')
time.sleep(0.05)
ser.write('FRESET BASE_WEEK\r\n')
time.sleep(0.05)
ser.write('UNLOGALL\r\n')
time.sleep(0.05)

print("Just ran the RESET, FRESET, and FRESET BASE_WEEK commands. Power cycle the unit to complete the reset.")

