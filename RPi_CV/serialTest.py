# This python sketch gets an input from the keyboard
# and writes it to serial waiting from the arduino to
# pass it rhrough the flight controller.

import time
import serial

# Create a new serial with the arduino.
ser=serial.Serial('/dev/ttyACM0', 9600)

while True:
	if(ser.isOpen()):
		ser.write('s')
		time.sleep(1)
		incData = ser.readline()
		print incData
		#time.sleep(0.5)
	else:
		print "Arduino is not connected!"
