# This python sketch gets an input from the keyboard
# and writes it to serial waiting from the arduino to
# pass it rhrough the flight controller.

import time
import serial
import cv2

# Create a new serial with the arduino.
ser=serial.Serial('/dev/ttyACM0', 9600)

# Wait till arduino signals that it is ready
data = ser.readline()
print data

# Send a signal to arm the quadrotor
print "Sending signal to arm the quadrotor..."
ser.write("<ARM>")
print "Waiting for arduino to respond..."
data = ser.readline()
print data

while True:
	try:
		if(ser.isOpen()):
			#incData = ser.readline()
			#print incData
			for i in range(1000, 1600, 100):
				data = "<"+str(i)+",1464,1464,1464>"
				ser.write(data)
				print "Waiting for response"
				incData = ser.readline()
				print incData
		else:
			print "Arduino is not connected!"
	except KeyboardInterrupt:
		# Send a signal to disarm the quadrotor
		print "Sending signal to disarm the quadrotor..."
		ser.write("<DARM>")
		print "Waiting for the arduino to respond..."
		data = ser.readline()
		print data

		# Break the loop and terminate the program
		break

print "Program terminated successfully!"
