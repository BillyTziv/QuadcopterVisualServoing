import time
import serial

ser=serial.Serial('/dev/ttyACM0', 9600)

while True:
	myData = ser.readline()
	print myData
