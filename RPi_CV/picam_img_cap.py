# Capture an image from pi camera

import cv2
import numpy as np
import sys
import serial
import time
import serial.tools.list_ports
from picamera.array import PiRGBArray
from picamera import PiCamera 

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)

s = [0, 0, 0, 0, 0, 0, 0, 0]

# Image points threshold
pb_threshold = 20 	# Pixel Brightness thrshold is the max value of pixels we accept
c_dist = 20		# Distance between coordinates to accept into the s vector

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# Capture a new frame from the webcam
	cframe_rgb = frame.array

	# Convert the captured frame from RGB to GRAY scale
	cframe_gray = cv2.cvtColor(cframe_rgb, cv2.COLOR_BGR2GRAY)
	
	# Create a list with all the black 2D coordinates found (a point must have a brighness
	# lower than 10 to be black)
	blackList = np.argwhere(cframe_gray < pb_threshold)
	counter = 1

	# Run through all of the coordinates and keep one record for each unique point found.
	for blackPoint in blackList:
		# If s vector is null insert the black point (initialize the s vector)
		if not s:
			#print "First element :: Inserting: ", blackPoint
			s.append(blackPoint)
		else:
			# We assume that the every blackPoint is new. If there is already a similar
			# point in the s vector, EXISTS variable equals to 1 and the blackPoint will
			# not be inserted to the s list. Else the blackPoint is categorized as unique.
			EXISTS=0
			for sPoint in s:
				if( ( abs(blackPoint[0]-sPoint[0]) < c_dist ) and ( abs(blackPoint[1]-sPoint[1]) < c_dist )):
					EXISTS=1
			if EXISTS==0:	
				#print "New element match :: Inserting: ", blackPoint
				s.append(blackPoint)

	print s

        if len(s) == 4:
		# Prepare the string for the arduino (must start with '<' and end with '>')
		s_raw = s[0][0], s[0][1], s[1][0], s[1][1], s[2][0], s[2][1], s[3][0], s[3][1]

		# Calculate the error between s and s _des
		#error = s - s_des
		
		# Find the velociteis of the desired camera
		#v_cam_des = L1 * 

		strToSend = "<"+str(s_raw)+ ">"
		print strToSend	
	break
	rawCapture.truncate(0)
	cv2.imshow('frame', cframe_gray)
# Destroy all windows and exit
cv2.destroyAllWindows()

