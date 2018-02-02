# Calibration code for the desired s vector
#
# It takes an image which becomes the desired image

import cv2
import numpy as np
import sys
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

print "\nCircle tracking software stated...\n"

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)


# Hough Parameters
dp=2			# The inverse ration of resolution
min_dist=50 	# Minimum distance between the detecetd cirlces
par1=200
par2=100

# Set the video device (-1 seems to work for RPI using a commin webcam)
videoDevice=-1

# Vector containing the 2D image coordinates
s_des = [0, 0, 0, 0, 0, 0, 0, 0]

# Set the video feed
cap = cv2.VideoCapture(videoDevice)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
#while True:
	# Capture a new frame from the webcam
	cframe_rgb = frame.array
	#cframe_rgb = cap.read()[1]
	#cframe_rgb = cv2.imread('input.png')

	# Convert the captured frame from RGB to GRAY scale
	cframe_gray = cv2.cvtColor(cframe_rgb, cv2.COLOR_BGR2GRAY)
	
 
	# Apply a HoughCircles, using HOUGH_GRADIENT method.
	circles = cv2.HoughCircles(cframe_gray, cv2.HOUGH_GRADIENT, dp, min_dist, par1, par2)
	print "Checking for circles"
	# Check if any circle have been detected
	if circles is not None:
		totalCircles = len(circles[0, :])
		print "Total: "+str(totalCircles)
		print circles
		if(totalCircles == 4):
                        print "Found 4 circles"
			s_des[0] = circles[0, 0, 0]
			s_des[1] = circles[0, 0, 1]
			s_des[2] = circles[0, 1, 0]
			s_des[3] = circles[0, 1, 1]
			s_des[4] = circles[0, 2, 0]
			s_des[5] = circles[0, 2, 1]
			s_des[6] = circles[0, 3, 0]
			s_des[7] = circles[0, 3, 1]
			print "Desired camera frame projected points: "
			print s_des
			break;

# Destroy all windows and exit
cap.release()
cv2.destroyAllWindows()
