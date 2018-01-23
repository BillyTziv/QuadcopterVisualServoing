# Calibration code for the desired s vector
#
# It takes an image which becomes the desired image

import cv2
import numpy as np
import sys

print "\nCircle tracking software stated...\n"

# Hough Parameters
dp=1			# The inverse ration of resolution
min_dist=50 	# Minimum distance between the detecetd cirlces
min_radius=20	# Min radius of the circle
max_radius=150	# Max radius of the circle
	
# Set the video device (-1 seems to work for RPI using a commin webcam)
videoDevice=-1

# Vector containing the 2D image coordinates
s_des = [0, 0, 0, 0, 0, 0, 0, 0]

# Set the video feed
cap = cv2.VideoCapture(videoDevice)

while True:
	# Receive frames from the camera, as long as it is open	
	# Convert the RGB image taken from the camera to greyscale 
	fr_rgb = cv2.cvtColor(cap.read()[1], cv2.COLOR_BGR2GRAY)
	
	# MedianBlur function, takes median of all pixels and replace each pixel in
	# in the input image 'mbImg' according to the ksize argument '5'. 
	fr_gray = cv2.medianBlur(fr_rgb, 5)
	    
	# Apply a HoughCircles, using HOUGH_GRADIENT method.
	circles = cv2.HoughCircles(fr_gray, cv2.HOUGH_GRADIENT, dp, min_dist, 50, 30)
	
	# Check if any circle have been detected
	if circles is not None:
		totalCircles = len(circles[0, :])
		
		if(totalCircles == 4):
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
