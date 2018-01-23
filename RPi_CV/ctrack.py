# This python code, detectes 4 circles and returns a vector containing
# the coordinates of each circle.

import cv2
import numpy as np
import sys
import serial
import time

print "\nCircle tracking software stated...\n"

# Hough Parameters
dp=1			# The inverse ration of resolution
min_dist=50 	# Minimum distance between the detecetd cirlces
min_radius=20	# Min radius of the circle
max_radius=150	# Max radius of the circle
	
# Set the video device (-1 seems to work for RPI using a commin webcam)
videoDevice=-1

# Vector containing the 2D image coordinates
s = [0, 0, 0, 0, 0, 0, 0, 0]

# Desired vector containing the 2D image coordinates
s_des = [532.5, 363.5, 153.5, 384.5, 511.5, 25.5, 107.5, 36.5]

# Initializes the camera module by defining the port and checking 
# for any errors.
def initializeCamera():
	# Set the video device (-1 seems to work for RPI using a commin webcam)
	videoDevice=-1

	# Set the video feed
	cap = cv2.VideoCapture(videoDevice)

	# Check for errors and print messages
	if(cap.isOpened()):
		return cap
	else:
		sys.exit("An error occured while opening/using the camera port")

# According to the error between s and s_des try to land to the ground.
def land(cap):
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
			s[0] = circles[0, 0, 0]
			s[1] = circles[0, 0, 1]
			s[2] = circles[0, 1, 0]
			s[3] = circles[0, 1, 1]
			s[4] = circles[0, 2, 0]
			s[5] = circles[0, 2, 1]
			s[6] = circles[0, 3, 0]
			s[7] = circles[0, 3, 1]
			print s

			# For each circle found
			for i in circles[0, :]:
				# Calculate the center and radius of the circle
				center = (i[0], i[1])
				radius = i[2]
				
				# Create a new circle and a center in the frame for visual purposes
				cv2.circle(fr_gray, (center[0], center[1]), radius, (0,255,0), 3)
				cv2.circle(fr_gray, (center[0], center[1]), 2, (255,0,0), 3)
				return radius
	# Uncomment those lines if GUI is available
	cv2.imshow('video', fr_gray)
	# If ESC key is pressed break the loop and exit the program
#	if cv2.waitKey(1)==27:
#		break

def main():
	print "Opening serial communication with arduino..."
	ser=serial.Serial('/dev/ttyACM0', 9600)
	print "OK"

	# Initialize the camera and check for any errors
	print "Initializing camera"
	cap_img = initializeCamera()
	print "OK"
	
	error =2 
	while True:
		# Run the landing algorithm
		print "Reading radius"
		r = land(cap_img)
		print r

		# Calculate desired velocities
		v_des = r
		strToSend="<"+str(r)+">"
		ser.write(strToSend)
		time.sleep(0.10)
		print "End of story"
		# Update the error value
		#error = s-s_des
	# Destroy all windows and exit
	cap_img.release()
	cv2.destroyAllWindows()


if __name__ == "__main__":
	main()
