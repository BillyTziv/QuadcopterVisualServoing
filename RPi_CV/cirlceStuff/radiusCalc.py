# This python code, detectes 4 circles and returns a vector containing
# the coordinates of each circle.

import cv2
import numpy as np
import sys

print "======================================"
print "\nCircle radius tracking stated...\n"
print "======================================"

# Hough Parameters
dp=2			# The inverse ration of resolution
min_dist=50 	# Minimum distance between the detecetd cirlces
par1=200
par2=100

# Frame window size
max_height=480
max_width=640

# Set the video device (0 seems to work for RPI using a commin webcam)
videoDevice=0
def main():
	# Initialize the camera and check for any errors
	print "Initializing camera feed"
	cap = cv2.VideoCapture(videoDevice)
	if(cap.isOpened() == False):
		sys.exit("Error when opening the video feed.")
		
	# Set frame width and height
	print "Set the max width and height to "+str(max_width)+"x"+str(max_height)
	cap.set(3, max_width) # Width
	cap.set(4, max_height) # Height
	
	print "Capturing frames...\n"
	while True:
		# Convert the RGB image taken from the camera to greyscale
		ret, frame = cap.read()
		fr_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		# MedianBlur function, takes median of all pixels and replace each pixel in
		# in the input image 'mbImg' according to the ksize argument '5'. 
		#fr_gray = cv2.medianBlur(fr_rgb, 1)
		
                # Apply a HoughCircles, using HOUGH_GRADIENT method.
                circles = cv2.HoughCircles(fr_gray, cv2.HOUGH_GRADIENT, dp, min_dist, par1, par2)
                print "Checking for circles"
                # Check if any circle have been detected
                if (circles is not None:
                        totalCircles = len(circles[0, :])
                        print "Total: "+str(totalCircles)
                        print circles
                        if(totalCircles == 4):
                                print "Found 4 circles"
 
			print "Desired camera frame projected points: "
			print s_des
			break;			
		# Uncomment those lines if GUI is available
		cv2.imshow('video', fr_gray)
		
		# If ESC key is pressed break the loop and exit the program
		if cv2.waitKey(1)==27:
			break
			
	# Destroy all windows and exit
	print "\nReleasing the camera feed..."
	cap.release()
	print "Closing all video frame windows..."
	cv2.destroyAllWindows()
	print "Program ended.\n\n"

if __name__ == "__main__":
	main()
