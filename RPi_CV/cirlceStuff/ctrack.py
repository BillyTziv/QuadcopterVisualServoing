# This python code, detectes 4 circles and returns a vector containing
# the 2D coordinates of each circle.

import cv2
import numpy as np
import sys
import serial
import time

print "\nCircle tracking software stated...\n"


	
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

        # Set frame width and height
        max_width=400
        max_height=400
	print "Set the max width and height to: "+str(max_width)+"x"+str(max_height)
	cap.set(3, max_width) # Max window frame width
	cap.set(4, max_height) # Max window frame height
	
	# Check for errors and print messages
	if(cap.isOpened()):
		return cap
	else:
		sys.exit("An error occured while opening/using the camera port!")

# According to the error between s and s_des try to land to the ground.
def land(cap):
	# Receive frames from the camera, as long as it is open	
	# Convert the RGB image taken from the camera to greyscale 
	fr_rgb = cv2.cvtColor(cap.read()[1], cv2.COLOR_BGR2GRAY)
	
	# MedianBlur function, takes median of all pixels and replace each pixel in
	# in the input image 'mbImg' according to the ksize argument '5'. 
	fr_gray = cv2.medianBlur(fr_rgb, 5)
	    
	# Apply a HoughCircles, using HOUGH_GRADIENT method.
	# Hough Parameters
        dp=2			# The inverse ration of resolution
        min_dist=5 	# Minimum distance between the detecetd cirlces
        par1=250
        par2=100
	circles = cv2.HoughCircles(fr_gray, cv2.HOUGH_GRADIENT, dp, min_dist, par1, par2)
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
			return circles[0, 0, 2]

def main():
	print "Opening serial communication with arduino..."
	ser=serial.Serial('/dev/ttyACM0', 9600)

	# Initialize the camera and check for any errors
	print "Initializing camera settings..."
	cap_img = initializeCamera()
	
	print "Start the tik-tak clock..."
	start = time.time()
	time.clock()
	secElapsed=0
	counter=0
	
	print "Running..."
	while (secElapsed < 10):
		# Update the seconds elapsed
		secElapsed = time.time()-start
		# Run the landing algorithm
		#print "Reading radius"
		c_radius = land(cap_img)
		print c_radius
		counter=counter+1

		# Calculate desired velocities
		#v_des = r
		
		# Send data to Arduino
		if(c_radius is not -1):
                    strToSend="<"+str(c_radius)+">"
                    ser.write(strToSend)
                    print strToSend

	# Destroy all windows and exit
	print "Execution was successful..."
	print "Total frames read: "+str(counter/10)
	print "Last points vector returned: "+str(s)
	
	print "Closing serial communication..."
	ser.close()

        print "Reseasing the camera port..."
        cap_img.release()

        print "Destroying all windows..."
	cv2.destroyAllWindows()

if __name__ == "__main__":
	main()