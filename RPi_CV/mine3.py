# Calibration code for the desired s vector
#
# It takes an image which becomes the desired image

import cv2
import numpy as np
import sys
import serial
import time
import serial.tools.list_ports
from picamera.array import PiRGBArray
from picamera import PiCamera 
print "\nCircle tracking software stated...\n"

# Hough Parameters
dp=2			# The inverse ration of resolution
min_dist=150 		# Minimum distance between the detecetd cirlces
par1=200
par2=100

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)

# Set the video device (-1 seems to work for RPI using a commin webcam)
videoDevice=-1

# Vector containing the 2D image coordinates
s_des = [0, 0, 0, 0, 0, 0, 0, 0]

# Vector with the current X and Y coordinates in the image frame
s = []

# Image points threshold
pb_threshold = 20 	# Pixel Brightness thrshold is the max value of pixels we accept
c_dist = 20		# Distance between coordinates to accept into the s vector

# Vector with the previous X and Y coordinates in the image frame
# in case the camera does not detect the 4 target points. 
s_prev = []

# Z for all points
x1_Z = 1
x2_Z = 1
X3_Z = 1
x4_Z = 1

# Set the video feed
cap = cv2.VideoCapture(videoDevice)

print "Searching for arduino tty port..."
for p in serial.tools.list_ports.comports():
	if "ACM" in p[1]:
        	arduinoPort = p[0]
		break
	else:
		arduinoPort = None

if(arduinoPort is not None):
	print "Arduino was found connected at port: ", arduinoPort
else:
	sys.exit("Error, no arduino found with the port name ttyACM*.\nWill now exit.\n")

print "Creating serial communication with arduino at ", arduinoPort

ser = serial.Serial('/dev/ttyACM0', 9600)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
#while True:
	# Capture a new frame from the webcam
	cframe_rgb = frame.array
	#cframe_rgb = cap.read()[1]
	#cframe_rgb = cv2.imread('input.png')

	# Convert the captured frame from RGB to GRAY scale
	cframe_gray = cv2.cvtColor(cframe_rgb, cv2.COLOR_BGR2GRAY)
	
	# Apply a blur filter (optional)
	# cframe_blur = cv2.medianBlur(cframe_gray, 5)
	
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

	#Update the previous s vector with the current
	s_prev = s
	print s

        if len(s) == 4:
		# Prepare the string for the arduino (must start with '<' and end with '>')
		s_raw = s[0][0], s[0][1], s[1][0], s[1][1], s[2][0], s[2][1], s[3][0], s[3][1]

		# Calculate the error between s and s _des
		#error = s - s_des
		
		# Find the velociteis of the desired camera
		#v_cam_des = L1 * 

		strToSend = "<"+str(s_raw)+ ">"
		#print strToSend	
		# Write the string to the serial usb cable
		ser.write(strToSend)

		# Initialze the camera frame coordinate vectors
		x1_cm = s[0][0]
		y1_cm = s[0][1]
		x2_cm = s[1][0]
		y2_cm = s[1][1]
		x3_cm = s[2][0]
		y3_cm = s[2][1]
		x4_cm = s[3][0]
		y4_cm = s[3][1]

		# Calculate the L1 and L2 matrixes
		L1 = [[-1/x1_Z, 0, x1_cm/x1_Z, y1_cm], [0, -1/x1_Z, y1_cm/x1_Z, x1_cm], \
			[-1/x2_Z, 0, x2_cm/x2_Z, y2_cm], [0, -1/x2_Z, y2_cm/x2_Z, x2_cm], \
			[-1/x3_Z, 0, x3_cm/x3_Z, y3_cm], [0, -1/x3_Z, y3_cm/x3_Z, x3_cm],\
			[-1/x4_Z, 0, x4_cm/x4_Z, y4_cm], [0, -1/x4_Z, y4_cm/x4_Z, x4_cm]]

		L2 = [[x_cm1*y_cm1, -(1+x_cm1^2)], [1+y_cm1^2, -x_cm1*y_cm1] \
			[x_cm2*y_cm2, -(1+x_cm2^2)], [1+y_cm2^2, -x_cm2*y_cm2] \
			[x_cm3*y_cm3, -(1+x_cm3^2)], [1+y_cm3^2, -x_cm3*y_cm3] \
			[x_cm4*y_cm4, -(1+x_cm4^2)], [1+y_cm4^2, -x_cm4*y_cm4]]

		# Calculate the error between the s and s_des vectors
		error = np.substract(s_des, s)

		# Velocity controller IBVS
		omega = [0 0 0]
		
		v_cam = (-np.diag([1 1 1 1]) * np.transpose(L1) * error) - (np.transpose(L1) * L2 * omega)
		print v_cam

	# Clear the s vector for the next loop
	s = []
	rawCapture.truncate(0)
	cv2.imshow('frame', cframe_gray)
	# Display the resulting frame
    	if cv2.waitKey(1) & 0xFF == ord('q'):
        	break
# Destroy all windows and exit
cap.release()
cv2.destroyAllWindows()

