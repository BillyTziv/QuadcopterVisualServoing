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

print "\nBlack points tracking started...\n"

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)

# Vector containing the 2D image coordinates
s_des = [ 343, 120, 193, 122, 342, 258, 183, 267 ]

# Vector with the current X and Y coordinates in the image frame
s = []

# Image points threshold
pb_threshold = 50 	# Pixel Brightness thrshold is the max value of pixels we accept
c_dist = 25		# Distance between coordinates to accept into the s vector

# Vector with the previous X and Y coordinates in the image frame
# in case the camera does not detect the 4 target points. 
s_prev = []

# Z for all points
x1_Z = 1
x2_Z = 1
x3_Z = 1
x4_Z = 1

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


print "Arming the quadrotor"
strToSend = "<962, 1464, 1464, 1954>"
ser.write(strToSend);
time.sleep(3)
strToSend = "<1050, 1464, 1464, 1464>"
ser.write(strToSend)
		
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# Capture a new frame from the webcam
	cframe_rgb = frame.array
	#cframe_rgb = cap.read()[1]
	#cframe_rgb = cv2.imread('input.png')

	# Convert the captured frame from RGB to GRAY scale
	cframe_gray = cv2.cvtColor(cframe_rgb, cv2.COLOR_BGR2GRAY)
	cframe_gray[cframe_gray >60] = 255

	rawCapture.truncate(0)
	# Apply a blur filter (optional)
	# cframe_blur = cv2.medianBlur(cframe_gray, 5)
	
	# Create a list with all the black 2D coordinates found (a point must have a brighness
	# lower than 10 to be black)
	blackList = np.argwhere(cframe_gray < pb_threshold)
	counter = 1
	if len(blackList) == 0:
		continue	
	# Run through all of the coordinates and keep one record for each unique point found.
	for blackPoint in blackList:
		# If s vector is null insert the black point (initialize the s vector)
		if not s:
			#print "First element :: Inserting: ", blackPoint, "with brighness: ", cframe_gray[blackPoint[0]][blackPoint[1]]
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
				#print "New element match :: Inserting: ", blackPoint, "with brighness: ", cframe_gray[blackPoint[0]][blackPoint[1]]
				s.append(blackPoint)

	#Update the previous s vector with the current
	s_prev = s

        if len(s) == 4:
		# Prepare the string for the arduino (must start with '<' and end with '>')
		s_raw = s[0][1], s[0][0], s[1][1], s[1][0], s[2][1], s[2][0], s[3][1], s[3][0]
                #print s_raw
                
		# Initialze the camera frame coordinate vectors
                x1_cm = s[0][1]
                y1_cm = s[0][0]
                x2_cm = s[1][1]
                y2_cm = s[1][0]
                x3_cm = s[2][1]
                y3_cm = s[2][0]
                x4_cm = s[3][1]
                y4_cm = s[3][0]
                
		# Calculate the L1 and L2 matrixes
                L1 = [[-1/x1_Z, 0, x1_cm/x1_Z, y1_cm], [0, -1/x1_Z, y1_cm/x1_Z, -x1_cm], \
                        [-1/x2_Z, 0, x2_cm/x2_Z, y2_cm], [0, -1/x2_Z, y2_cm/x2_Z, -x2_cm], \
                        [-1/x3_Z, 0, x3_cm/x3_Z, y3_cm], [0, -1/x3_Z, y3_cm/x3_Z, -x3_cm], \
                        [-1/x4_Z, 0, x4_cm/x4_Z, y4_cm], [0, -1/x4_Z, y4_cm/x4_Z, -x4_cm]]

                L2 = [[x1_cm*y1_cm, -(1+x1_cm**2)], [1+y1_cm**2, -x1_cm*y1_cm], \
                        [x2_cm*y2_cm, -(1+x2_cm**2)], [1+y2_cm**2, -x2_cm*y2_cm], \
                        [x3_cm*y3_cm, -(1+x3_cm**2)], [1+y3_cm**2, -x3_cm*y3_cm], \
                        [x4_cm*y4_cm, -(1+x4_cm**2)], [1+y4_cm**2, -x4_cm*y4_cm]]

                # Calculate the error between the s and s_des vectors
                #print "Desired s: ", s_des
                #print "Current s: ", s_raw
                error = np.subtract(s_raw, s_des)

                # Velocity controller IBVS
		#print s_des
		#print s_raw
		#print L1
		#print L2
		#print "Error :: ", error
                omega = [[0, 0]]
		v_cam =  np.transpose(np.dot(-np.diag([0.1, 0.1, 0.1, 0.1]), np.dot(np.linalg.pinv(L1), error))) - np.dot(np.linalg.pinv(L1), np.dot(L2, np.transpose(omega)))
		print "Desired cam velocity :: X:", int(v_cam[0][0]), "Y: ", int(v_cam[0][1]), "Z: ", int(v_cam[0][2])
		thr=1300
		rud=1464
		if(v_cam[0][0] > 0):
			ail=1350
			print "Aileron LEFT"
		elif(v_cam[0][0] < 0):
			ail=1650
                        print "Aileron RIGHT"
                        
                if(v_cam[0][1] > 0):
			ele=1650
			print "Elevator UP"
			
		elif(v_cam[0][1] < 0):
			ele=1350
                        print "Elevator DOWN"
		strToSend = "<962, ail, ele, 1464>"

		# Write the string to the serial usb cable
		ser.write(strToSend)
                
	
	# Clear whatever need to be empty for the next loop
	s = []

        cv2.circle(cframe_rgb, (15, 15), 15, (255, 0, 0), -1)
        cv2.circle(cframe_rgb, (50, 50), 15, (0, 255, 0), -1)
        cv2.circle(cframe_rgb, (250, 15), 15, (0, 0, 255), -1)
	
	# Display with green circles the desired points
	cv2.circle(cframe_rgb, (s_des[0], s_des[1]), 5, (100, 255, 0), -1)
	cv2.circle(cframe_rgb, (s_des[2], s_des[3]), 5, (100, 255, 0), -1)
	cv2.circle(cframe_rgb, (s_des[4], s_des[5]), 5, (100, 255, 0), -1)
	cv2.circle(cframe_rgb, (s_des[6], s_des[7]), 5, (100, 255, 0), -1)
	
	# Display the current and desired points in a window
	cv2.imshow('frame', cframe_rgb)
	#print "\n"
	
	# Display the resulting frame
    	if cv2.waitKey(1) & 0xFF == ord('q'):
        	break
# Destroy all windows and exit
cv2.destroyAllWindows()

