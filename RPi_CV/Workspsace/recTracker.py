# Image Based Visual Servoing using a 6DOF camera module.

import cv2
import numpy as np
import sys
import serial
import time
import serial.tools.list_ports
from picamera.array import PiRGBArray
from picamera import PiCamera 
import math

print "\nImage Based Visual Servoing (IBVS) system initiated...\n"

# Check if the program started with the according arguments
if( len(sys.argv) != 4 ):
        sys.exit("[Error] No argument was specified. Try again with with the correct syntax.\neg. <brightness threshold> <max distance> <blur effect>")

print "Initializing the camera settings..."
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)
print "[INFO] Resolution was set at: 640x480"
print "[INFO] Framerate was set at: 32"

# The desired points will be initialized the first time in the loop. Setting this varialbe
# to True indicated that the s desired will be initialized by the first s vector found. After
# that it changes to FALSE.
SETDESIRED = True

# Constants and variables initialization
m = 1			# Total mass of the vehicle
g = 9.81		# Gravity constant
theta = [0, 0, 0]	# Euler angles Z-X-Y
vel = [0, 0, 0]		# Current velocity of the quadrotor
acc = [0, 0, 0]		# Acceleration of the quadrotor
posZ_des = 0		# Desired position in Z axis
posZ = 0		# Current position in Z axis
dt = 0.001		# Time for integrals
thetax_i = 0
thetay_i = 0
thetax_des_i = 0
thetay_des_i = 0
omega = [0, 0, 0]
torq_des_i = 0
thetadot = [0, 0, 0]
torq_des  = [0, 0, 0]
I = np.diag([5e-3, 5e-3, 10e-3])  # Inertia Matrix

k_thrust_p = 0.5	# Propotional constant for thrust PID controller
k_thrust_d = 0.2	# Derivative constant for thrust PID controller
k_thrust_i = 0.3	# Integral constant for thrust PID controller

k_torqy_p = 3.2
k_torqy_d = 0.5
k_torqy_i = 0.2

k_torqx_p = 3.2
k_torqx_d = 0.5
k_torqx_i = 0.2

# Desired points which are initialized in the loop using the SETDESIRED variable.
s_des = []

# This vector will contain the current points in every frame. s vector contain the X and Y 
# coordinated of the 3D space projected points in the camera frame.
s = []

# Pixel Brightness threshold is the max value of pixels we accept
pb_threshold = int(sys.argv[1])
print "[INFO] Pixel brightness threshold was set at (max): ", pb_threshold

# Distance between coordinates to accept into the s vector
c_dist = int(sys.argv[2])
print "[INFO] Distance between points was set at (max): ", c_dist

# Estimation of depth for Z axis of the camera.
x1_Z = 1
x2_Z = 1
x3_Z = 1
x4_Z = 1

# Find the connection with the arduino board
print "Searching for arduino tty port..."
for p in serial.tools.list_ports.comports():
	if "ACM" in p[1]:
        	arduinoPort = p[0]
		break
	else:
		arduinoPort = None

if(arduinoPort is not None):
	print "[INFO] Arduino was found connected at port: ", arduinoPort
else:
	sys.exit("[ERROR] No arduino found with the port name ttyACM*.")

print "[INFO] Serial communication established at: ", arduinoPort

ser = serial.Serial('/dev/ttyACM0', 9600)

# Wait till arduino signals that it is ready
data = ser.readline()
print data

# Send a signal to arm the quadrotor
print "[INFO] Sending ARM command to the quadrotor"
ser.write("<ARM>")
print "Waiting for arduino to respond..."
#data = ser.readline()
#print data
#print ""

# Display a waiting message (for fun)
sys.stdout.write("System LOOP will be activated in 3")
sys.stdout.flush()
time.sleep(1)
sys.stdout.write(" 2")
sys.stdout.flush()
time.sleep(1)
sys.stdout.write(" 1")
sys.stdout.flush()
time.sleep(1)
print ""

# Count how many frames passed
fr_counter = 0

# Start the loop
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# Display a new section
	#print "================= FRAME ", str(fr_counter), " ================="
	fr_counter = fr_counter+1

	# Convert the array captured to an image
	cframe_rgb = frame.array		# Get the frame from the onboard pi camera
	#cframe_rgb = cap.read()[1]		# Get the frame from a webcam
	#cframe_rgb = cv2.imread('input.png')	# Get the frame from a file

	# Convert the captured frame from RGB to GRAY scale
	cframe_gray = cv2.cvtColor(cframe_rgb, cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(cframe_gray, 50, 255, 1)
	#cv2.imshow('Threshold image', thresh)

	im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	ENABLE_ROI=0
	#print "contours #", len(contours)
	for cnt in contours:
		approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
		if(len(approx) == 4):
			perimeter = cv2.arcLength(cnt, True)
			#print "Contour perimeter: ", len(cnt)
			#cv2.drawContours(cframe_rgb, cnt, -1, (0,255,0), 3)
			x, y, w, h = cv2.boundingRect(cnt)
    			roi = cframe_gray[y+10:y+h-10, x+10:x+w-10]
			roi[cnt[0]] = 255

			roi_out = cframe_rgb[y:y+h, x:x+w]
			cv2.imshow('Region of Interest', roi)
			ENABLE_ROI = 1
	if ENABLE_ROI == 0:
		print "[WARNING] ROI was not found! "
		rawCapture.truncate(0)
		continue

	# Change all the pixels over 100 to white pixels (255)
	#roi[roi > 100] = 255
	#cframe_gray = roi
	#cframe_rgb = roi
	# Clera buffers for the next read
	rawCapture.truncate(0)
	# Apply a blur filter (optional)
	#cv2.imwrite('before.jpg', cframe_gray)
	#cframe_gray= cv2.medianBlur(cframe_gray, int(sys.argv[3]))
	#cv2.imwrite('after.jpg', cframe_gray)
	
	# Create a list with all the black 2D coordinates found
	blackList = np.argwhere(cframe_gray < pb_threshold)
	#print "Blacklist contains total items: ", blackList

	# Approximate the center of the points max distance using the number
	# of the black points found.
	#dist = len(blackList)/70
	#print "DISTANCE ==== > ", dist
	#if(dist >=45):
	#	c_dist = 100
	#else:
	#	c_dist = 60
	# Check if the list with the black points is empty and go to the next loop
	if len(blackList) == 0:
		print "[WARNING] No points found!"
		s = []
		#print "=============================================\n"
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
				if( ( abs(blackPoint[0]-sPoint[0]) < c_dist ) and ( abs(blackPoint[1]-sPoint[1]) < c_dist ) ):
					EXISTS=1
			if EXISTS==0:	
				#print "New element match :: Inserting: ", blackPoint, "with brighness: ", cframe_gray[blackPoint[0]][blackPoint[1]]
				s.append(blackPoint)

	#Update the previous s vector with the current
	s_prev = s

	# Calculate the number of points found.
	numOfPoints = len(s)

        if numOfPoints == 4:	# Number of points are four. Perfect match here!
		# Prepare the string for the arduino (must start with '<' and end with '>')
		s_raw = s[0][1], s[0][0], s[1][1], s[1][0], s[2][1], s[2][0], s[3][1], s[3][0]
		if(SETDESIRED == True):
			s_des = s_raw
			SETDESIRED = False
		#print s_raw
		# Get the brighness of the current points
		p1_value = roi[s[0][0]][s[0][1]]
		p2_value = roi[s[1][0]][s[1][1]]
		p3_value = roi[s[2][0]][s[2][1]]
		p4_value = roi[s[3][0]][s[3][1]]
		
		# There are two options here. Either a black points is found first or a gray one. If the first one is black
		# then we follow a specific order according to the normal way of image scanning.
		if( p1_value < 10 ):
                	x1_cm = s_raw[0]
                	y1_cm = s_raw[1]
	                x2_cm = s_raw[2]
        	        y2_cm = s_raw[3]
                	x4_cm = s_raw[4]
                	y4_cm = s_raw[5]
                	x3_cm = s_raw[6]
                	y3_cm = s_raw[7]
		else:
                        x1_cm = s_raw[2]
                        y1_cm = s_raw[3]
                        x2_cm = s_raw[0]
                        y2_cm = s_raw[1]
                        x3_cm = s_raw[4]
                        y3_cm = s_raw[5]
                        x4_cm = s_raw[6]
                        y4_cm = s_raw[7]		

		# Calculate the L1 and L2 matrixes
                L = [[-1/x1_Z, 0, x1_cm/x1_Z, y1_cm*x1_cm, -(1+x1_cm**2), y1_cm], [0, -1/x1_Z, y1_cm/x1_Z, 1+y1_cm**2, -x1_cm*y1_cm, -x1_cm], \
                     [-1/x2_Z, 0, x2_cm/x2_Z, y2_cm*x2_cm, -(1+x2_cm**2), y2_cm], [0, -1/x2_Z, y2_cm/x2_Z, 1+y2_cm**2, -x2_cm*y2_cm, -x2_cm], \
                     [-1/x3_Z, 0, x3_cm/x3_Z, y3_cm*x3_cm, -(1+x3_cm**2), y3_cm], [0, -1/x3_Z, y3_cm/x3_Z, 1+y3_cm**2, -x3_cm*y3_cm, -x3_cm], \
                     [-1/x4_Z, 0, x4_cm/x4_Z, y4_cm*x4_cm, -(1+x4_cm**2), y4_cm], [0, -1/x4_Z, y4_cm/x4_Z, 1+y4_cm**2, -x4_cm*y4_cm, -x4_cm]]

                # Calculate the error between the s and s_des vectors
                #print "Desired s: ", s_des[0], s_des[1], s_des[2], s_des[3], s_des[4], s_des[5], s_des[6], s_des[7]
                #print "Current s: ", s_raw[0], s_raw[1], s_raw[2], s_raw[3], s_raw[4], s_raw[5], s_raw[6], s_raw[7]
                error = np.subtract(s_raw, s_des)

                # Velocity controller IBVS
		#print "DESIRED S VECTOR :: ", s_des
		#print "CURRENT S VECTOR :: ", s_raw
		#print "INTERATION MATRIX L :: ", L
		
		#print "S ERROR :: ", error
		lamda = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 1.5])
		v_cam = np.dot(-lamda, np.dot(np.linalg.pinv(L), np.transpose(error)))# - np.dot(np.linalg.pinv(L1), np.dot(L2, np.transpose(omega)))
		#print "CAM VELOCITY :: ", v_cam[0], v_cam[1], v_cam[2], v_cam[5] 
		#print "Desired cam velocity :: X:", int(v_cam[0]), "Y: ", int(v_cam[0][1]), "Z: ", int(v_cam[0][2])
		#print "\nFor the matlab"
		#print "current s :", s_raw[1], s_raw[0], s_raw[3], s_raw[2],s_raw[5], s_raw[4],s_raw[7], s_raw[6]
		#print "desired s :", s_des[1], s_des[0], s_des[3], s_des[2],s_des[5], s_des[4],s_des[7], s_des[6]

		# Define the 3 euler angles for each axis		
                psi = theta[0]  # Z axis
                phi = theta[1]  # X axis
                yaw = theta[2]  # Y axis

		# Rotation matrix from the Camera Frame to the Bodyframe (180 deg angle in X (phi) axis)
		R1 = [[1, 0, 0], [0, math.cos(phi), -math.sin(phi)], [0, math.sin(phi), math.cos(phi)]]
		#print "R_C_to_B : ", R1
		
                # Rotation matrix from the Bodyframe to the Velocity Coordinate System (yaw deg angle in Z (psi) axis)
                R2 = [[math.cos(psi), -math.sin(psi), 0], [math.sin(psi), math.cos(psi), 0], [0, 0, 1]] 
                #print "R_B_to_V : ", R2
   		
		# Rotation matrix from the  Bodyframe to the Inertial
                R =  [[math.cos(psi) * math.cos(yaw) - math.sin(yaw) * math.sin(phi) * math.sin(psi), -math.cos(phi) * math.sin(psi), math.cos(psi)*math.sin(yaw)+math.sin(psi)*math.sin(phi)*math.cos(yaw)], \
                        [math.cos(yaw) * math.sin(psi) + math.cos(psi) * math.sin(phi)* math.sin(yaw), math.cos(psi) * math.cos(phi), math.sin(psi)*math.sin(yaw)-math.cos(psi) * math.sin(phi)*math.cos(yaw)], \
                        [math.cos(phi) * math.sin(yaw), math.sin(phi), math.cos(phi)*math.cos(yaw)]]
                #print "R_B_to_I", R
		
		# Desired velocity of the camera expressed in the Bodyframe coordinate system.
		v_c_des_B = np.dot(R1, np.transpose([v_cam[0], v_cam[1], v_cam[2]]))
		print "Desired Camera Velocity[BODYFRAME]: ", v_c_des_B
		'''	
		# Desired velocity of the camera expressed in the inertial coordinate system
		v_c_des_V = np.dot(np.dot(R2, R1), [v_cam[0], v_cam[1], v_cam[2]])
		#print "Des Cam Vel in the Inertial: ", v_c_des_V

		# ======================================================== checked ==============================

		# Express the current drone velocity from the inertial to the bodyframe
		vel_quad_B = np.transpose(R) * vel
		#print "Current Quad Vel in the Bodyframe: ", vel_quad_B
		
		# Express the current drone velocity from the inertial to the velocity coordinate system
		vel_quad_V = np.dot(np.dot(R2, np.transpose(R)), vel)
		#print "Current Quad Vel in the Velocity Coordinate System: ", vel_quad_V
		
		# PID velocity controller for the total quadcopter thrust.
		posZ_des = posZ_des + dt * v_c_des_B[2]
		posZ = posZ + dt * vel_quad_B[2]
		
		# PID controller for the desired thrust of the quadrotor
		thrust_des = (m*g) + k_thrust_p*(v_c_des_B[2] - vel_quad_B[2]) - k_thrust_d*acc[2] + k_thrust_i*(posZ_des - posZ)
		#print "Total thrust: ", thrust_des
		
		# Calculate the desired theta (roll) angle via a P velocity controller in X axis.a
		theta_x_des = -0.6*(m/thrust_des)*(v_c_des_V[0] - vel_quad_V[0])
		
		# PID controller for the torque in X axis
		thetax_des_i = thetax_des_i + dt*theta_x_des 	# Integral of the desired theta angle in X axis.
		thetax_i = thetax_i + dt * theta[1] 		# Integral of theta angle in X axis;
		torq_des[0] = k_torqy_p * (theta_x_des - theta[1]) - k_torqy_d*thetadot[1] + k_torqy_i*(thetax_des_i - thetax_i)
		
		# Calculate the desired theta (pitch) angle via a P velocity controller in Y axis.
		theta_y_des = 0.6*(m/thrust_des)*(v_c_des_V[1] - vel_quad_V[1]);
		
		# PD controller for the torque in Y axis.
		thetay_des_i = thetay_des_i + dt*theta_y_des; # Integral of the desired theta angle in Y axis.
		thetay_i = thetay_i + dt * theta[2]; # Integral of theta angle in Y axis;
		torq_des[1] = k_torqx_p*(theta_y_des - theta[2]) - k_torqx_d*thetadot[2] + k_torqx_i*(thetay_des_i - thetay_i);
		
		# Convert the current angular velocities to theta dot
		W = [[0, math.cos(psi), -math.cos(phi)*math.sin(psi)], [0, math.sin(psi), math.cos(phi)*math.cos(psi)], [1, 0, math.sin(phi)]]
		td = np.linalg.pinv(W)*np.transpose([omega[0], -omega[1], -v_cam[2]])
		
		# P controller for the torque in Z axis.
		torq_des[2] = 1.5*(td[0] - thetadot[0]);

		# Calculate the updated omega dot
		#print "=>", np.cross(-np.transpose(omega), np.dot(I, np.transpose(omega)))
		#print torq_des
		#print [torq_des[0], torq_des[1], torq_des[2]]
		omegadot= np.dot(np.linalg.pinv(I), np.cross(-np.transpose(omega), np.dot(I, np.transpose(omega))) + [torq_des[0][0], torq_des[0][1], torq_des[0][2]])
		
		# Calculate quadcopter omega from omega dot
		omega = omega + dt * omegadot
		'''

		# Display with green circles the current points
		#print "s vector: ", s, str(len(s))
		#print "s des vec:", s_des, str(len(s_des)-1)

		# Paint red and green ricles for the current and desired points	
		for p in range(0, 7, 2):
			cv2.circle(roi_out, (s_raw[p], s_raw[p+1]), 8, (0, 255, 0), -1)
			#print s[p][1]
	
		# Display with red circles the desired points
		for l in range(0, 7, 2):
			cv2.circle(roi_out, (s_des[l], s_des[l+1]), 5, (0, 0, 255), -1)
			#print s_des[l+1]
	
		# 20 for X and 80 for Y
		cv2.circle(roi_out, (20, 80), 15, (150, 77, 10), -1)
		#cv2.imwrite('output.jpg', cframe_rgb)
		cv2.imshow('OUTPUT', roi_out)
	
	elif(numOfPoints > 4):	# More than 4 points found. Decrease the c_dist parameter.
		print "[WARNING] More than 4 points found. Total points: ", len(s)
		print "Points found: ", s

		print "[INFO] Decreasing c_dist parameter by 10"
		c_dist = c_dist - 10
	elif(numOfPoints < 4):	# Less then 4 points found. Increase the c_dist parameter.
		print "[WARNING] Less than 4 points found. Total points: ", len(s)
		print "Points found: ", s

		print "[INFO] Increasing c_dist parameter by 10"
		c_dist = c_dist + 10

	# Clear whatever need to be empty for the next loop
	s = []
	
	# Break the loop	
    	if cv2.waitKey(1) & 0xFF == ord('q'):
        	break

# Destroy all windows and exit
cv2.destroyAllWindows()

