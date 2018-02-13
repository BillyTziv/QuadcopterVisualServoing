# Capture an image from pi camera

import cv2
import numpy as np
import sys
from picamera.array import PiRGBArray
from picamera import PiCamera 

# Define the PI camera which is connected to the onboard slot.
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

print "Captuing a new frame..."
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

	# Capture a new frame from the webcam
	img_bgr = frame.array
	
	# Convert the captured image from BGR to GRAY scale
	img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
	
	# Save the image captured
	print "Saving the captured frame (GRAYSCALE) as: 'RPI_capture_GRAY.png'"
	cv2.imwrite('RPI_capture_GRAY.png', img_gray)
	
	print "Saving the captured frame (BGR) as: 'RPI_capture_BGR.png'"
	cv2.imwrite('RPI_capture_BGR.png', img_bgr)
	break
