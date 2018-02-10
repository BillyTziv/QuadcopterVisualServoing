# Capture an image from pi camera

import cv2
import numpy as np
import sys
from picamera.array import PiRGBArray
from picamera import PiCamera 

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# Capture a new frame from the webcam
	cframe_rgb = frame.array
	print cframe_rgb[100, 100]
	# Convert the captured frame from RGB to GRAY scale
	cframe_gray = cv2.cvtColor(cframe_rgb, cv2.COLOR_BGR2GRAY)

	# Save the image captured
	print "Saving image as: 'picam_image.jpg'"
	cv2.imwrite('picam_image.jpg', cframe_rgb)
	break
