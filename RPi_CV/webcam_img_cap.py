# Capturs an image from an ordinate webcam, connected via usb cable

import cv2
import numpy as np

print "\nImage saving using OpenCV script...\n"

# Set the video device (-1 seems to work for RPI using a commin webcam)
videoDevice=-1

# Set the video feed
cap = cv2.VideoCapture(videoDevice)

# Receive frames from the camera, as long as it is open	
# Convert the RGB image taken from the camera to greyscale 
fr_rgb = cv2.cvtColor(cap.read()[1], cv2.COLOR_BGR2GRAY)

# MedianBlur function, takes median of all pixels and replace each pixel in
# in the input image 'mbImg' according to the ksize argument '5'. 
#fr_gray = cv2.medianBlur(fr_rgb, 5)

# Save the image captured in the same folder
cv2.imwrite('capturedImage.jpg', fr_gray)    

# Destroy all windows and exit
cap.release()
cv2.destroyAllWindows()
