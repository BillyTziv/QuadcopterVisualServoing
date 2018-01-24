#
# This code open the video stream from a webcam and
# displays every frame in window.
#

import numpy as np
import cv2

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Add blur effect (important for night)
    #gray = cv2.medianBlur(gray, 5)
    
    # Apply a HoughCircles, using HOUGH_GRADIENT method
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 15, 200, 100)
    #print "Reading new frame..."
    
    # Check if any circle have been detected
    if (circles is not None) and (len(circles) == 4):
        print circles[0, :]

    # Display the resulting frame
    cv2.imshow('video', gray)
    
    # Break the loop if q is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
