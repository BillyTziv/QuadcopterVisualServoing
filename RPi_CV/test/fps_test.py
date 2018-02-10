import numpy as np
import time
import cv2

start = time.time()
time.clock()
secElapsed=0
counter=0
cap = cv2.VideoCapture(0)
cap.set(3, 40) # Width
cap.set(4, 80) # Height

while(secElapsed < 10):
    # Update the seconds elapsed
    secElapsed = time.time()-start

    # Capture frame-by-frame
    ret, frame = cap.read()
    counter=counter+1
    
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)

    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 50, 20, 150)

    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print "Total frames read: "+str(counter/10)
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
