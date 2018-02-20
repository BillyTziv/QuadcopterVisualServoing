import cv2

img = cv2.imread("logframe-input.jpg")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

binary = cv2.bitwise_not(gray)

(_,contours,_) = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

for contour in contours:
    rect = cv2.boundingRect(contour)
    cv2.rectangle(img, (rect[0],rect[1]), (rect[2]+rect[0],rect[3]+rect[1]), (0,255,0), 2)

cv2.imshow('img', img)
cv2.waitKey(0)
