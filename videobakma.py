import cv2
import numpy
import time

path = './YesilTespit.avi'
cap = cv2.VideoCapture(path)
frame_number = 1

while True:
    ret, frame = cap.read()
    if ret:
        frame_copy = frame.copy() 
        
        frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)   # HSV Format
        
        cv2.imshow('Tespit',frame_copy)
        time.sleep(0.1)
    
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') : break
    else : break
cap.release()
cv2.destroyAllWindows()  