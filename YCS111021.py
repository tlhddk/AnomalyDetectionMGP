import numpy as np
import cv2
import matplotlib.pyplot as plt
import time

cThr=[100,100]
greenLower=(30,100,0)
greenUpper=(70,255,255)
minArea=10

path = "C:\\Users\\subas\\Desktop\\WORKPLACE\\ANOMALY_DEDECTION\\Ucus6m.mp4"
cap = cv2.VideoCapture(path)
frame_number = 1

def reorder(myPoints): #corner coordinates
    #print(myPoints.shape)
    myPointsNew = np.zeros_like(myPoints)
    myPoints = myPoints.reshape((4,2))
    add = myPoints.sum(1)
    myPointsNew[0] = myPoints[np.argmin(add)]
    myPointsNew[3] = myPoints[np.argmax(add)]
    diff = np.diff(myPoints,axis=1)
    myPointsNew[1]= myPoints[np.argmin(diff)]
    myPointsNew[2] = myPoints[np.argmax(diff)]
    return myPointsNew

while True:
    ret, frame = cap.read()
    if ret:
        frame_copy = frame.copy() 
     
        imgGray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)       
        frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        frame_green_area = cv2.inRange(frame_hsv,greenLower,greenUpper)
        
        imgCanny = cv2.Canny( frame_green_area,cThr[0],cThr[1])
        kernel = np.ones((5,5))
        imgDial = cv2.dilate(frame_green_area,kernel,iterations=3)
        imgThre = cv2.erode(imgDial,kernel,iterations=3)

        (contours2,hiearchy2) = cv2.findContours(imgThre,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        finalCountours = []
        
        for i in contours2:
            area = cv2.contourArea(i)
            if area > minArea:
                peri = cv2.arcLength(i,True)
                approx = cv2.approxPolyDP(i,0.02*peri,True)
                bbox = cv2.boundingRect(approx)
                finalCountours.append([len(approx),area,approx,bbox,i])
        
           
        finalCountours = sorted(finalCountours,key = lambda x:x[1] ,reverse= True)
            
        # for con in finalCountours:
        #     cv2.drawContours(frame_copy,con[4],-1,(0,0,255),3)    
          
        try:
             Contours2_Area= max(contours2,key=cv2.contourArea)         
             rect = cv2.minAreaRect(Contours2_Area)  
             ((x,y),(w,h),rotation) = rect
             box = cv2.boxPoints(rect)
             box = np.int64(box)
             cv2.drawContours(frame_copy,[box], 0, (255,0,255),1)
        except:
            pass
        try:
            for obj in contours2:
                    print(obj)
                    #cv2.polylines(frame_copy,[obj[2]],True,(0,255,0),2)
                    points = reorder(obj[2])
                    print(points)
            #else:print('boş küme')
        except:
            pass

        cv2.imshow('Green Area',frame_green_area)
        cv2.imshow('Canny',imgThre)
        cv2.imshow('Tespit',frame_copy) 
        time.sleep(0.1)
        frame_number = frame_number + 1
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') : break
    else : break
cap.release()
cv2.destroyAllWindows() 