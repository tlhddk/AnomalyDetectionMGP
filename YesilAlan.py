import numpy as np
import cv2
import time
from picamera import PiCamera
from picamera.array import PiRGBArray

frame_number = 1                                       # Ekranda gözükecek fotoğrafın sırası.
greenLower = (36,  50,  70)                            # Yesil Maske icin alt deger
greenUpper = (89, 250, 250)                            # Yesil Maske icin ust deger
kernel = np.ones((3,3))                                # Erozyon ve Genisletme icin kernel
cThr=[100,100]

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out1 = cv2.VideoWriter('YesilTespit.avi',fourcc,5.0, (640,480))
out2 = cv2.VideoWriter('YesilMaske.avi',fourcc,5.0,(640,480),0)

for frame_arr in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    
    frame = frame_arr.array
    
    if len(frame) > 0:
        frame_copy = frame.copy()                                    # Gerçek Görüntünün Etkilenmemesi için kopyası alınır.
        
        # Resizing 
        frame_resize = cv2.resize(frame,(int(frame.shape[1]*1000/frame.shape[0]),720)) 
        
        # HSV Format
        frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)   
        
        # Black-White Format
        frame_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) 
        
        
        # Dış Bölge Yeşil Maskeleme
    
        frame_green_area = cv2.inRange(frame_hsv,greenLower,greenUpper)        
        
        
        # Outer Area Erosion and Dilating
        
        frame_dilate = cv2.dilate(frame_green_area,kernel,iterations=1)
        frame_erode = cv2.erode(frame_dilate,kernel,iterations=2)
        
        # Area Blurring
        frame_blurred = cv2.GaussianBlur(frame_erode,ksize=(3,3),sigmaX=1)
        
        # Canny Edge Detection
        
        img_canny = cv2.Canny(frame_blurred,cThr[0],cThr[1])
        
        rawCapture.truncate(0)
        
        # Dış Bölge işaretleme
        (frame_contour,contours,hierarcy) = cv2.findContours(frame_blurred.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        
        if len(contours)>0:
            
            hlist = []                                 # Kontur buyukluklerinin eklendigi liste
            ext_arr = []

            
            c = max(contours,key=cv2.contourArea)      # Maksimum alana sahip kontur alani
            for i in range(len(contours)):
                hlist.append(contours[i].shape[0])     
            hlist_c = hlist.copy()                     
            hlist_c.sort(reverse=True)
            hlist_array = np.array(hlist)              # Siralanmis liste
            
            rect = cv2.minAreaRect(c)                  # Alana çizilebilecek minimum boyuttaki diktortgen
            
            ((x,y),(w,h),rotation) = rect
            
            box = cv2.boxPoints(rect)
            box = np.int64(box)
            
            M = cv2.moments(c)
            
            try:center = (int(M['m10']/M['m00']),int(M['m01']/M['m00']))
            except:pass
        
        try:
            cv2.drawContours(frame_copy,[box], 0, (0,255,255),1)
            cv2.putText(frame_copy,'Frame Sayisi: {}'.format(frame_number),
                        (10,40),cv2.FONT_HERSHEY_TRIPLEX,0.5,(255,255,225),1)
            cv2.imshow('Tespit',frame_copy)
            cv2.imshow('Yesil Alan',frame_green_area)
            cv2.imshow('Yesil Alan Gurultusuz',frame_blurred)
            out1.write(frame_copy)
            out2.write(frame_erode)
        except: pass
        
                             
        
        frame_number = frame_number + 1

        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') : break
    else : break

cv2.destroyAllWindows() 