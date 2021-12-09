import numpy as np
import cv2
import time
from picamera import PiCamera
from picamera.array import PiRGBArray

                     
frame_number = 1                                       # Ekranda gözükecek fotoğrafın sırası.
blackLower = (0,  0, 0)                                # Yesil Maske icin alt deger
blackUpper = (350,55,100)                               # Yesil Maske icin ust deger
kernel = np.ones((3,3))                                # Erozyon ve Genisletme icin kernel
cThr=[100,100]                                         # Canny Edge icin esik degeri
most_extensive = np.zeros((1,2)).reshape(1,2)          # En buyuk alan ve merkezinin kaydedilecegi kısım


camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out1 = cv2.VideoWriter('SiyahTespit.avi',fourcc, 5.0, (640,480))
out2 = cv2.VideoWriter('SiyahMaske.avi',fourcc,5.0,(640,480),0)



for frame_arr in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    
    frame = frame_arr.array
    
    if len(frame) > 0:
        
        frame_copy = frame.copy()                                    # Gerçek Görüntünün Etkilenmemesi için kopyası alınır.
        
        
        # HSV Format
        frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)   
        
        
        
        threshed_gray= cv2.inRange(frame_hsv, blackLower, blackUpper)         # gri alan araması yap
                    
        # Outer Area Erosion and Dilating
        
        frame_dilate_gray = cv2.dilate(threshed_gray,kernel,iterations=1)
        frame_erode_gray = cv2.erode(frame_dilate_gray,kernel,iterations=1)
                    
        # Area Blurring
        frame_blurred_gray = cv2.GaussianBlur(frame_erode_gray,ksize=(5,5),sigmaX=1)

        rawCapture.truncate(0)
        
        (frame_contour_gray,contours_gray,hierarcy_gray) = cv2.findContours(frame_blurred_gray.copy(),
                                                                            cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)   # konturları çıkartext_arr.append(hlist_c[ht-1])
        if len(contours_gray)>0:
                    
                    
                c_gray = max(contours_gray,key=cv2.contourArea)
                    
                    
                rect_gray = cv2.minAreaRect(c_gray)
                    
                ((x_gray,y_gray),(w_gray,h_gray),rotation_gray) = rect_gray
                    
                box_gray = cv2.boxPoints(rect_gray)
                box_gray = np.int64(box_gray)
                M_gray = cv2.moments(c_gray)
                center_gray = (int(M_gray['m10']/M_gray['m00']),int(M_gray['m01']/M_gray['m00']))
                
                
                cv2.drawContours(frame_copy,[box_gray], 0, (255,0,255),1)
                
                cv2.imshow('Tespit',frame_copy)
                cv2.imshow('Siyah Alan',threshed_gray)
                cv2.imshow('Siyah Alan Gurultusuz',frame_blurred_gray)
                out1.write(frame_copy)
                out2.write(frame_blurred_gray)
                
                
        frame_number = frame_number + 1

        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') : break
    else : break

cv2.destroyAllWindows()             