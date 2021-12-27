from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import numpy as np
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
from multiprocessing import Process,Pipe

center_gray_in_frame =  [0,0]

#Parameters#
# class detectionParameters:
#     frame_number = 1                                       # Ekranda gözükecek fotoğrafın sırası.
#     greenLower = (36,  50,  70)                            # Yesil Maske icin alt deger
#     greenUpper = (89, 255, 250)                            # Yesil Maske icin ust deger
#     kernel = np.ones((3,3))                                # Erozyon ve Genisletme icin kernel
#     cThr=[100,100]                                         # Canny Edge icin esik degeri
#     blackThr=50
    
    
#algorithm_modes=["Search","Drop"]                      # Algoritmanın modları
#detectionParameters_instant=detectionParameters()
#Initialization#
#vehicle = connect('/dev/ttyS0',baud=921600,wait_ready=True)             #Connecting to vehicle
#print("Connected to vehicle!")


#mode=algorithm_modes[0]
class detectionMessage:
    def __init__(self, det_suc, x, y):
        self.detection_success = det_suc
        self.x = x
        self.y = y
    
class pixels:
    def __init__(self, x, y):
        self.x = x
        self.y = y


#Functions#
def imageProcessingRealTime(child_conn,detectionParameters):
    time.sleep(1)
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    print("frame is waiting")
    for frame_arr in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame = frame_arr.array
        
        if len(frame) > 0:
            frame_copy = frame.copy()                                    # Gerçek Görüntünün Etkilenmemesi için kopyası alınır.
            
            # HSV Format
            frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)   
            
            # Black-White Format
            frame_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) 
            
            # Dış Bölge Yeşil Maskeleme
        
            frame_green_area = cv2.inRange(frame_hsv,detectionParameters.greenLower,detectionParameters.greenUpper)        
            
            # Outer Area Erosion and Dilating
            
            frame_dilate = cv2.dilate(frame_green_area,detectionParameters.kernel,iterations=1)
            frame_erode = cv2.erode(frame_dilate,detectionParameters.kernel,iterations=2)
            
            # Area Blurring
            
            frame_blurred = cv2.GaussianBlur(frame_erode,ksize=(3,3),sigmaX=1)
            
            # Canny Edge Detection
            
            img_canny = cv2.Canny(frame_blurred,detectionParameters.cThr[0],detectionParameters.cThr[1])
            
            rawCapture.truncate(0)
            
            # Dış Bölge işaretleme
            (frame_contour,contours,hierarcy) = cv2.findContours(frame_blurred.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours)>0:
                
                c = max(contours,key=cv2.contourArea)      # Maksimum alana sahip kontur alani
                
                rect = cv2.minAreaRect(c)                  # Alana çizilebilecek minimum boyuttaki diktortgen
                
                ((x,y),(w,h),rotation) = rect
                
                box = cv2.boxPoints(rect)
                box = np.int64(box)
                
                M = cv2.moments(c)
                
                try:center = (int(M['m10']/M['m00']),int(M['m01']/M['m00']))
                except:pass
                
                up,down,left,right = min(box[:,0]),max(box[:,0]),min(box[:,1]),max(box[:,1])
                
                target_area = frame_gray[left:right,up:down]           # tespit edilen alnın daha içi olması için 
                

                ret_gray, threshed_gray= cv2.threshold(target_area, detectionParameters.blackThr, 255, cv2.THRESH_BINARY_INV)         # gri alan araması yap
                
                if len(target_area)>0:
                    try:
                        frame_dilate_gray = cv2.dilate(threshed_gray,detectionParameters.kernel,iterations=1)
                        frame_erode_gray = cv2.erode(frame_dilate_gray,detectionParameters.kernel,iterations=1)
                            
                        # Area Blurring
                        frame_blurred_gray = cv2.GaussianBlur(frame_erode_gray,ksize=(5,5),sigmaX=1)

                                
                        (frame_contour_gray,contours_gray,hierarcy_gray) = cv2.findContours(frame_blurred_gray.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)   # konturları çıkart

                        
                        if len(contours_gray)>0:
                            c_gray = max(contours_gray,key=cv2.contourArea)
                            rect_gray = cv2.minAreaRect(c_gray)
                            ((x_gray,y_gray),(w_gray,h_gray),rotation_gray) = rect_gray
                            box_gray = cv2.boxPoints(rect_gray)
                            box_gray = np.int64(box_gray)
                            cv2.drawContours(target_area,[box_gray], 0, (255,0,255),1)
                            
                            M_gray = cv2.moments(c_gray)
                            
                            try:center_gray = (int(M_gray['m10']/M_gray['m00']),int(M_gray['m01']/M_gray['m00']))
                            except:pass
                            
                            global center_gray_in_frame
                            center_gray_in_frame = (center_gray[0]+up,center_gray[1]+left)
                            
                            detectionMessage2Send = detectionMessage(True, center_gray[0]+up, center_gray[1]+left)
                            
                            
                            area_inner = cv2.contourArea(c_gray)
                            area_outer = cv2.contourArea(c)
                        else:
                            detectionMessage2Send = detectionMessage(False, 0, 0)
                            
                    except:
                        detectionMessage2Send = detectionMessage(False, 0, 0)
                        
                else:
                    detectionMessage2Send = detectionMessage(False, 0, 0)
                    

                                            
                try:
                    cv2.drawContours(frame_copy,[box], 0, (0,255,255),1)
                except:pass
                
                try:
                    rect_gray_in_image = ((up+center_gray[0],left+[center_gray[1]]),(w_gray,h_gray),rotation_gray)
                    box_gray_in_image = cv2.boxPoints(rect_gray_in_image)
                    box_gray_in_image_int = np.int64(box_gray_in_image)
                    cv2.drawContours(frame_copy,[box_gray_in_image_int], 0, (255,0,255),1)
                    cv2.circle(frame_copy,(center_gray_in_frame[0],center_gray_in_frame[1]),1,(255,255,255),2)                   
                except:pass    
                        
                
                try:
                    cv2.imshow('Tespit',frame_copy)
                    cv2.imshow('Yesil Alan',frame_green_area)
                    cv2.imshow('Kenarliklar',frame_blurred)
                except:pass
                
                try:
                    cv2.imshow('Tespit Siyah',cv2.resize(target_area,(200,200)))
                    cv2.imshow('th gray',cv2.resize(threshed_gray,(200,200)))
                except:pass
                
             
            else:
                detectionMessage2Send = detectionMessage(False, 0, 0)
            try:
                cv2.imshow('Tespit',frame_copy)
            except:pass
        
            detectionParameters.frame_number = detectionParameters.frame_number + 1
            
            
                
            

            
        
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') : break
        else : break
        child_conn.send(detectionMessage2Send)
    cv2.destroyAllWindows()
    
    
#Infinite Loop#
#while True:
#    imageProcessingRealTime(camera,rawCapture,detectionParameters);
#    print(center_gray_in_frame)
    