from types import MappingProxyType
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
import Distance

# Görüntü içe aktarım

path = './Ucus6m.mp4'
cap = cv2.VideoCapture(path)
frame_number = 1




target1 = Distance.Target()
camera2  = Distance.Camera(76,32)
print(camera2.CamFov)
mgp     = Distance.Plane()
mgp.setAltitude(60)
target1.MeasureDistance(1,camera2,mgp.getAltitude())

while True:
    ret, frame = cap.read()
    if ret:
        frame_copy = frame.copy() 
        frame_resize = cv2.resize(frame,(int(frame.shape[1]*1000/frame.shape[0]),720)) # Yeniden Boyutlandırma
        frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)   # HSV Format
        frame_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Siyah-Beyaz Format
        
        # Bölge Blur
        # frame_blurred = cv2.GaussianBlur(frame_hsv,ksize=(7,7),sigmaX=1)
        
        # Dış Bölge Yeşil Maskeleme
        
        greenLower=(30,100,0)
        greenUpper=(70,255,255)
        frame_green_area = cv2.inRange(frame_hsv,greenLower,greenUpper)
        
        # Canny Kenar bulma
        cThr=[100,100]
        img_canny = cv2.Canny(frame_green_area,cThr[0],cThr[1])
        kernel = np.ones((3,3))
        
        
        # Dış Bölge Erozyon ve Genişletme
        
        frame_dilate = cv2.dilate(img_canny,kernel,iterations=1)
        frame_erode = cv2.erode(frame_dilate,kernel,iterations=3)
        
        

        # Dış Bölge işaretleme
        (frame_contour,contours,hierarcy) = cv2.findContours(frame_dilate.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours)>0:

            c = max(contours,key=cv2.contourArea)
            
            rect = cv2.minAreaRect(c)
            
            ((x,y),(w,h),rotation) = rect
            
            box = cv2.boxPoints(rect)
            box = np.int64(box)
            
            M = cv2.moments(c)
            center = (int(M['m10']/M['m00']),int(M['m01']/M['m00']))
            
            cv2.drawContours(frame_copy,[box], 0, (0,255,255),1)
            
            up,down,left,right = min(box[:,0]),max(box[:,0]),min(box[:,1]),max(box[:,1])
            
            target_area = frame_gray[left:right,up:down]
            
            ret_gray, threshed_gray= cv2.threshold(target_area, 54, 255, cv2.THRESH_BINARY_INV)
            
            (frame_contour_gray,contours_gray,hierarcy_gray) = cv2.findContours(threshed_gray.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            
                
            if len(contours_gray)>0:
                
                
                c_gray = max(contours_gray,key=cv2.contourArea)
                
                
                rect_gray = cv2.minAreaRect(c_gray)
                
                ((x_gray,y_gray),(w_gray,h_gray),rotation_gray) = rect_gray
                
                box_gray = cv2.boxPoints(rect_gray)
                box_gray = np.int64(box_gray)
                
                M_gray = cv2.moments(c_gray)
                try:
                    center_gray = (int(M_gray['m10']/M_gray['m00']),int(M_gray['m01']/M_gray['m00']))
                    center_gray_in_frame = (center_gray[0]+up,center_gray[1]+left)
                except:
                    pass
                
                cv2.drawContours(target_area,[box_gray], 0, (255,0,255),1)
                area_inner = cv2.contourArea(c_gray)
                area_outer = cv2.contourArea(c)
                
                
                
                try:
                
                    rect_gray_in_image = ((up+center_gray[0],left+[center_gray[1]]),(w_gray,h_gray),rotation_gray)
                    box_gray_in_image = cv2.boxPoints(rect_gray_in_image)
                    box_gray_in_image_int = np.int64(box_gray_in_image)
                    cv2.drawContours(frame_copy,[box_gray_in_image_int], 0, (255,0,255),1)
                    cv2.putText(frame_copy, ' | Dis Alan: {}  | Ic Alan: {} | {}'.format(area_outer,area_inner,frame_number),(10,40),cv2.FONT_HERSHEY_TRIPLEX,0.5,(255,255,225),1)
                    cv2.circle(frame_copy,center_gray_in_frame,1,(255,255,255),2)
                except:
                    pass
            
                        
            
            
            

            cv2.imshow('Tespit',frame_copy)
            cv2.imshow('Tespit Yesil',frame_dilate)
            cv2.imshow('Tespit Siyah',cv2.resize(target_area,(600,800)))
            time.sleep(0.1)
        
        frame_number = frame_number + 1
        
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') : break
    else : break
cap.release()
cv2.destroyAllWindows()       
        