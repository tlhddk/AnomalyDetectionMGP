from types import MappingProxyType
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
import Distance

#%%
# Görüntü içe aktarım

path = './Ucus6m.mp4'
cap = cv2.VideoCapture(path)
frame_number = 1

target1 = Distance.target
print(target1.upperEdge)
camera = Distance.Camera(76,32)
mgp = Distance.Plane

 


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
        
        # Dış Bölge Erozyon ve Genişletme
        
        frame_erode = cv2.erode(frame_green_area,None,iterations=2)
        frame_dilate = cv2.dilate(frame_erode,None,iterations=2)
        

        # Dış Bölge işaretleme
        (contours,hierarcy) = cv2.findContours(frame_dilate.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
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
            
            (contours_gray,hierarcy_gray) = cv2.findContours(threshed_gray.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            
                
            if len(contours_gray)>0:
                
                
                c_gray = max(contours_gray,key=cv2.contourArea)
                
                
                rect_gray = cv2.minAreaRect(c_gray)
                
                ((x_gray,y_gray),(w_gray,h_gray),rotation_gray) = rect_gray
                
                box_gray = cv2.boxPoints(rect_gray)
                box_gray = np.int64(box_gray)
                
                M_gray = cv2.moments(c_gray)
                try:
                    center_gray = (int(M_gray['m10']/M_gray['m00']),int(M_gray['m01']/M_gray['m00']))
                except:
                    continue
                cv2.drawContours(target_area,[box_gray], 0, (255,0,255),1)
                
                
                rect_gray_in_image = ((float(up+center_gray[0]),float(left+center_gray[1])),(w_gray,h_gray),rotation_gray)
                try:
                    box_gray_in_image = cv2.boxPoints(rect_gray_in_image)
                    box_gray_in_image_int = np.int64(box_gray_in_image)
                    cv2.drawContours(frame_copy,[box_gray_in_image_int], 0, (255,0,255),1)
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
        
# %%
