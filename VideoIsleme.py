# -*- coding: utf-8 -*-
"""
Created on Mon Sep 27 08:38:20 2021

@author: mteks
"""


import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time

#Görüntü içe aktarımı
path = './video_ucus_3.mp4'
cap = cv2.VideoCapture(path)
frame_number = 1
while True:
    ret, frame = cap.read()
    if ret:
        frame_h = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        
        # Dış Bölge Gaussian Blur
        img_g = cv2.GaussianBlur(frame_h,ksize=(7,7),sigmaX=1)
        
        # Dış Bölge Yeşil Maskeleme
        
        greenLower=(30,100,0)
        greenUpper=(70,255,255)
        frame_m = cv2.inRange(frame_h,greenLower,greenUpper)
        
        
        
        # Dış Bölge Erozyon ve Genişletme
         
        frame_masked = cv2.erode(frame_m,None,iterations=1)
        frame_masked = cv2.dilate(frame_masked,None,iterations=2)
        
        
        
        # Dış Bölge işaretleme
        (frame_con,contours,hierarcy) = cv2.findContours(frame_masked.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        if len(contours)>0:
            
            c = max(contours,key=cv2.contourArea)
            
            
            rect = cv2.minAreaRect(c)
            
            ((x,y),(w,h),rotation) = rect
            
            box = cv2.boxPoints(rect)
            box = np.int64(box)
            
            M = cv2.moments(c)
            center = (int(M['m10']/M['m00']),int(M['m01']/M['m00']))
            
            cv2.drawContours(frame, [box], 0, (0,255,255),2)
        
        #frame_hs = frame[box[1,0]:box[3,0],box[1,1],]
        
        # İç Bölge Siyah Maske
        
        blackLower=(50,0,0)
        blackUpper=(100,60,60)
        frame_m_s = cv2.inRange(frame_h,blackLower,blackUpper)
        
        # İç bölge erozyon ve genişletme
        
        #frame_s_masked = cv2.erode(frame_m_s,None,iterations=1)
        frame_s_masked = cv2.dilate(frame_m_s,None,iterations=3)
        
        
        (frame_con,s_contours,s_hierarcy) = cv2.findContours(frame_s_masked.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        if len(s_contours)>0:
            
            c_s = max(s_contours,key=cv2.contourArea)
            
            rect_s = cv2.minAreaRect(c_s)
            
            ((x_s,y_s),(w_s,h_s),rotation_s) = rect_s
            
            box_s = cv2.boxPoints(rect_s)
            box_s = np.int64(box_s)
            
            M_s = cv2.moments(c_s)
            center_s = (int(M_s['m10']/M_s['m00']),int(M_s['m01']/M_s['m00']))
            
            cv2.drawContours(frame, [box_s], 0, (255,0,0),1)
            
            area_inner = cv2.contourArea(c_s)
            area_outer = cv2.contourArea(c)-area_inner
       
        cv2.putText(frame, ' | Dis Alan: {}  | Ic Alan: {} | {}'.format(area_outer,area_inner,frame_number),(10,40),cv2.FONT_HERSHEY_TRIPLEX,0.5,(255,255,225),1)
        cv2.circle(frame,center_s,1,(100,100,0),2)
        cv2.imshow('Tespit',frame)
        
        cv2.imshow('Tespit_G',frame_masked)
        cv2.imshow('Tespit_S',frame_s_masked)
        time.sleep(0.2)
        
        frame_number = frame_number + 1
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') : break
    else : break
cap.release()
cv2.destroyAllWindows()       
        
                
        