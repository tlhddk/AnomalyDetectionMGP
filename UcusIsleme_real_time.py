import numpy as np
import cv2
import time
from picamera import PiCamera
from picamera.array import PiRGBArray

                     
frame_number = 1                                       # Ekranda gözükecek fotoğrafın sırası.
greenLower = (36,  50,  70)                            # Yesil Maske icin alt deger
greenUpper = (89, 250, 250)                            # Yesil Maske icin ust deger
kernel = np.ones((3,3))                                # Erozyon ve Genisletme icin kernel
cThr=[100,100]                                         # Canny Edge icin esik degeri
most_extensive = np.zeros((1,2)).reshape(1,2)          # En buyuk alan ve merkezinin kaydedilecegi kısım


camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))



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
            
            up,down,left,right = min(box[:,0]),max(box[:,0]),min(box[:,1]),max(box[:,1])
            
            target_area = frame_gray[left:right,up:down]           # tespit edilen alnın daha içi olması için 
            
              
            for ht in range (1,len(contours)):
                
                
                try:
                    ret_gray, threshed_gray= cv2.threshold(target_area, 46, 255, cv2.THRESH_BINARY_INV)         # gri alan araması yap
                    # Outer Area Erosion and Dilating
        
                    frame_dilate_gray = cv2.dilate(threshed_gray,kernel,iterations=1)
                    frame_erode_gray = cv2.erode(frame_dilate_gray,kernel,iterations=1)
                    
                    # Area Blurring
                    frame_blurred_gray = cv2.GaussianBlur(frame_erode_gray,ksize=(5,5),sigmaX=1)

                    
                    (frame_contour_gray,contours_gray,hierarcy_gray) = cv2.findContours(frame_blurred_gray.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)   # konturları çıkart
                    ext_arr.append(hlist_c[ht-1]) 
                    
                    
                except:

                    
                    c = contours[np.where(hlist_array==hlist_c[ht])[0][0]]                                      #  sonraki en büyük alanın seçilmesi sağlanır
                    
                    
                    rect = cv2.minAreaRect(c)
                    
                    ((x,y),(w,h),rotation) = rect
                    
                    box = cv2.boxPoints(rect)
                    box = np.int64(box)
                    M = cv2.moments(c)
                
                    up,down,left,right = min(box[:,0]),max(box[:,0]),min(box[:,1]),max(box[:,1])
                    
                    target_area = frame_gray[left:right,up:down]                                     # Tespit edilen alanın daha içi olması için
                    
                    try:center = (int(M['m10']/M['m00']),int(M['m01']/M['m00']))
                    except: pass
                    
                    
                    continue
                    
                break
            
            
            try:    
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
                        cv2.drawContours(target_area,[box_gray], 0, (255,0,255),1)
                        area_inner = cv2.contourArea(c_gray)
                        area_outer = cv2.contourArea(c)
                        ext_arr.append(center_gray_in_frame)      # O anki grinin merkezini kaydet
                    except:pass
                    try:
                        if len(most_extensive) == 6:
                            most_extensive = np.delete(most_extensive,[1,2,3,4,5],0)                       # Her 5 döngüde bir temizle
                            
                        most_extensive = np.append(most_extensive,np.array(ext_arr).reshape(1,2),axis=0)   # Alan büyüklüğü ve merkez noktası eklenmesi
                        most_extensive = most_extensive[most_extensive[:,0].argsort()]                     # Verileri alan büyüklüğüne göre sirala
                        top_merkez_x = 0
                        top_merkez_y = 0
                        for i in most_extensive[1:,1]:                                                     # Ortalama merkez değerini döndür
                            top_merkez_y = top_merkez_y + i[1]
                            top_merkez_x = top_merkez_x + i[0]
                        ort_merkez_y = top_merkez_y/(len(most_extensive)-1)                                                   
                        ort_merkez_x = top_merkez_x/(len(most_extensive)-1)
                        center_ort = (ort_merkez_x,ort_merkez_y)
                    
                    
                        
                    except:pass
                    
                    
                    
                    try:
                        cv2.drawContours(frame_copy,[box], 0, (0,255,255),1)
                        rect_gray_in_image = ((up+center_gray[0],left+[center_gray[1]]),(w_gray,h_gray),rotation_gray)
                        box_gray_in_image = cv2.boxPoints(rect_gray_in_image)
                        box_gray_in_image_int = np.int64(box_gray_in_image)
                        cv2.drawContours(frame_copy,[box_gray_in_image_int], 0, (255,0,255),1)
                        cv2.putText(frame_copy, 'Tespit Edilen En Buyuk Alan : {} '.format(most_extensive[-1,0]),(10,40),cv2.FONT_HERSHEY_TRIPLEX,0.5,(255,255,225),1)
                        cv2.putText(frame_copy, 'Merkezi : {}'.format(most_extensive[-1,1]),(10,60),cv2.FONT_HERSHEY_TRIPLEX,0.5,(255,255,225),1)
                        cv2.circle(frame_copy,(center_gray_in_frame[0],center_gray_in_frame[1]),1,(255,255,255),2)                   
                        cv2.circle(frame_copy, (int(center_ort[0]),int(center_ort[1])),1,(0,0,255),2)
                    except:pass    
                    
            except:pass
            
            try:
                cv2.imshow('Tespit',frame_copy)
                cv2.imshow('Yesil Alan',frame_green_area)
                cv2.imshow('Kenarliklar',frame_blurred)
                cv2.imshow('th gray',threshed_gray)
                cv2.imshow('Tespit Siyah',target_area)
                time.sleep(0.2)
            
            except:pass
        
        frame_number = frame_number + 1

        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') : break
    else : break

cv2.destroyAllWindows()  