from multiprocessing import Process,Queue,Pipe
from master_piece_v1 import imageProcessingRealTime
from mission_file import midPointCoordinates
import numpy as np
import mission_file
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time, math
from pymavlink import mavutil

class detectionParameters:
    frame_number = 1                                       # Ekranda gözükecek fotoğrafın sırası.
    greenLower = (36,  50,  70)                            # Yesil Maske icin alt deger
    greenUpper = (89, 255, 250)                            # Yesil Maske icin ust deger
    kernel = np.ones((3,3))                                # Erozyon ve Genisletme icin kernel
    cThr=[100,100]                                         # Canny Edge icin esik degeri
    blackThr=50
    
    
    
    
if __name__ == '__main__':
    parent_conn,child_conn = Pipe()
    p = Process(target=imageProcessingRealTime, args=(child_conn,detectionParameters,))
    p.start()
    detection_counter = 0
    print("calisiyor")
    x_t = 0
    y_t = 0
    aim_is_not_defined = True
    total_detection = 0
    total_time = 0
    while True:
        start_time = time.time()
        min_detection_for_trigger = 3
        max_detection_for_trigger = 8
        ReceivedMessage = parent_conn.recv()

        print(ReceivedMessage.detection_success, ReceivedMessage.x, ReceivedMessage.y)
        
        if ReceivedMessage.detection_success and detection_counter <= max_detection_for_trigger and aim_is_not_defined:
            print("center_detected ", detection_counter)
            try:
                x_d, y_d = parent_conn.recv().x, parent_conn.recv().y
                detection_counter += 1
                x_t += x_d
                y_t += y_d
            except:pass
        
        elif detection_counter >= min_detection_for_trigger and ReceivedMessage.detection_success == False:
            print("aim is defined ", detection_counter)
            aim_is_not_defined = False
            print(x_t/detection_counter, y_t/detection_counter)
            detection_counter = 0
            # Ortalama hesapla
            
        elif ReceivedMessage.detection_success and detection_counter >= max_detection_for_trigger:
            print("aim is defined ", detection_counter)
            print(x_t/detection_counter, y_t/detection_counter)
            detection_counter = 0
            aim_is_not_defined = False
        
        elif ReceivedMessage.detection_success == False and detection_counter <= min_detection_for_trigger:
            detection_counter = 0
            x_t = 0
            y_t = 0
            
            
            
            
            
            
        if time.time() - start_time < 0.3:
            total_time += time.time() - start_time
            total_detection += 1
            avg_time = total_time/total_detection
            print(avg_time)

            
        