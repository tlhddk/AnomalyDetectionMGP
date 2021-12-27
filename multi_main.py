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
    connection_string = '/dev/ttyS0'

    # Connect to the Vehicle
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string,baud=921600, wait_ready=True)
    print('Connectted to : %s' % connection_string)
    print(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt )
    
    detection_counter = 0
    aim_is_not_defined = True
    x_t = 0        # total x distance
    y_t = 0        # total y distance
    x_d = 0        # current x distance
    y_d = 0        # current y distance
    min_detection_for_trigger = 3
    max_detection_for_trigger = 8
    
    while True:
        ReceivedMessage = parent_conn.recv()
        print(ReceivedMessage.detection_success, ReceivedMessage.x, ReceivedMessage.y)
        
        # ard arda 3 ila 8 tespit olması durumunda hedef gps konumu belirleyen, diger durumlarda aramaya devam eden yapı
        if aim_is_not_defined: 
            if ReceivedMessage.detection_success and detection_counter <= max_detection_for_trigger:
                
                print("center_detected ", detection_counter)
                detected_pixels = pixels(ReceivedMessage.x, ReceivedMessage.y)
                x_d,y_d = midPointCoordinates(vehicle.location.global_relative_frame, detected_pixels)
                x_t += x_d
                y_t += y_d
                detection_counter += 1
                
            elif (ReceivedMessage.detection_success and detection_counter >= max_detection_for_trigger) or \
                 (detection_counter >= min_detection_for_trigger and ReceivedMessage.detection_success == False):
                
                print("aim is defined ", detection_counter)
                print(x_t/detection_counter, y_t/detection_counter)
                aim_point = get_location_metres(vehicle.location.global_relative_frame, x_t/detection_counter, y_t/detection_counter)
                detection_counter = 0
                aim_is_not_defined = False
                
            elif ReceivedMessage.detection_success == False and detection_counter <= min_detection_for_trigger:
                detection_counter = 0
                x_d = 0
                y_d = 0
                x_t = 0
                y_t = 0
        

            
        
