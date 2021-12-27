#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time, math
from pymavlink import mavutil
import array as arr
#from pynput.keyboard import Key, Listener

def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.
    print("Current mission is downloaded!")
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)
def midPointCoordinates(gps, pixels):
    fov = 63
    fov_triangle_angle = 53.13
    width=gps.alt*math.tan((fov/2)*math.pi/180)*math.cos(fov_triangle_angle*math.pi/180)*(2)   # yarim diagonal hesabı * 2
    height=gps.alt*math.tan((fov/2)*math.pi/180)*math.sin(fov_triangle_angle*math.pi/180)*(2)
    print(width)
    print(pixels.x)
    print(pixels.y)
    print(((320 - pixels.lon)*height)/320 )
    print(((240 - pixels.len)*width)/240 ) 
    k = 36.76804728839592
    x = math.tan((240 - pixels.len)*(fov*(math.pi/180)/2)/240)*gps.alt
    y = math.tan((320 - pixels.lon)*(fov*(math.pi/180)/2)/320)*gps.alt
    return x/k*width , y/k*height
def directionAssignment(vehicle):
    search_angle=15
    search_alt=60
    turnRadius=0.0006
    x=[vehicle.location.global_relative_frame.lat + turnRadius * math.cos(vehicle.heading * math.pi / 180)]
    y=[vehicle.location.global_relative_frame.lon + turnRadius * math.sin(vehicle.heading * math.pi / 180)]
    z=[search_alt]
    for i in range(0, 6):
        if i == 0 or i == 5:
            x.append(x[i] + turnRadius * math.cos((vehicle.heading-search_angle) * math.pi / 180))
            y.append(y[i] + turnRadius * math.sin((vehicle.heading-search_angle) * math.pi / 180))
            z.append(search_alt)
        elif i == 1 or i == 4:
            x.append(x[i] + turnRadius * math.cos((vehicle.heading)* math.pi / 180))
            y.append(y[i] + turnRadius * math.sin((vehicle.heading)* math.pi / 180))
            z.append(search_alt)
        else:
            x.append(x[i] + turnRadius * math.cos((vehicle.heading+search_angle) * math.pi / 180))
            y.append(y[i] + turnRadius * math.sin((vehicle.heading+search_angle) * math.pi / 180))
            z.append(search_alt)
    return x,y,z

def mission(x,y,z,initial_alt):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).
    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """ 

    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, initial_alt))

    for new_point in range(len(x)):
        
        point=LocationGlobal(x[new_point], y[new_point], z[new_point])
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point.lat, point.lon, point.alt))

    #Dummy ending point
    point=LocationGlobal(x[-1], y[-1], z[-1])
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point.lat, point.lon, point.alt))    

    print("Waypoints are created!")
    print(" Upload new commands to vehicle")
    cmds.upload()

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    while not vehicle.armed:      
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)
    print("Vehicle is armed!")

    
    vehicle.mode = VehicleMode("AUTO")
    print("Vehicle mode is changed to: AUTO")

    
    print("Taking off!")

    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(0.1)

#

class gps:
    alt = 60
    len = 0
    lon = 0










# while True:
#     #Takeoff and start searching
#     if vehicle_mode == 0:
#         download_mission()
#         x = [vehicle.home_location.lat+0.01*math.cos(vehicle.heading*math.pi/180)]
#         y = [vehicle.home_location.lon+0.01*math.sin(vehicle.heading*math.pi/180)]
#         z = [60]
#         mission(x,y,z,60)
# 
# 
#         arm_and_takeoff(60)
# 
#         print("TAKEOFF and SEARCH")
#         # Reset mission set to first (0) waypoint
#         vehicle.commands.next=0
# 
#         # Set mode to AUTO to start mission
#         vehicle.mode = VehicleMode("AUTO")
#         vehicle_mode = 1
#     #Area check 
#     if vehicle_mode == 1:
#     x = [-35.36331824]
#         y = [149.16668315]
#         z = [60]
#     mission(x,y,z,60)
#     vehicle_mode=2
#     if vehicle_mode==2:
#     if vehicle.location.global_relative_frame.lat<-35.3615:
#         vehicle_mode=3
#     if vehicle_mode == 3:
#     print('Gorev Basladi')
#         x, y, z = directionAssignment(vehicle)
#     print(x,y,z)
#         mission(x, y, z,60)
#     vehicle_mode=4
