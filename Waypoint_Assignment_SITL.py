#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time, math
from pymavlink import mavutil
from pynput.keyboard import Key, Listener
import math
from enum import Enum

connection_string = "127.0.0.1:14550"

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
print('Connectted to : %s' % connection_string)

vehicle_mode=0
turnRadius=0.001

def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.
    print("Current mission is downloaded!")

def directionAssignment(vehicle):
    x=[vehicle.location.global_relative_frame.lat + turnRadius * math.cos(vehicle.heading * math.pi / 180)]
    y=[vehicle.location.global_relative_frame.lon + turnRadius * math.sin(vehicle.heading * math.pi / 180)]
    z=[60]
    for i in range(0, 6):
        if i == 0 or i == 5:
            x.append(x[i] + turnRadius * math.cos((vehicle.heading-15) * math.pi / 180))
            y.append(y[i] + turnRadius * math.sin((vehicle.heading-15) * math.pi / 180))
            z.append(60)
        elif i == 1 or i == 4:
            x.append(x[i] + turnRadius * math.cos((vehicle.heading)* math.pi / 180))
            y.append(y[i] + turnRadius * math.sin((vehicle.heading)* math.pi / 180))
            z.append(60)
        else:
            x.append(x[i] + turnRadius * math.cos((vehicle.heading+15) * math.pi / 180))
            y.append(y[i] + turnRadius * math.sin((vehicle.heading+15) * math.pi / 180))
            z.append(60)
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


while True:
    #Takeoff and start searching
    if vehicle_mode == 0:
        download_mission()
        x = [vehicle.home_location.lat+0.01*math.cos(vehicle.heading*math.pi/180)]
        y = [vehicle.home_location.lon+0.01*math.sin(vehicle.heading*math.pi/180)]
        z = [60]
        mission(x,y,z,60)


        arm_and_takeoff(60)

        print("TAKEOFF and SEARCH")
        # Reset mission set to first (0) waypoint
        vehicle.commands.next=0

        # Set mode to AUTO to start mission
        vehicle.mode = VehicleMode("AUTO")
        vehicle_mode = 1
    #Area check 
    if vehicle_mode == 1:
	x = [-35.36331824]
        y = [149.16668315]
        z = [60]
	mission(x,y,z,60)
	vehicle_mode=2
    if vehicle_mode==2:
	if vehicle.location.global_relative_frame.lat<-35.3615:
	    vehicle_mode=3
    if vehicle_mode == 3:
	print('Gorev Basladi')
        x, y, z = directionAssignment(vehicle)
	print(x,y,z)
        mission(x, y, z,60)
	vehicle_mode=4

