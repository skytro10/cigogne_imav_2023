
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2022

@author: Thomas Pavot
"""
import os
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time
import math

from threading import Thread
import threading

from math import atan2, cos, radians, sin, sqrt, pi
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil 
from array import array
from datetime import datetime
from picamera import PiCamera,Color
from picamera.array import PiRGBArray


from detection_target import Detection
from commande_drone import Drone


###################### Dronekit ####################


#--------------------- Connection ----------------------------


monDrone = Drone()


missionList = monDrone.download_mission()

print("nbr WP : "+ str(monDrone.vehicle.commands.count))

monDrone.vehicle.commands.next = 0

print ("Current Waypoint: %s" % monDrone.vehicle.commands.next)

monDrone.vehicle.commands.next = 5

print ("Current Waypoint: %s" % monDrone.vehicle.commands.next)

nextwaypoint = monDrone.vehicle.commands.next

missionitem = monDrone.vehicle.commands[nextwaypoint-1] #commands are zero indexed

lat=missionitem.x

print("lat : "+str(lat))









print("Fin du test")



"""
while True :
  altitudeAuSol = monDrone.vehicle.rangefinder.distance
  longitude = monDrone.vehicle.location.global_relative_frame.lon
  latitude = monDrone.vehicle.location.global_relative_frame.lat
  
  x_centerPixel_target, y_centerPixel_target, marker_found, whiteSquare_found = detection_object.Detection_aruco(latitude,longitude,altitudeAuSol,False)
  print( "alt : "+ str(altitudeAuSol))
  measured_distance = detection_object.get_distance_image(x_centerPixel_target, y_centerPixel_target, altitudeAuSol)
  print("distance :" + str(measured_distance))




print("Fin du test")"""

