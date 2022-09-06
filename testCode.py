
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

print("Connecting...")
chemin_drone = '/dev/ttyACM0'
global vehicle
vehicle = connect(chemin_drone, wait_ready=True, baud=57600, heartbeat_timeout=2)

print("Connection OK")


while True :
  altitudeAuSol = vehicle.rangefinder.distance
  longitude = vehicle.location.global_relative_frame.lon
  latitude = vehicle.location.global_relative_frame.lat
  
  detection_object = Detection()
  x_imageCenter, y_imageCenter, x_centerPixel_target, y_centerPixel_target, marker_found, whiteSquare_found = detection_object.Detection_aruco(latitude,longitude,altitudeAuSol,False)
  print( "alt : "+ str(altitudeAuSol))
  measured_distance = detection_object.get_distance_image(x_imageCenter, y_imageCenter, x_centerPixel_target, y_centerPixel_target)
  print("distance :" + str(measured_distance))


"""print( "lat : "+ str(latitude))
print( "long : "+ str(longitude))

area = 70000*altitude**-2

print ( "area : "+str(area))


currentLocation = vehicle.location.global_relative_frame
print(currentLocation)

locationMaisonStrasbourgThomas = LocationGlobalRelative(48.573451, 7.770484, 10)
locationMaisonStrasbourgTerrainBasket = LocationGlobalRelative(48.574458, 7.771747, 10)

distanceEnMetre = drone.get_distance_metres(locationMaisonStrasbourgThomas,locationMaisonStrasbourgTerrainBasket)

print("Distance en metre : "+ str(distanceEnMetre))"""

print("Fin du test")
