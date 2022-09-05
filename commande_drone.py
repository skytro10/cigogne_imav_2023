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


class Drone:
  def connectionPixhawk():
    #--------------------- Connection ----------------------------
    chemin_drone = '/dev/ttyACM0'
    self.vehicle = connect(chemin_drone, wait_ready=True, baud=57600, heartbeat_timeout=2)
    print("Connection OK")
  
  def arm_and_takeoff(self, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not self.vehicle.is_armable:
      print(" Waiting for vehicle to initialise...")
      time.sleep(1)
    print("Arming motors")
    # Copter should arm in GUIDED mode
    self.vehicle.mode = VehicleMode("GUIDED")
    self.vehicle.armed = True
  
    while not self.vehicle.armed:
      print(" Waiting for arming...")
      time.sleep(1)
  
    print("Taking off!")
    self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
  
    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)      
        if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
  
  def move_servo(self, channel, ouvert, duration=1):
      if ouvert:
          pwm = 1100
          print("Opening servo")
      else:
          pwm = 1800
          print("Closing servo")
      msg = self.vehicle.message_factory.command_long_encode(0, 0,  # target_system, target_component
                                                      mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
                                                      0,  # confirmation
                                                      channel,  # servo number
                                                      pwm,  # servo position between 1000 ferme and 2000
                                                      0, 0, 0, 0, 0)  # param 3 ~ 7 not used
      # send command to vehicle
      for x in range(0,duration) :
  		
          self.vehicle.send_mavlink(msg)
          time.sleep(0.2)
  
  #set_mode - set the mode of the vehicle as long as we are in control
  def set_mode(self, mode):
    self.vehicle.mode = VehicleMode(mode)
    print("set_mode : "+str(mode))
    self.vehicle.flush()
   
  #get_mode - get current mode of vehicle 
  def get_mode(self):
    last_mode = self.vehicle.mode.name
    return last_mode

  def set_velocity(self, velocity_x, velocity_y, velocity_z, duration):
  	# only let commands through at 10hz
  	print("vx %s, vy %s, vz %s. \n" % (velocity_x, velocity_y, velocity_z))
  
  	#global last_set_velocity
  	#last_set_velocity = time.time()
  
  	# create the SET_POSITION_TARGET_LOCAL_NED command
  	msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
  		0,  # time_boot_ms (not used)
  		0, 0,  # target system, target component
  		mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
  		0x0DC7,  # type_mask (ignore pos | ignore acc)
  		0, 0, 0,  # x, y, z positions (not used)
  		velocity_x, velocity_y, velocity_z,
  		# x, y, z velocity in m/s -- X positive forward or North/ Y positive right or East / Z positive down
  		0, 0, 0,  # x, y, z acceleration (not used)
  		0, 0)  # yaw, yaw_rate (not used)
  
  	# send command to vehicle
  	for x in range(0,duration) :
  		self.vehicle.send_mavlink(msg)
  		time.sleep(0.1)
     
  def get_distance_metres(aLocation1, aLocation2):  ### changer pour code different de lat1 - lat2 ...
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
   
  def goto(self, targetLocation, vitesse=2.5):    ### modif changer la condition validant Reached target par une condition GPS avec rayon  
    ############################################vehicle.airspeed(vitesse)
    ############################################print("Vitesse de vol : "+str(vitesse))
    currentLocation = self.vehicle.location.global_relative_frame
    targetDistance = Drone.get_distance_metres(currentLocation, targetLocation)

    # Haversine formula to compute distance between two GPS points
    targLat = point.lat
    targLon = point.lon
    realLat = vehicle.location.global_relative_frame.lat
    realLon = vehicle.location.global_relative_frame.lon

    R = 6371000  # Mean earth radius (meters)
    phi_1 = radians(targLat)
    phi_2 = radians(targLat)
    delta_phi = radians(targLat-realLat)    # Latitude difference (radians)
    delta_theta = radians(targLon-realLon)  # Longitude difference (radians)
    a = sin(delta_phi/2)**2 + cos(phi_1) * cos(phi_2) * sin(delta_theta/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    d = R * c
    print("Distance to the GPS target: %.2fm" % d)

    if d <= 20:         # If the distance to the target is less than 20m
        arrived = True  # Change the boolean variable to True
        print("Arrived at GPS target !")
    
    self.vehicle.simple_goto(targetLocation)                          
    
    while self.vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
      #print "DEBUG: mode: %s" % vehicle.mode.name
      remainingDistance=Drone.get_distance_metres(self.vehicle.location.global_relative_frame, targetLocation)
      print("Distance to target: ", remainingDistance)
      if remainingDistance<=targetDistance*0.1: #Just below target, in case of undershoot.
          print("Reached target")
          break;
      time.sleep(1)

  def lancement_decollage(self, altitudeDeVol):
    #Initialisaion du programme en mode stabilize
    self.vehicle.mode = VehicleMode("STABILIZE")
    #verrouillage servomoteur de larguage
    self.move_servo(10,False)
    while True:
            print ("en attente de auto")
            print ("mode: %s" % self.vehicle.mode)
            if (self.vehicle.mode == VehicleMode("AUTO")):
                    self.arm_and_takeoff(altitudeDeVol)
                    break
            time.sleep(0.25)
        
"""
# test du code
channel_servo = 10

connectionPixhawk()     
move_servo(vehicle,channel_servo,False)
time.sleep(1)
move_servo(vehicle,channel_servo,True)
time.sleep(1)
move_servo(vehicle,channel_servo,False)
time.sleep(1)"""
