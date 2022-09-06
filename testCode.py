
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


#from detection_target import Detection
from commande_drone import Drone


###################### Dronekit ####################


#--------------------- Connection ----------------------------

monDrone = Drone()

monDrone.save_mission("mission.txt")

monDrone.printfile("mission.txt")





print("Fin du test")