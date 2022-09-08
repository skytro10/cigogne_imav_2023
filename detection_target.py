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
from math import sqrt
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil 
from array import array
from datetime import datetime
from picamera import PiCamera,Color
from picamera.array import PiRGBArray
from utilities import get_distance_metres, get_GPS_location, get_distance_angle_picture

class Detection:
  def __init__(self, camera, id_to_find):
    self.marker_found = False
    self.whiteSquare_found = False 
    self.camera = camera
    #self.camera.brightness = 50
    self.camera.resolution = (640, 480)
    #camera.resolution = (1920, 1080)
    self.camera.framerate = 32
    self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
  
    #--- Define Tag
    self.id_to_find = id_to_find
    self.marker_size = 5 #- [cm]
    self.found_count = 0 
    self.notfound_count = 0
  
    #--------------- Resolution ---------------------------
    # Focal length and sensors dimensions for Pi camera
    # See: https://www.raspberrypi.com/documentation/accessories/camera.html 
    focal_length = 3.60   # Focal length [mm]
    horizotal_res = 640   # Horizontal resolution (x dimension) [px] 
    vertical_res = 480    # Vertical resolution (y dimension) [px]
    sensor_length = 3.76  # Sensor length (x dimension) [mm]
    sensor_height = 2.74  # Sensor length (y dimension) [mm]  
    self.dist_coeff_x = sensor_length/(focal_length*horizotal_res)
    self.dist_coeff_y = sensor_height/(focal_length*vertical_res)
    self.x_imageCenter = int(horizotal_res/2)
    self.y_imageCenter = int(vertical_res/2)
  
    self.img_compteur = 0
    dossier = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
    parent_dir = '/home/housso97/Desktop/code_IMAV2022_Thomas/saved_images'
    self.path = os.path.join(parent_dir, dossier)
    os.mkdir(self.path)
  
    #--- Camera calibration path
    calib_path  = "/home/housso97/Desktop/camera_01/cameraa_02/"
    self.camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
    self.camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
  
    #--- Definir le dictionnaire aruco 
    self.aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
    self.parameters  = aruco.DetectorParameters_create()
  
    self.closeToAruco = False

    #--------------- Saved Markers ---------------------------
    # self.saved_markers = {-1: (LocationGlobalRelative(48.58111,7.764722,0), False)}

  def Detection_aruco(self, latitude, longitude, altitude, heading, saved_markers, id_to_test, research_whiteSquare):
    
    #--- Capturer le videocamera 
    self.camera.capture(self.rawCapture, format="bgr")
    frame = self.rawCapture.array
    self.rawCapture.truncate(0)
    
    #frame = cv2.imread("/home/housso97/Desktop/code_IMAV2022_Thomas/saved_images/2022_09_07-12:17:38_PM/mask_closingTest_1_Img_29_lat_48.5809508lon_7.7648142alt_23.8799991607666.png")
    
    font = cv2.FONT_HERSHEY_PLAIN  # Text font for frame annotation
    
    """print("Captured frame: " + str(type(frame)))  
    frame = cv2.flip (frame,-1)
    print("Flipped frame: " + str(type(frame)))
    cv2.waitKey(33)"""
    
    self.img_compteur+=1
    
    ########################## traitement pour aruco
    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
  
    #-- Trouver tous les marquers dans l'image 
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self.aruco_dict, parameters=self.parameters,
                              cameraMatrix=self.camera_matrix, distCoeff=self.camera_distortion)
    
    # print("ids : "+str(ids))
    self.marker_found = False
    self.whiteSquare_found = False

    #--------------- Detection ArUco Tags ---------------------------
    if ids is not None : # and ids[0] == Detection.id_to_find:
      x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
      y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
      x_centerPixel_target = int(x_sum*.25)
      y_centerPixel_target = int(y_sum*.25)
      arrete_marker_pxl = sqrt((corners[0][0][0][0]-corners[0][0][1][0])**2+(corners[0][0][0][1]-corners[0][0][1][1])**2)
        
      cv2.line(frame, (x_centerPixel_target, y_centerPixel_target-20), (x_centerPixel_target, y_centerPixel_target+20), (0, 0, 255), 2)
      cv2.line(frame, (x_centerPixel_target-20, y_centerPixel_target), (x_centerPixel_target+20, y_centerPixel_target), (0, 0, 255), 2)

      # Estimating marker location from vision
      distance_vision, angle_vision = get_distance_angle_picture(self.x_imageCenter, self.y_imageCenter,
                                                                 x_centerPixel_target, y_centerPixel_target,
                                                                 altitude, self.dist_coeff_x, self.dist_coeff_y)
      current_location = LocationGlobalRelative(latitude, longitude, 0)
      estimated_location = get_GPS_location(current_location, heading + angle_vision, distance_vision)

      saved_location = saved_markers[id_to_test][0]
      distance_meters = get_distance_metres(estimated_location, saved_location)

      # If the white square of interest is located at the ArUco place
      if distance_meters < 7:
        saved_markers[ids[0]] = saved_markers[id_to_test]  # Put white square infos inside ArUco ids
        saved_markers.pop(id_to_test)                      # Remove current white square id

      if ids[0] == self.id_to_find:
        self.marker_found = True
      
      #-- Incrementer les compteurs 
      self.found_count+=1
      # print("marquer trouve")
      # print("found_count : "+str(self.found_count))
      
        
    #--------------- Detection White Squares ------------------------
    elif research_whiteSquare == True :
      #------------- Image processing for white squares -------------
      blur = cv2.GaussianBlur(frame,(5,5),0)       # Gaussian blur filter  
      hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)  # Convert from BGR to HLS color space  
      lower_bound = (0,200,0)     # Select white color in HLS space
      upper_bound = (255,255,255)
      mask_hls = cv2.inRange(hls, lower_bound, upper_bound)
      cv2.imwrite(os.path.join(self.path, "mask_hls"+name), mask_hls)
      # Closing detected elements
      closing_kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))
      mask_closing = cv2.morphologyEx(mask_hls, cv2.MORPH_CLOSE, closing_kernel)
      cv2.imwrite(os.path.join(self.path, "mask_closing"+name), mask_closing)
      contours, hierarchy = cv2.findContours(mask_closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      #print ("aire max : "+str(30000*altitude**-1.743))
      #print ("aire min : "+str(25000*altitude**-1.743))

      #--------------White square corners ---------------------------
      for c in contours:
        # pour identifier un carre
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        area = cv2.contourArea(c)
        
        #print ("aire du carre : "+str(area))
        #print( "alt : "+ str(altitude))
        
        #--------------- Altitude and square filters ------------------
        if altitude == 0.0:
          break
        if area < 70000*altitude**-2  and area > 10000*altitude**-2 and len(approx) ==4 and altitude > 5:
          # print("Detection area correspondant")
          (x, y, w, h) = cv2.boundingRect(approx)
          ar = w / float(h)
          print("ar : "+str(ar))
          if ar >= 0.90 and ar <= 1.10:  # Square filter
            # print ("Detection carre blanc OK")
            # cv2.drawContours(frame, [c], -1, (0, 0, 255), 1)
            x_centerPixel_target = np.mean(c, axis=0)[0][0]
            y_centerPixel_target = np.mean(c, axis=0)[0][1]
            arrete_marker_pxl = sqrt(area)
            
            pixelTest = mask_closing[int(y_centerPixel_target),int(x_centerPixel_target)]
            #print("pixelTest : "+str(pixelTest))
            if pixelTest == 255 :  #verifie couleur du carre detecte 255 c est blanc
              self.whiteSquare_found = True
              # cv2.drawContours(frame, [c], -1, (255, 0, 0), 1)
              cv2.line(frame, (int(x_centerPixel_target), int(y_centerPixel_target)-20), (int(x_centerPixel_target), int(y_centerPixel_target)+20), (0, 0, 255), 2)
              cv2.line(frame, (int(x_centerPixel_target)-20, int(y_centerPixel_target)), (int(x_centerPixel_target)+20, int(y_centerPixel_target)), (0, 0, 255), 2)

              # Estimating marker location from vision
              distance_vision, angle_vision = get_distance_angle_picture(self.x_imageCenter, self.y_imageCenter,
                                                                 x_centerPixel_target, y_centerPixel_target,
                                                                 altitude, self.dist_coeff_x, self.dist_coeff_y)
              current_location = LocationGlobalRelative(latitude, longitude, 0)
              estimated_location = get_GPS_location(current_location, heading + angle_vision, distance_vision)

              # White square found and compared to dictionary
              self.new_location_found = False
              white_square_id = 0
              for id_markers in saved_markers:
                saved_location = saved_markers[id_markers][0]
                distance_meters = get_distance_metres(estimated_location, saved_location)
                # print(distance_meters)

                # White square already checked with location fusion
                if distance_meters < 7:
                  white_square_id = id_markers
                  # Location already found
                else:
                  new_location_found = True

              # Storing new white squares in dictionary
              if new_location_found:
                if max(saved_markers.keys()) <= 1000:
                  white_square_id = 1001
                  # print("New location found")
                  # cv2.line(frame, (int(x_centerPixel_target), int(y_centerPixel_target)-20), (int(x_centerPixel_target), int(y_centerPixel_target)+20), (0, 255, 0), 2)
                  # cv2.line(frame, (int(x_centerPixel_target)-20, int(y_centerPixel_target)), (int(x_centerPixel_target)+20, int(y_centerPixel_target)), (0, 255, 0), 2)
                else:
                  max_id = max(saved_markers.keys())
                  white_square_id = max_id + 1
                  # cv2.line(frame, (int(x_centerPixel_target), int(y_centerPixel_target)-20), (int(x_centerPixel_target), int(y_centerPixel_target)+20), (0, 255, 0), 2)
                  # cv2.line(frame, (int(x_centerPixel_target)-20, int(y_centerPixel_target)), (int(x_centerPixel_target)+20, int(y_centerPixel_target)), (0, 255, 0), 2)
                saved_markers[white_square_id] = (estimated_location, False)
              cv2.putText(frame, str(white_square_id), (x_centerPixel_target, y_centerPixel_target), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

    detect_string = "yes"
    if self.marker_found == False and self.whiteSquare_found == False:
      self.notfound_count+=1
      x_centerPixel_target = None
      y_centerPixel_target = None
      detect_string = "no"
      # print ("aruco and white square likely not found")
      # print("notfound_count : "+str(self.notfound_count))  
        
    cv2.circle(frame, (320, 240), 50, (255,255,255), 1)
    cv2.line(frame, (self.x_imageCenter, self.y_imageCenter-20), (self.x_imageCenter, self.y_imageCenter+20), (255, 0, 0), 2)
    cv2.line(frame, (self.x_imageCenter-20, self.y_imageCenter), (self.x_imageCenter+20, self.y_imageCenter), (255, 0, 0), 2)

    name = "Test_1_Img_" + str(self.img_compteur) + "_"+ detect_string + "_lat_" + str(latitude)+ "lon_" + str(longitude) + "alt_" + str(altitude) +".png"
    cv2.imwrite(os.path.join(self.path, name), frame)
    # print("Image saved !")
    
    return x_centerPixel_target, y_centerPixel_target, self.marker_found, self.whiteSquare_found, saved_markers
