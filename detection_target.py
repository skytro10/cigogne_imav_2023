
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
from math import atan2, cos, radians, sin, sqrt, pi
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil 
from array import array
from datetime import datetime
from picamera import PiCamera,Color
from picamera.array import PiRGBArray

class Detection:
  ####################### Variables ##########################
  """global camera
  global marker_found
  global whiteSquare_found
  global rawCapture
  global id_to_find
  global found_count
  global notfound_count
  global x_imageCenter
  global y_imageCenter
  global img_compteur
  global dossier
  global parent_dir
  global path
  global aruco_dict
  global parameters
  global closeToAruco"""
  
  marker_found = False
  whiteSquare_found = False 
  camera = PiCamera()
  camera.brightness = 50
  camera.resolution = (640, 480)
  #camera.resolution = (1920, 1080)
  camera.framerate = 32
  rawCapture = PiRGBArray(camera, size=(640, 480))
  
  #--- Define Tag
  id_to_find  = 69
  marker_size  = 5 #- [cm]
  found_count = 0 
  notfound_count = 0
  
  #--------------- Resolution ---------------------------
  
  horizotanle_res = 640 
  vertical_res = 480
  x_imageCenter = int(horizotanle_res/2)
  y_imageCenter = int(vertical_res/2)
  
  img_compteur = 0
  dossier = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
  parent_dir = '/home/housso97/Desktop/code_IMAV2022_Thomas/saved_images'
  path = os.path.join(parent_dir, dossier)
  os.mkdir(path)
  
  #--- Camera calibration path
  calib_path  = "/home/housso97/Desktop/camera_01/cameraa_02/"
  camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
  camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
  
  #--- Definir le dictionnaire aruco 
  aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
  parameters  = aruco.DetectorParameters_create()
  
  closeToAruco = False


  def Detection_aruco(latitude,longitude,altitude):
    
    #--- Capturer le videocamera 
    Detection.camera.capture(Detection.rawCapture, format="bgr")
    frame = Detection.rawCapture.array
    Detection.rawCapture.truncate(0)
    
    #definir a quoi ca sert
    font = cv2.FONT_HERSHEY_PLAIN
    
    """print("Captured frame: " + str(type(frame)))  
    frame = cv2.flip (frame,-1)
    print("Flipped frame: " + str(type(frame)))
    cv2.waitKey(33)"""
      
    name = "Test_1_Img_" + str(Detection.img_compteur)+ "_lat_" + str(latitude)+ "lon_" + str(longitude) + "alt_" + str(altitude) +".png"
          
    Detection.img_compteur+=1
    
    
    
    ########################## traitement pour aruco peut etre a optimiser avec Detection pour carre blanc
    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
  
    #-- Trouver tous les marquers dans l'image 
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=Detection.aruco_dict, parameters=Detection.parameters,
                              cameraMatrix=Detection.camera_matrix, distCoeff=Detection.camera_distortion)
    
    print("ids : "+str(ids))
    
    if ids is not None : # and ids[0] == Detection.id_to_find:
        
        Detection.marker_found = True
        Detection.whiteSquare_found = False
        
        x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
        y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
        x_centerPixel_target = int(x_sum*.25)
        y_centerPixel_target = int(y_sum*.25)
        
        cv2.line(frame, (x_centerPixel_target, y_centerPixel_target-20), (x_centerPixel_target, y_centerPixel_target+20), (0, 0, 255), 2)
        cv2.line(frame, (x_centerPixel_target-20, y_centerPixel_target), (x_centerPixel_target+20, y_centerPixel_target), (0, 0, 255), 2)
        #-- Incrementer les compteurs 
        Detection.found_count+=1
        print("marquer trouve")
        print("found_count : "+str(Detection.found_count))
        
    ################## Detection carree blanc####################      
    else :
      Detection.marker_found = False
      ########################## traitement pour Detection carre blanc
      blur = cv2.GaussianBlur(frame,(5,5),0)
      # Convert from BGR to HSV color space
      hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)
      # Select white color in HSV space
      lower_bound = (0,200,0)
      upper_bound = (255,255,255)
      # Get the saturation plane - all black/white/gray pixels are zero, and colored pixels are above zero.
      # s = hsv[:, :, 1]
      # Apply threshold on s
      mask_hls = cv2.inRange(hls, lower_bound, upper_bound)
      cv2.imwrite(os.path.join(Detection.path, "mask_hls"+name), mask_hls)
      # Closing detected elements
      closing_kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))
      mask_closing = cv2.morphologyEx(mask_hls, cv2.MORPH_CLOSE, closing_kernel)
      cv2.imwrite(os.path.join(Detection.path, "mask_closing"+name), mask_closing)
      contours, hierarchy = cv2.findContours(mask_closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      #print ("aire max : "+str(30000*altitude**-1.743))
      #print ("aire min : "+str(25000*altitude**-1.743))
      
      Detection.whiteSquare_found = False
      
      for c in contours:
        # pour identifier un carre
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        area = cv2.contourArea(c)
        print ("aire du carre : "+str(area))
        print( "alt : "+ str(altitude))
  
        
        if area < 70000*altitude**-2  and area > 15000*altitude**-2 and len(approx) ==4 and altitude > 5 :
          
  
          print("Detection area correspondant")
          (x, y, w, h) = cv2.boundingRect(approx)
          ar = w / float(h)
          
          if ar >= 0.80 and ar <= 1.20 :
            print ("Detection carre blanc OK")
            cv2.drawContours(frame, [c], -1, (0, 0, 255), 1)
  
  
            x_centerPixel_target = np.mean(c, axis=0)[0][0]
            y_centerPixel_target = np.mean(c, axis=0)[0][1]
            
            cv2.drawContours(frame, [c], -1, (255, 0, 0), 1)
            cv2.line(frame, (int(x_centerPixel_target), int(y_centerPixel_target)-20), (int(x_centerPixel_target), int(y_centerPixel_target)+20), (0, 0, 255), 2)
            cv2.line(frame, (int(x_centerPixel_target)-20, int(y_centerPixel_target)), (int(x_centerPixel_target)+20, int(y_centerPixel_target)), (0, 0, 255), 2)
            Detection.whiteSquare_found = True
            
            

            
            
    if Detection.marker_found == False and Detection.whiteSquare_found == False:
      Detection.notfound_count+=1
      x_centerPixel_target = None
      y_centerPixel_target = None
      print ("aruco and white square likely not found")
      print("notfound_count : "+str(Detection.notfound_count))  
        
    cv2.circle(frame, (320, 240), 50, (255,255,255), 1)
    cv2.line(frame, (Detection.x_imageCenter, Detection.y_imageCenter-20), (Detection.x_imageCenter, Detection.y_imageCenter+20), (255, 0, 0), 2)
    cv2.line(frame, (Detection.x_imageCenter-20, Detection.y_imageCenter), (Detection.x_imageCenter+20, Detection.y_imageCenter), (255, 0, 0), 2)
      
    cv2.imwrite(os.path.join(Detection.path, name), frame)
    print("Image saved !")
    
    
    
    return Detection.x_imageCenter, Detection.y_imageCenter, x_centerPixel_target, y_centerPixel_target, Detection.marker_found, Detection.whiteSquare_found
      

#### pour tester le code

#Detection_aruco(1,2,6.36)
      

  
