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



#--------------------- Parametres du vehicule ----------------------------
vitesse = .3 #m/s
altitudeDeVol = 20
#exemple de position GPS
  #locationMaisonStrasbourgTerrainBasket = LocationGlobalRelative(48.574458, 7.771747, 10)
research_whiteSquare = True
distanceAccuracy = 2 # rayon en metre pour valider un goto

global goodID_marker_found
global whiteSquare_found
global id_to_test
id_to_test = -1
global saved_markers
saved_markers = {-1: (LocationGlobalRelative(48.58111,7.764722,0), True)}

###################### Thread creation et appel de fonction ####################

class myThread(threading.Thread):

  

  def __init__(self, threadID, name, altitudeDeVol, drone_object, research_whiteSquare):
    threading.Thread.__init__(self)
    self._stop_event = threading.Event()
    
    self.threadID = threadID
    self.name = name
    self.altitudeDeVol = altitudeDeVol
    self.drone_object = drone_object
    self.research_whiteSquare = research_whiteSquare
    
  def run(self):
    print ("Starting " + self.name)
    
    
    # setup variable global pour les threads
    global compteur_no_detect
    compteur_no_detect = 0
    global compteur_whiteSquare
    compteur_whiteSquare = 0
    global compteur_aruco
    compteur_aruco = 0
    global package_dropped
    package_dropped = False
    global x_centerPixel_target
    x_centerPixel_target = None
    global y_centerPixel_target
    y_centerPixel_target = None
    global x_imageCenter
    x_imageCenter = 0
    global y_imageCenter
    y_imageCenter = 0
    global altitudeAuSol
    global altitudeRelative
    global longitude
    global latitude
    
    if self.name == "Thread_Detection_target":
      print("[visiont] HELLO WORLD! Vision thread launched!")
      while True :
        
        # actualisation de l altitude et gps
        altitudeAuSol = drone_object.vehicle.rangefinder.distance
        altitudeRelative = drone_object.vehicle.location.global_relative_frame.alt
        longitude = drone_object.vehicle.location.global_relative_frame.lon
        latitude = drone_object.vehicle.location.global_relative_frame.lat
        heading = drone_object.vehicle.attitude.yaw
        
        #le srcipt Detection Target
        x_centerPixel_target, y_centerPixel_target, goodID_marker_found, whiteSquare_found, saved_markers = Detection.Detection_aruco(latitude, longitude, altitude, heading, saved_markers, id_to_test, research_whiteSquare)
        if goodID_marker_found == True :
          compteur_aruco += 1
        elif whiteSquare_found == True :
          compteur_whiteSquare += 1
        else :
          compteur_no_detect += 1
          
        
        
        if (not drone_object.get_mode() == "GUIDED" and not drone_object.get_mode() == "AUTO")  or altitudeRelative > 30 : #arret du Thread en cas de changement de mode, de larguage ou d'altitude sup a 25m
          
          break
      
      print("[visiont] Fin du thread. BYE BYE!")

    if self.name == "Thread_asservissement":
      print("[asservt] HI GUYS! Asserv thread launched!")
      dErrx = 0
      dErry = 0
      errsumx = 0
      errsumy = 0
      
      last_errx = 0
      last_erry = 0
      
      # PD Coefficients
      kpx = 0.004
      kpy = 0.004
      kdx = 0.0001  # 0.00001 working "fine" for both
      kdy = 0.0001
      kix = 0.000001  # 0.0000001
      kiy = 0.000001
      
      vx = 0
      vy = 0
      vz = 0
        
      while True :
        
        if (not drone_object.get_mode() == "GUIDED" and not drone_object.get_mode() == "AUTO") or package_dropped == True :
          drone_object.set_velocity(0, 0, 0, 1)
          break
          

             
        if altitudeAuSol < 5 :
          kpx = 0.001
          kpy = 0.001
          #kix = 0.000001  # 0.0000001
          #kiy = 0.000001
        else :
          kpx = 0.002
          kpy = 0.002
          
          
        if x_centerPixel_target == None or y_centerPixel_target == None :   # echec Detection
          
          if compteur_no_detect > 10 :   #on fixe le nombre d'image consecutive sans Detection pour considerer qu il ne detecte pas
            if altitudeRelative > 25 :  # si on altitudeRelative sup a 25m stop le thread
              drone_object.set_velocity(0, 0, 0, 1)
              #print ("altitudeRelative > 30")
              break
            else :  # pas de Detection Drone prend de l altitude
              vx = 0
              vy = 0
              vz = -0.5
              drone_object.set_velocity(vx, vy, vz, 1)
              #print ("prise d'altitude vz = -0.5")
          
          elif compteur_no_detect > 2 :   # fixer la position du Drone en cas de non Detection
            drone_object.set_velocity(0, 0, 0, 1)
            #print ("compteur_no_detect > 2   stabilisation drone")
              
                 
        
        else :  # Detection ok 
      
          dist_center = math.sqrt((detection_object.x_imageCenter-x_centerPixel_target)**2+(detection_object.y_imageCenter-y_centerPixel_target)**2)
          errx = detection_object.x_imageCenter - x_centerPixel_target
          erry = detection_object.y_imageCenter - y_centerPixel_target
          if abs(errx) <= 10:   #marge de 10pxl pour considerer que la cible est au centre de l image
                  errx = 0
          if abs(erry) <= 10:
                  erry = 0
        
          # PD control
          dErrx = (errx - last_errx)# / delta_time
          dErry = (erry - last_erry)# / delta_time
          errsumx += errx# * delta_time
          errsumy += erry# * delta_time
        
          #~ print("errsumx: %s, errsumy: %s" % (errsumy, errsumy))
        
          vx = (kpx * errx) + (kdx * dErrx) + (kix * errsumx)
          vy = (kpy * erry) + (kdy * dErry) + (kiy * errsumy)
          
          if altitudeAuSol < 3 :
            vz = 0.2  # a changer pour descendre
          elif altitudeAuSol > 7 :
            vz = 1  # a changer pour descendre
          else :
            vz = 0.5
          # Establish limit to outputs
          vx = min(max(vx, -5.0), 5.0)
          vy = min(max(vy, -5.0), 5.0)
          vx = -vx                        # High opencv is south Dronekit
        
          # Dronekit
          # X positive Forward / North
          # Y positive Right / East
          # Z positive down
        
          # Last error for Derivative
          
          last_errx = errx
          last_erry = erry
          
          if dist_center <= 50 :
            drone_object.set_velocity(vy, vx, vz, 1) 
            #print("vy : "+str(vy)+" vx : "+str(vx)+" vz : "+str(vz)+" dist_center <= 30")
          else :
            #lancer un deplacement pour ce rapprocher du centre sans descendre ou monter
            if altitudeAuSol < 2 :
              vz = 0

            drone_object.set_velocity(vy, vx, vz, 1)  # Pour le sense de la camera, X controle le 'east' et Y controle le 'North'
            #print("vy : "+str(vy)+" vx : "+str(vx)+" vz : "+str(vz)+" dist_center decale")
        
      print("[asservt] Fin du thread. SEE YOU SOON!")

  def stop(self):
    self._stop_event.set()

  def stopped(self):
    return self._stop_event.is_set()

#--------------------------------------------------------------
def mission_largage_GPS_connu(GPS_target_delivery, id_to_find):
  print("[mission] Mission1 started: Delivery at known location.")
  drone_object = Drone()    #permet de connecter le drone via dronekit en creant l objet drone
  detection_object = Detection(PiCamera(), id_to_find)  # creer l objet detection
  #########verrouillage servomoteur et procedure arm and takeoff
  drone_object.lancement_decollage(altitudeDeVol)
  #########Drone se deplace sur cible
  drone_object.goto(GPS_target_delivery, distanceAccuracy)
  
  
  #Drone.move_servo(vehicle,10, False)
  #########Create new threads
  myThread_Detection_target= myThread(1, "Thread_Detection_target",altitudeDeVol,None,research_whiteSquare)
  myThread_asservissement= myThread(2, "Thread_asservissement",altitudeDeVol,drone_object,research_whiteSquare)  

  #########debut de la Detection et du mouvement
  myThread_Detection_target.start()
  time.sleep(1)
  myThread_asservissement.start()
  
  """# Ensure to stop a thread if the other is stopped
  # https://stackoverflow.com/questions/323972/is-there-any-way-to-kill-a-thread
  while not myThread_Detection_target.stopped() and not myThread_asservissement.stopped():
    if myThread_Detection_target.stopped():
      myThread_asservissement.stop()
    if myThread_asservissement.stopped():
      myThread_Detection_target.stop()"""
      
  #########attente de la fin de la Detection et du mouvement
  myThread_Detection_target.join()
  myThread_asservissement.join()
  
  if drone_object.get_mode() == "GUIDED" or drone_object.get_mode() == "AUTO") :  #securite pour ne pas que le drone reprenne la main en cas d interruption
    #########repart en mode RTL
    drone_object.set_mode("RTL") #### modif preciser qu on est en guided avant et ajouter l altitude du RTL
#--------------------------------------------------------------

#--------------------------------------------------------------
"""
def mission_largage_GPS_incertain(GPS_target_delivery, id_to_find):

  drone_object = Drone()    #permet de connecter le drone via dronekit en creant l objet drone
  detection_object = Detection(PiCamera(), id_to_find)  # creer l objet detection
  
  #########verrouillage servomoteur et procedure arm and takeoff
  drone_object.lancement_decollage(altitudeDeVol)
  #########Drone se deplace sur cible
  drone_object.goto(GPS_target_delivery, distanceAccuracy)
  
  
  #Drone.move_servo(vehicle,10, False)
  #########Create new threads
  myThread_Detection_target= myThread(1, "Thread_Detection_target",altitudeDeVol,None,research_whiteSquare)
  myThread_asservissement= myThread(2, "Thread_asservissement",altitudeDeVol,drone_object,research_whiteSquare)  

  #########debut de la Detection et du mouvement
  myThread_Detection_target.start()
  time.sleep(1)
  myThread_asservissement.start()
  
  # Ensure to stop a thread if the other is stopped
  # https://stackoverflow.com/questions/323972/is-there-any-way-to-kill-a-thread
  while not myThread_Detection_target.stopped() and not myThread_asservissement.stopped():
    if myThread_Detection_target.stopped():
      myThread_asservissement.stop()
    if myThread_asservissement.stopped():
      myThread_Detection_target.stop()
      
  #########attente de la fin de la Detection et du mouvement
  myThread_Detection_target.join()
  myThread_asservissement.join()
  
  if drone_object.get_mode() == "GUIDED" or drone_object.get_mode() == "AUTO") :  #securite pour ne pas que le drone reprenne la main en cas d interruption
    #########repart en mode RTL
    drone_object.set_mode("RTL") #### modif preciser qu on est en guided avant et ajouter l altitude du RTL
"""
#--------------------------------------------------------------

#--------------------------------------------------------------
def mission_largage_zone_inconnu(id_to_find):
  print("[mission] Mission3 started: Delivery at unknown location.")
  drone_object = Drone()    #permet de connecter le drone via dronekit en creant l objet drone
  detection_object = Detection(PiCamera(), id_to_find)  # creer l objet detection
  
  #########verrouillage servomoteur et procedure arm and takeoff
  print("[mission] Mission3 started: Delivery at unknown location.")
  drone_object.lancement_decollage(altitudeDeVol)
  
  #########passage en mode AUTO et debut de la mission
  drone_object.passage_mode_Auto()
  
  #########Create new threads
  myThread_Detection_target= myThread(1, "Thread_Detection_target",altitudeDeVol,None,research_whiteSquare)
  myThread_asservissement= myThread(2, "Thread_asservissement",altitudeDeVol,drone_object,research_whiteSquare)  
  
  # a partir d'un certain waypoint declencher le thread de detection
  
  while drone_object.vehicle.commands.next <= 2 :
    pass
    
  
  #########debut de la Detection 
  myThread_Detection_target.start()
  time.sleep(1)

  while drone_object.get_mode() == "GUIDED" or drone_object.get_mode() == "AUTO":
  
    #--------------- Good ArUco ID found -----------------------
    if goodID_marker_found == True :
      #compteur_whiteSquare = 0
      compteur_no_detect = 0
      print("[mission] Good aruco tag detected !")
      
      dist_center = math.sqrt((detection_object.x_imageCenter-x_centerPixel_target)**2+(y_imageCenter-y_centerPixel_target)**2)
      print("[mission] Current distance: %.2fpx ; Altitude: %.2fm." % (dist_center, altitudeAuSol))
        
      if not myThread_asservissement.is_alive() : #verifie que le thread d asservissement n est pas deja lance si non lancement du thread asservissement
        drone_object.set_mode("GUIDED")
        time.sleep(O.3)
        myThread_asservissement.start()
        
      if dist_center <= 50 and altitudeAuSol < 1.5 :  # condition pour faire le largage
        print("[mission] Largage !")
        drone_object.move_servo(10, True)
        time.sleep(0.5)
        package_dropped = True
        break
      
    #--------------- Some white square found -------------------
    elif whiteSquare_found == True :
      compteur_aruco = 0
      compteur_no_detect = 0
      print("[mission] Detection of 1 or many white squares (%i times)" % compteur_whiteSquare)

      # Check saved_ids in detection dictionary
      for saved_id in saved_markers :
        # Check boolean: if True, needs to be explored
        if saved_markers[saved_id][1] == False:
                print("[mission] Detection of 1 or many white squares (%i times)" % compteur_whiteSquare)
          if not myThread_asservissement.is_alive() : #verifie que le thread d asservissement n est pas deja lance si non lancement du thread asservissement
            drone_object.set_mode("GUIDED")
            time.sleep(O.3)
            myThread_asservissement.start()
          id_to_test = saved_id
          break
              
    else :
      compteur_aruco = 0
      #compteur_whiteSquare = 0
      print("[mission] No detection or wrong tag (%i times)" % compteur_no_detect)
        
      #--------------- No white square, no ArUco -----------------
      if compteur_no_detect > 10 and compteur_whiteSquare =! 0 :
        print("[mission] 10 times without tag detection, not interesting place.")
        saved_markers[id_to_test][1] = True  # Saved markers explored, not interesting
        id_to_test = -1
        drone_object.passage_mode_Auto()
        compteur_whiteSquare = 0
        # Gestion mission auto, voir avec Thomas

      
    # print("compteur_aruco = "+str(compteur_aruco))
    # print("compteur_whiteSquare = "+str(compteur_whiteSquare))
    # print("compteur_no_detect = "+str(compteur_no_detect))
  
  
  
  ####### a faire declenchement detection au WP 2 
  ####### faire le declenchement asservissement en passant en guided en fonction d un parametre 
  ####### couper asserv en fonction d une condition et reprise auto
  ####### definir une fin largage ou echec et stopper code avec RTL
  
  # permet d'attendre la fin du thread
  myThread_Detection_target.join()
  
  # if  drone_object.vehicle.get_mode() == "GUIDED" or drone_object.vehicle.get_mode() == "AUTO"):  #securite pour ne pas que le drone reprenne la main en cas d interruption
    #########repart en mode RTL
  drone_object.set_mode("RTL") #### modif preciser qu on est en guided avant et ajouter l altitude du RTL
#--------------------------------------------------------------

#--------------------------------------------------------------
"""
def mission_largage_GPS_silent(GPS_target_delivery, id_to_find):

  drone_object = Drone()    #permet de connecter le drone via dronekit en creant l objet drone
  detection_object = Detection(PiCamera(), id_to_find)  # creer l objet detection
  #########verrouillage servomoteur et procedure arm and takeoff
  drone_object.lancement_decollage(altitudeDeVol)
  #########Drone se deplace sur cible
  drone_object.goto(GPS_target_delivery, distanceAccuracy)
  
  
  #Drone.move_servo(vehicle,10, False)
  #########Create new threads
  myThread_Detection_target= myThread(1, "Thread_Detection_target",altitudeDeVol,None,research_whiteSquare)
  myThread_asservissement= myThread(2, "Thread_asservissement",altitudeDeVol,drone_object,research_whiteSquare)  

  #########debut de la Detection et du mouvement
  myThread_Detection_target.start()
  time.sleep(1)
  myThread_asservissement.start()
  
  """# Ensure to stop a thread if the other is stopped
  # https://stackoverflow.com/questions/323972/is-there-any-way-to-kill-a-thread
  while not myThread_Detection_target.stopped() and not myThread_asservissement.stopped():
    if myThread_Detection_target.stopped():
      myThread_asservissement.stop()
    if myThread_asservissement.stopped():
      myThread_Detection_target.stop()"""
      
  #########attente de la fin de la Detection et du mouvement
  myThread_Detection_target.join()
  myThread_asservissement.join()
  
  if drone_object.get_mode() == "GUIDED" or drone_object.get_mode() == "AUTO") :  #securite pour ne pas que le drone reprenne la main en cas d interruption
    #########repart en mode RTL
    drone_object.set_mode("RTL") #### modif preciser qu on est en guided avant et ajouter l altitude du RTL
"""
#--------------------------------------------------------------

if __name__ == "__main__":
  id_to_find = 69  # List of ids:
  GPS_target_delivery = LocationGlobalRelative(48.7068570, 7.7344260, altitudeDeVol)
  
  # Mission 1: Delivery at known location
  mission_largage_GPS_connu(GPS_target_delivery, id_to_find)

  # Mission 2: Delivery at uncertain location 
  # mission_largage_GPS_incertain(GPS_target_delivery, id_to_find)

  # Mission 3: Delivery at unknown location 
  # mission_largage_zone_inconnu(id_to_find)

  # Mission 4: Silent Delivery
  # mission_largage_GPS_silent(GPS_target_delivery, id_to_find)

  # Mission 5: Delivery on a moving pickup truck
  # Mission 6: Delivery far and fast
  
  print ("fin du code")
