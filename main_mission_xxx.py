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
import sys


#--------------------- Parametres du vehicule ----------------------------
vitesse = .3 #m/s
altitudeDeVol = 15
#exemple de position GPS
  #locationMaisonStrasbourgTerrainBasket = LocationGlobalRelative(48.574458, 7.771747, 10)
research_whiteSquare = True
distanceAccuracy = 2 # rayon en metre pour valider un goto

global aruco_seen
global good_aruco_found
global white_square_seen
global id_to_test
id_to_test = -1
global saved_markers
saved_markers = {-1: (LocationGlobalRelative(52.1715877,4.4169307,0), True)}


# setup variable global pour les threads
global counter_no_detect
counter_no_detect = 0
global counter_white_square
counter_white_square = 0
# global compteur_aruco
# compteur_aruco = 0
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
global altitudeAuSol

class RedirectText(object):
    def __init__(self,aWxTextCtrl):
        self.file=aWxTextCtrl
         
    def flush(self):
        pass
 
    def write(self,string):
        f = open(self.file,'a')
        f.write(string)
        f.close()

log="./test.log"
redir=RedirectText(log)
#sys.stdout=redir
#sys.stderr=redir

###################### Thread creation et appel de fonction ####################

class myThread(threading.Thread):
  global counter_no_detect
  global counter_white_square
  global aruco_seen
  global good_aruco_found
  global white_square_seen
  global x_centerPixel_target
  global y_centerPixel_target
  global altitudeAuSol
  global id_to_test
  global altitudeRelative
  

  def __init__(self, threadID, name, altitudeDeVol, drone_object, research_whiteSquare,saved_markers,detection_object):
    threading.Thread.__init__(self)
    self._stop_event = threading.Event()
    
    self.threadID = threadID
    self.name = name
    self.altitudeDeVol = altitudeDeVol
    self.drone_object = drone_object
    self.research_whiteSquare = research_whiteSquare
    self.saved_markers = saved_markers
    self.detection_object = detection_object

  def run(self):
    print ("Starting " + self.name)
    
    global counter_no_detect
    global counter_white_square
    global aruco_seen
    global good_aruco_found
    global white_square_seen
    global x_centerPixel_target
    global y_centerPixel_target
    global altitudeAuSol
    global id_to_test
    global altitudeRelative

    if self.name == "Thread_Detection_target":
      print("[visiont] HELLO WORLD! Vision thread launched!")
      while True :
        print(self.saved_markers)
        # actualisation de l altitude et gps
        altitudeAuSol = self.drone_object.vehicle.rangefinder.distance
        altitudeRelative = self.drone_object.vehicle.location.global_relative_frame.alt
        longitude = self.drone_object.vehicle.location.global_relative_frame.lon
        latitude = self.drone_object.vehicle.location.global_relative_frame.lat
        heading = self.drone_object.vehicle.attitude.yaw
        
        #le srcipt Detection Target
        x_centerPixel_target, y_centerPixel_target, aruco_seen, good_aruco_found, white_square_seen, self.saved_markers = self.detection_object.Detection_aruco(latitude, longitude, altitudeAuSol, heading,self.saved_markers, id_to_test, research_whiteSquare)
        

        # Arret du Thread en cas de changement de mode, de larguage ou d'altitude sup a 25m
        if (not self.drone_object.get_mode() == "GUIDED" and not self.drone_object.get_mode() == "AUTO")  or altitudeRelative > 30 : 
          break

        #--------------- Case 1: Good ArUco ID found -----------------------
        if good_aruco_found:
          counter_no_detect = 0

        #--------------- Case 2: Some white square seen --------------------
        elif white_square_seen:
          counter_no_detect = 0
          counter_white_square += 1

        #--------------- Case 3: ArUco tag seen but id false ---------------
        elif aruco_seen:
          counter_no_detect = 0

          saved_markers[id_to_test] = (saved_markers[id_to_test][0], True)
          id_to_test = -1
        # print("[mission] Detection targetted towards id %s" % id_to_test)

        #--------------- Case 4: No detection of white or ArUco ------------
        else:
          counter_no_detect += 1
          counter_white_square = 0
      
      print("[visiont] Fin du thread. BYE BYE!")

  def stop(self):
    self._stop_event.set()

  def stopped(self):
    return self._stop_event.is_set()

def asservissement(drone_object, detection_object, last_errx, last_erry, errsumx, errsumy):
  global counter_no_detect
  global counter_white_square
  global aruco_seen
  global good_aruco_found
  global white_square_seen
  global x_centerPixel_target
  global y_centerPixel_target
  global altitudeAuSol
  global id_to_test
  global altitudeRelative
  # PD Coefficients
  kpx = 0.005
  kpy = 0.005
  kdx = 0.0001  # 0.00001 working "fine" for both
  kdy = 0.0001
  kix = 0.000001  # 0.0000001
  kiy = 0.000001
 

  vx = 0
  vy = 0
  vz = 0
               
  # print("ASERVISSEMENT !!!!!!!!!!!!!!!!!!")
             
  if altitudeAuSol < 5 :
    kpx = 0.002
    kpy = 0.002
    #kix = 0.000001  # 0.0000001
    #kiy = 0.000001
  else :
    kpx = 0.005
    kpy = 0.005

  print("Pixels values - x:%s - y:%s" % (x_centerPixel_target, y_centerPixel_target))
  if x_centerPixel_target == None or y_centerPixel_target == None :   # echec Detection
    if counter_no_detect > 10 :   #on fixe le nombre d'image consecutive sans Detection pour considerer qu il ne detecte pas
      if altitudeAuSol > 30 :  # si on altitudeRelative sup a 25m stop le thread
        drone_object.set_velocity(0, 0, 0, 1)
        print ("[asserv] altitudeRelative > 30")
        return 0, 0, 0, 0
      else :  # pas de Detection Drone prend de l altitude
        vx = 0
        vy = 0
        vz = 0
        drone_object.set_velocity(vx, vy, vz, 1)
        #
    # elif counter_no_detect > 5 :   # fixer la position du Drone en cas de non Detection
    drone_object.set_velocity(0, 0, 0, 1)
      #print ("[asserv] compteur_no_detect > 2   stabilisation drone")
  
    errx = 0
    erry = 0
    errsumx = 0
    errsumy = 0
    
  else :  # Detection ok
    print ("[asserv] detection OK")
    dist_center = math.sqrt((detection_object.x_imageCenter-int(x_centerPixel_target))**2+(detection_object.y_imageCenter-int(y_centerPixel_target))**2)
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
      vy = -vy
      
    # Dronekit
    # X positive Forward / North
    # Y positive Right / East
    # Z positive down
    
    if altitudeAuSol < 2 :
      dist_center_threshold = 50
    
    else :
      dist_center_threshold = 1000

    if dist_center <= dist_center_threshold :
      drone_object.set_velocity(vy, vx, vz, 1) 
      #print("vy : "+str(vy)+" vx : "+str(vx)+" vz : "+str(vz)+" dist_center <= 30"
    
    else :
      #lancer un deplacement pour ce rapprocher du centre sans descendre ou monter
      vz = 0
      drone_object.set_velocity(vy, vx, vz, 1)  # Pour le sense de la camera, X controle le 'east' et Y controle le 'North'
      #print("vy : "+str(vy)+" vx : "+str(vx)+" vz : "+str(vz)+" dist_center decale")

  # Return last errors and sums for derivative and integrator terms
  return errx, erry, errsumx, errsumy
  

#--------------------------------------------------------------
def mission_largage_GPS_connu(GPS_target_delivery, id_to_find):
  global counter_no_detect
  global counter_white_square
  global aruco_seen
  global good_aruco_found
  global white_square_seen
  global x_centerPixel_target
  global y_centerPixel_target
  global altitudeAuSol
  
  print("[mission] Mission1 started: Delivery at known location.")
  drone_object = Drone()    #permet de connecter le drone via dronekit en creant l objet drone
  detection_object = Detection(PiCamera(), id_to_find)  # creer l objet detection
  #########verrouillage servomoteur et procedure arm and takeoff
  drone_object.lancement_decollage(altitudeDeVol)
  #########Drone se deplace sur cible
  drone_object.goto(GPS_target_delivery, distanceAccuracy)
  
  #Drone.move_servo(vehicle,10, False)
  #########Create new threads
  # myThread_Detection_target= myThread(1, "Thread_Detection_target",altitudeDeVol,drone_object,research_whiteSquare,saved_markers,detection_object)
  # myThread_asservissement= myThread(2, "Thread_asservissement",altitudeDeVol,drone_object,research_whiteSquare,saved_markers,detection_object)  

  #########debut de la Detection et du mouvement
  # myThread_Detection_target.start()
  # time.sleep(2)
  
  while drone_object.get_mode() == "GUIDED" or drone_object.get_mode() == "AUTO":
    # Asservissement control
    if drone_object.get_mode() == "GUIDED" :
      # print ("Condition GUIDED OK")
      last_errx, last_erry, errsumx, errsumy = asservissement(drone_object, detection_object, last_errx, last_erry, errsumx, errsumy)
  
  """# Ensure to stop a thread if the other is stopped
  # https://stackoverflow.com/questions/323972/is-there-any-way-to-kill-a-thread
  while not myThread_Detection_target.stopped() and not myThread_asservissement.stopped():
    if myThread_Detection_target.stopped():
      myThread_asservissement.stop()
    if myThread_asservissement.stopped():
      myThread_Detection_target.stop()"""

  # while drone_object.get_mode() == "GUIDED" or drone_object.get_mode() == "AUTO":

  
  #########attente de la fin de la Detection et du mouvement
  # myThread_Detection_target.join()
  
  if drone_object.get_mode() == "GUIDED" or drone_object.get_mode() == "AUTO" :  #securite pour ne pas que le drone reprenne la main en cas d interruption
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
  # Boolean variables
  global aruco_seen
  global good_aruco_found
  global white_square_seen

  # Counter variables
  global counter_no_detect
  global counter_white_square
  global counter_aruco

  global x_centerPixel_target
  global y_centerPixel_target
  global altitudeAuSol
  global id_to_test
  global saved_markers

  id_to_test = -1
  
  last_errx = 0
  last_erry = 0
  errsumx = 0
  errsumy = 0

  print("[mission] Mission3 started: Delivery at unknown location.")
  drone_object = Drone()    #permet de connecter le drone via dronekit en creant l objet drone
  detection_object = Detection(PiCamera(), id_to_find)  # creer l objet detection
  # self.f.write("[mission] Mission3 started: Delivery at unknown location.")
  #########verrouillage servomoteur et procedure arm and takeoff
  print("[mission] Launching and take off routine...")
  drone_object.lancement_decollage(5)
  
  #########passage en mode AUTO et debut de la mission
  drone_object.passage_mode_Auto()
  
  #########Create new threads
  # myThread_Detection_target= myThread(1, "Thread_Detection_target",altitudeDeVol,drone_object,research_whiteSquare,saved_markers,detection_object)
  # myThread_asservissement= myThread(2, "Thread_asservissement",altitudeDeVol,drone_object,research_whiteSquare,saved_markers,detection_object)  
  
  # a partir d'un certain waypoint declencher le thread de detection
  while drone_object.vehicle.commands.next <= 2 :
    pass
    
  #########debut de la Detection 
  # myThread_Detection_target.start()
  # time.sleep(2)

  while (drone_object.get_mode() == "GUIDED" or drone_object.get_mode() == "AUTO") or not package_dropped:
    # actualisation de l altitude et gps
    altitudeAuSol = drone_object.vehicle.rangefinder.distance
    altitudeRelative = drone_object.vehicle.location.global_relative_frame.alt
    longitude = drone_object.vehicle.location.global_relative_frame.lon
    latitude = drone_object.vehicle.location.global_relative_frame.lat
    heading = drone_object.vehicle.attitude.yaw
        
    #le srcipt Detection Target
    x_centerPixel_target, y_centerPixel_target, aruco_seen, good_aruco_found, white_square_seen, saved_markers = detection_object.Detection_aruco(latitude, longitude, altitudeAuSol, heading, saved_markers, id_to_test, True)
    # Asservissement control
    if drone_object.get_mode() == "GUIDED" :
      print ("Condition GUIDED OK")
      last_errx, last_erry, errsumx, errsumy = asservissement(drone_object, detection_object, last_errx, last_erry, errsumx, errsumy)
    
    if not drone_object.get_mode() == "GUIDED" and not drone_object.get_mode() == "AUTO":
      break
  
    #--------------- Case 1: Good ArUco ID found -----------------------
    if good_aruco_found:
      print("[detection] Case 1: Good ArUco ID found!")
      
      counter_no_detect = 0

      while drone_object.get_mode() != "GUIDED":
        drone_object.set_mode("GUIDED")
      
      # print("x_centerPixel_target : "+str(x_centerPixel_target))
      dist_center = math.sqrt((detection_object.x_imageCenter-x_centerPixel_target)**2+(detection_object.y_imageCenter-y_centerPixel_target)**2)
      print("[mission] Current distance: %.2fpx ; Altitude: %.2fm." % (dist_center, altitudeAuSol))
        
      if dist_center <= 50 and altitudeAuSol < 1.5 :  # condition pour faire le largage
        print("[mission] Largage !")
        drone_object.move_servo(10, True)
        time.sleep(0.5)
        package_dropped = True
        break

    #--------------- Case 2: Some white square seen --------------------
    elif white_square_seen:
      print("[detection] Case 2: Some white square seen.")

      counter_no_detect = 0
      counter_white_square += 1
      
      while drone_object.get_mode() != "GUIDED":
        drone_object.set_mode("GUIDED")
        
      print("[mission] Detection of 1 or many white squares (%i times)" % counter_white_square)

      # Check saved_ids in detection dictionary
      for saved_id in saved_markers :
        # Check boolean: if False, needs to be explored
        if saved_markers[saved_id][1] == False:
          id_to_test = saved_id
          print("[mission] Detection targetted towards id %s" % id_to_test)
          break

    #--------------- Case 3: ArUco tag seen but id false ---------------
    elif aruco_seen:
      print("[detection] Case 3: ArUco seen BUT not good ArUco found.")

      counter_no_detect = 0

      # Since it is not the good ArUco, continue the AUTO mission
      while drone_object.get_mode() != "AUTO" :
          drone_object.passage_mode_Auto()

      # Reset visual PID errors
      last_errx = 0
      last_erry = 0
      errsumx = 0
      errsumy = 0

      saved_markers[id_to_test] = (saved_markers[id_to_test][0], True)
      id_to_test = -1
      # print("[mission] Detection targetted towards id %s" % id_to_test)

    #--------------- Case 4: No detection of white or ArUco ------------
    else:
      print("[detection] Case 4: No detection of white or ArUco.")

      counter_no_detect += 1
      counter_white_square = 0
      
      # print ("COUCOU COUCOU COUCOU COUCOU COUCOU COUCOU COUCOU COUCOU")
      # print("[mission] No detection or wrong tag (%i times)" % compteur_no_detect)
      # print("[mission] compteur_whiteSquare (%i times)" % compteur_whiteSquare)
      
      if counter_no_detect > 5:
        print("[mission] 5 times without tag or white detection, not interesting place.")

        while drone_object.get_mode() != "AUTO" :
          drone_object.passage_mode_Auto()

          # Reset visual PID errors
          last_errx = 0
          last_erry = 0
          errsumx = 0
          errsumy = 0

          saved_markers[id_to_test] = (saved_markers[id_to_test][0], True)
          id_to_test = -1
      
    # print("compteur_aruco = "+str(compteur_aruco))
    # print("compteur_whiteSquare = "+str(compteur_whiteSquare))
    # print("compteur_no_detect = "+str(compteur_no_detect))
  
  
  
  ####### a faire declenchement detection au WP 2 
  ####### faire le declenchement asservissement en passant en guided en fonction d un parametre 
  ####### couper asserv en fonction d une condition et reprise auto
  ####### definir une fin largage ou echec et stopper code avec RTL
  
  # permet d'attendre la fin du thread
  # myThread_Detection_target.join()
  
  
  if drone_object.get_mode() == "GUIDED" or drone_object.get_mode() == "AUTO":  #securite pour ne pas que le drone reprenne la main en cas d interruption
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

if __name__ == "__main__":
  id_to_find = 25  # List of ids:
  GPS_target_delivery = LocationGlobalRelative(48.7068570, 7.7344260, altitudeDeVol)
  
  # Mission 1: Delivery at known location
  # mission_largage_GPS_connu(GPS_target_delivery, id_to_find)

  # Mission 2: Delivery at uncertain location 
  # mission_largage_GPS_incertain(GPS_target_delivery, id_to_find)

  # Mission 3: Delivery at unknown location 
  mission_largage_zone_inconnu(id_to_find)

  # Mission 4: Silent Delivery
  # mission_largage_GPS_silent(GPS_target_delivery, id_to_find)

  # Mission 5: Delivery on a moving pickup truck
  # Mission 6: Delivery far and fast
  
  print ("fin du code")
