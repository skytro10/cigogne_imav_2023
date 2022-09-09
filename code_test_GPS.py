from commande_drone import Drone
from detection_target import Detection
from picamera import PiCamera
from dronekit import LocationGlobalRelative
from math import degrees

monDrone = Drone()
detection_object = Detection(PiCamera(), 25)
saved_markers = {-1: (monDrone.vehicle.location.global_relative_frame, True)}

while True :
  altitudeAuSol = monDrone.vehicle.rangefinder.distance
  longitude = monDrone.vehicle.location.global_relative_frame.lon
  latitude = monDrone.vehicle.location.global_relative_frame.lat
  heading = degrees(monDrone.vehicle.attitude.yaw)
  print("Heading:", heading)
  print("Altitude:", altitudeAuSol)
  x_centerPixel_target, y_centerPixel_target, marker_found, whiteSquare_found, saved_markers = detection_object.Detection_aruco(latitude, longitude, altitudeAuSol, heading, saved_markers, -1, True)
  # print(altitudeAuSol)
  print("--- Markers dictionary --- ")
  # for ids in saved_markers:
  #  print(ids, len(saved_markers[ids]))
  print(saved_markers)
  # print( "alt : "+ str(altitudeAuSol))
  # measured_distance = detection_object.get_distance_image(x_centerPixel_target, y_centerPixel_target, altitudeAuSol)
  # print("distance :" + str(measured_distance))
