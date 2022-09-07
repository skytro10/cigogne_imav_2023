monDrone = Drone()
detection_object = Detection(PiCamera())

while True :
  altitudeAuSol = monDrone.vehicle.rangefinder.distance
  longitude = monDrone.vehicle.location.global_relative_frame.lon
  latitude = monDrone.vehicle.location.global_relative_frame.lat
  # heading = monDrone.vehicle
  heading = 180
  
  x_centerPixel_target, y_centerPixel_target, marker_found, whiteSquare_found, saved_markers = detection_object.Detection_aruco(latitude, longitude, altitudeAuSol, heading, True)
  print(saved_markers)
  # print( "alt : "+ str(altitudeAuSol))
  # measured_distance = detection_object.get_distance_image(x_centerPixel_target, y_centerPixel_target, altitudeAuSol)
  # print("distance :" + str(measured_distance))
