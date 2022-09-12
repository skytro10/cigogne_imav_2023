from commande_drone import Drone
from dronekit import LocationGlobalRelative
drone_object = Drone()

drone_object.list_fence()
time.sleep(1)

drone_object.load_fence()
time.sleep(1)

while True:
    line = ""
    for c in raw_input():
        line = line + c
    print("%s" % line)
    if line == "exit":
        break
