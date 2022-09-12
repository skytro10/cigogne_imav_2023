from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil, mavwp
import time
import json


class FenceTest:

    def __init__(self):
        self.last_fence_breach = 0
        self.last_fence_status = 0
        self.present = False
        self.enabled = False
        self.healthy = True
        self.have_list = False
        self.vehicle = None

        self.fenceloader = None
        self.fenceloader = mavwp.MAVFenceLoader()
        self.fenceloader.clear()

        self.is_listing_fence = False
        self.current_fence_point = None
        self.sending_fence_point = None



    def connect_vehicle(self, connection_string):
        print "Connecting to vehicle on: %s" % connection_string     #= "127.0.0.1:14550"        
        self.vehicle = connect(connection_string, wait_ready=False, status_printer=self.on_status_message)
        self.vehicle.wait_ready("autopilot_version")

        while not self.vehicle.home_location:
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()
            if not self.vehicle.home_location:
                print "Waiting for home location ..."
                time.sleep(2)

        # listen
        self.vehicle.add_message_listener("FENCE_POINT", self.on_fence_point)
        self.vehicle.add_message_listener("FENCE_STATUS", self.on_fence_status)
        
        print "Firmware version    %s" % self.vehicle.version
        print "Flight Mode         %s" % self.vehicle.mode.name
        print "Home Location       %s" % self.vehicle.home_location
        print "Param FENCE_TYPE    %s" % self.vehicle.parameters[ "FENCE_TYPE"    ]
        print "Param FENCE_TOTAL   %s" % self.vehicle.parameters[ "FENCE_TOTAL"   ]
        print "Param FENCE_RADIUS  %s" % self.vehicle.parameters[ "FENCE_RADIUS"  ]
        print "Param FENCE_MARGIN  %s" % self.vehicle.parameters[ "FENCE_MARGIN"  ]
        print "Param FENCE_ENABLE  %s" % self.vehicle.parameters[ "FENCE_ENABLE"  ]
        print "Param FENCE_ALT_MAX %s" % self.vehicle.parameters[ "FENCE_ALT_MAX" ]
        print "Param FENCE_ACTION  %s" % self.vehicle.parameters[ "FENCE_ACTION"  ]


    def fence_enable(self, enabled):
        msg = self.vehicle.message_factory.command_long_encode(
            0,          # target_system
            0,          # target_component
            mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, #command
            0,          # confirmation
            enabled,    # param 1 enabled 1, disable 0
            0,0,0,0,0,0 # param 2 ~ 7
        )
        # send command to vehicle
        print "Sending fence enable %s ..." % enabled
        self.vehicle.send_mavlink(msg)


    def on_status_message(self, vehi, msg_name, msg):
        print("%s %s" % (msg_name, msg))


    def on_fence_status(self, vehi, msg_name, msg):
        if msg.breach_status == 1:
            print("%s %s" % (msg_name, msg))


    def on_fence_point(self, vehi, msg_name, msg):
        if self.is_listing_fence:
            '''is listing fence points'''
            print("Listing %s" % (msg))
            self.fenceloader.add(msg)
        else:
            '''is sending fence points'''
            print("Sending %s" % (msg))
            self.sending_fence_point = msg
            if self.sending_fence_point is None:
                print "Failed to send fence point %s" % msg
            elif (self.current_fence_point.idx != self.sending_fence_point.idx or 
                abs(self.current_fence_point.lat - self.sending_fence_point.lat) >= 0.00003 or 
                abs(self.current_fence_point.lng - self.sending_fence_point.lng) >= 0.00003):
                print("Failed to send fence point %u" % msg)


    def fetch_fence_point(self, i):
        '''fetch one fence point'''
        self.vehicle._master.mav.fence_fetch_point_send( 0,0, i )
        tstart = time.time()
        while time.time() - tstart < 1:
            time.sleep(0.1)
            continue


    def list_fence(self):
        self.is_listing_fence = True
        count = int( self.vehicle.parameters["FENCE_TOTAL"] )
        if count == 0:
            print("No geo-fence points")
            return
        for i in range(count):
            self.fetch_fence_point(i)


    def load_fence(self):
        self.fenceloader.target_system = 0
        self.fenceloader.target_component = 0
        self.fenceloader.clear()
        
        # test datasets, geofence coordinates
        x = '[ { "lat": -20.079674, "lng": -47.416439 }, { "lat": -20.080217, "lng": -47.417088 }, { "lat": -20.079044, "lng": -47.417282 }, { "lat": -20.079683, "lng": -47.415550 }, { "lat": -20.081003, "lng": -47.416229 }, { "lat": -20.080217, "lng": -47.417088 } ]'
        y = '[ { "lat": -20.079439, "lng": -47.417011 }, { "lat": -20.079420, "lng": -47.417278 }, { "lat": -20.079643, "lng": -47.416950 }, { "lat": -20.079399, "lng": -47.416683 }, { "lat": -20.079206, "lng": -47.417000 }, { "lat": -20.079420, "lng": -47.417278 } ]'

        fence = json.loads(y)

        for i in range(len(fence)):
            point = fence[i]
            lat = point["lat"]
            lng = point["lng"]
            self.fenceloader.add_latlon(lat, lng)
            print("Loading point lat=%s lng=%s" % (lat, lng))

        self.send_fence()



    def send_fence(self):
        self.is_listing_fence = False 
        '''send fence points from fenceloader'''

        # must disable geo-fencing when loading
        self.fenceloader.target_system = 0
        self.fenceloader.target_component = 0
        self.fenceloader.reindex()
        
        self.vehicle.parameters["FENCE_ACTION"] = mavutil.mavlink.FENCE_ACTION_NONE # your action here
        self.vehicle.parameters["FENCE_TOTAL" ] = self.fenceloader.count()

        for i in range(self.fenceloader.count()):
            self.current_fence_point = self.fenceloader.point(i)
            self.vehicle._master.mav.send( self.current_fence_point )
            self.fetch_fence_point(i)


    def clear_fence(self):
        self.vehicle.parameters["FENCE_TOTAL" ] = 0


    def disconnect_vehicle(self):
        self.vehicle.close()
        print "Completed"



test = FenceTest()

test.connect_vehicle("127.0.0.1:14660")
time.sleep(2)

# test.list_fence()
# time.sleep(1)

# test.load_fence()
# time.sleep(1)

# FENCE_STATUS 
# breach_status : 1        # 0 inside fence, 1 outside fence
# breach_count  : 3        # how many breaches during the flight
# breach_type   : 3        # 0=No last fence breach, 1=Breached min alt, 2=Breached max alt, 3=Breached fence boundary
# breach_time   : 24542345 # time for the last breach

while True:
    line = ""
    for c in raw_input():
        line = line + c
    print("%s" % line)
    if line == "exit":
        break

test.disconnect_vehicle()




