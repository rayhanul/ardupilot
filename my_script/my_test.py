

import time 

from pymavlink import mavutil

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()
# Get some information !
while True:
    try:
        print(master.recv_match().to_dict())
    except:
        pass
    time.sleep(0.1)