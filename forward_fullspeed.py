#!/usr/bin/env python

"""
altitude hold mode, full thrust, forward
"""

import time
import sys
import math

from pymavlink import mavutil

def send_manual_control(x,y,z,r):
    master.mav.manual_control_send(
        master.target_system,
        x,	  # -1000 to 1000, static=0, backward<0, forward>0
        y,    # -1000 to 1000, static=0, left<0, right>0
        z,    # 0 to 1000, static=500, downward<500, upward>500
        r,    # -1000 to 1000, static=0, anti-clockwise<0, clockwise>0
        0    # useless (for other purpose)
    )



### Start program ###

# Create the connectionc
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Arm
master.arducopter_arm()
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

# Choose a mode
mode = 'ALT_HOLD'
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)

# set depth
time.sleep(2)
send_manual_control(0,0,400,0) # 20% downward force 
time.sleep(0.5)    # change param: maybe 0.5(s) -> 20cm

# forward
t = time.time()
while (time.time() - t == 10):   # change param: (second)
    send_manual_control(1000,0,500,0)

# Disarm
time.sleep(3)
master.arducopter_disarm()
print("Waiting for the vehicle to disarm")
# Wait for disarm
master.motors_disarmed_wait()
print('Disarmed!')