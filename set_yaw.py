#!/usr/bin/env python

"""
set target attitude (yaw)
"""

import time
import sys
import math

from pymavlink import mavutil # Import mavutil
from pymavlink.quaternion import QuaternionBase # Imports for attitude

def send_manual_control(x,y,z,r):
    master.mav.manual_control_send(
        master.target_system,
        x,	  # -1000 to 1000, static=0, backward<0, forward>0
        y,    # -1000 to 1000, static=0, left<0, right>0
        z,    # 0 to 1000, static=500, downward<500, upward>500
        r,    # -1000 to 1000, static=0, anti-clockwise<0, clockwise>0
        0    # useless (for other purpose)
    )

def set_target_attitude(roll, pitch, yaw):
    """ Sets the target attitude while in depth-hold mode.

    'roll', 'pitch', and 'yaw' are angles in degrees.

    """
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
    )

# Create the connection
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

time.sleep(4)
send_manual_control(0,0,400,0)
time.sleep(0.5)
send_manual_control(0,0,500,0)

set_target_attitude(0,0,180)

try:
    for i in range(5): # time
        send_manual_control(0,0,500,0)  # prevent disarm
        time.sleep(1)
        
    # go for a spin
    # (set target yaw from 0 to 500 degrees in steps of 10, one update per second)
#    roll_angle = pitch_angle = 0
#    for yaw_angle in range(0, 500, 10):
#        set_target_attitude(roll_angle, pitch_angle, yaw_angle)
#        time.sleep(3) # wait for a second

    # spin the other way with 3x larger steps
#    for yaw_angle in range(500, 0, -30):
#        set_target_attitude(roll_angle, pitch_angle, yaw_angle)
#        time.sleep(1)

    # Disarm
    master.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    # Wait for disarm
    master.motors_disarmed_wait()
    print('Disarmed!')

except KeyboardInterrupt:
    # Disarm
    master.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    # Wait for disarm
    master.motors_disarmed_wait()
    print('Disarmed!')