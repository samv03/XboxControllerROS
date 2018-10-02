#!/usr/bin/env python
# Sam Vinella - samuel.vinella@mines.sdsmt.edu
# This code will take xbox 360/one controller input and split it into
# three 8 bit control strings sent to the robot using ROS networking.

# Assembling the commands:

# 8-bit binary command string 1
# 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
#
# Collection:
#   0 - (A button) -- Actuate Out
#   1 - (B button) -- Actuate In
#   2 - (Y button) -- Rotate Up
#   3 - (X button) -- Rotate Down
#
# Autonomy/Manual Control
#   4 - (Start button) -- Start Autonomy
#   5 - (Select button) -- Take Manual Control
#   6 - Empty
#   7 - Empty
#   

# 8-bit binary command string 2
# 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
#
# Movement:
#   0-3 - (Left Stick Y axis) - Throttle Rate
#   4-7 - (Left Stick X axis) - Turn Rate
#  

# 8-bit binary command string 3
# 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
#
# Collection/Dumping:
#   0-3 - (Right Trigger) -- Collect Material
#   4-7 - (Left Trigger) -- Dump Material
#
#

from inputs import get_gamepad, devices, DeviceManager
from std_msgs.msg import UInt8
import rospy
import time
import sys

controller_values = {
    # Analog Joysticks
    "ABS_X": 0,  # corresponds to LX in old code
    "ABS_Y": 0,  # corresponds to LY in old code
    "ABS_RX": 0,  # corresponds to RX in old code
    "ABS_RY": 0,  # corresponds to RY in old code

    # Analog Triggers
    "ABS_RZ": 0,  # corresponds to RT in old code
    "ABS_Z": 0,  # corresponds to LT in old code

    # Face Buttons
    "BTN_SOUTH": 0,  # corresponds to BA in old code
    "BTN_EAST": 0,  # corresponds to BB in old code
    "BTN_NORTH": 0,  # corresponds to BX in old code
    "BTN_WEST": 0,  # corresponds to BY in old code

    # Start, Select, and Menu
    "BTN_START": 0,  # corresponds to ST in old code
    "BTN_SELECT": 0,  # corresponds to SL in old code
    "BTN_MODE": 0,  # corresponds to MD in old code

    # Stick Buttons
    "BTN_THUMBR": 0,  # corresponds to RS in old code
    "BTN_THUMBL": 0,  # corresponds to LS in old code

    # Bumpers
    "BTN_TR": 0,  # corresponds to RB in old code
    "BTN_TL": 0,  # corresponds to LB in old code

    # Directional Pad (D-Pad)
    # These values are exceptions to the other cases...
    "DU": 0,
    "DD": 0,
    "DL": 0,
    "DR": 0
}

### BEGIN MAIN ###

_oldABS_X = 0
_oldABS_Y = 0

dead_band_width = 2000
dead_band_slope = 32768.0/(32768 - (dead_band_width - 50))
dead_band_y_intercept = dead_band_slope * (dead_band_width - 50) * -1

def mapDeadBand(value):
    new_value = abs(value)
    #if value is in deadband, return zero
    if new_value < dead_band_width:
        return 0

    #otherwise map to linear function 
    else:
        new_value = dead_band_slope * new_value + dead_band_y_intercept

    if value < 0:
        return int(new_value*-1)
    else:
        return int(new_value)
    

pub1 = rospy.Publisher('controller_input1', UInt8, queue_size=10)
pub2 = rospy.Publisher('controller_input2', UInt8, queue_size=10)
pub3 = rospy.Publisher('controller_input3', UInt8, queue_size=10)


while True:
    try:

        # Clear values, otherwise values can get stuck
        for x in range(0, 21):
            controller_values[x] = 0

        events = get_gamepad()
        for event in events:
            controller_values[event.code] = event.state

        rospy.init_node('xbox_control', anonymous=True)
        control_string_1 = (controller_values["BTN_SOUTH"] << 7) + (controller_values["BTN_EAST"] << 6) + (controller_values["BTN_NORTH"] << 5) + (controller_values["BTN_WEST"] << 4) + (controller_values["BTN_START"] << 3) + (controller_values["BTN_SELECT"] << 2)

        control_string_3 = ((controller_values["ABS_RZ"] / 64) << 4) + ((controller_values["ABS_Z"] / 64) & 0b00001111)

        
        control_val_ABS_X = (((mapDeadBand(controller_values["ABS_X"]) + 32768) >> 12) & 0b00001111 )
        control_val_ABS_Y = ((((mapDeadBand(controller_values["ABS_Y"]) + 32768) >> 12) & 0b00001111) << 4)

        #check if the new values are the same as the old to save bandwidth
        if control_val_ABS_X != _oldABS_X or control_val_ABS_Y != _oldABS_Y:
            control_string_2 = control_val_ABS_X + control_val_ABS_Y
            pub2.publish(control_string_2)
        
            print "Sent X: " + str(control_val_ABS_X) + " Sent Y: " + str(control_val_ABS_Y>>4)

            _oldABS_X = control_val_ABS_X
            _oldABS_Y = control_val_ABS_Y
        

        pub1.publish(control_string_1)
        pub3.publish(control_string_3)

    except KeyboardInterrupt:
        sys.exit()
