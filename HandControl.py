import dynamixel_sdk as dxlSDK
import sys, math, time

global deviceSerial = '/dev/ttyUSB0'
global B_Rate = 57600

"""
    Hardware Limits on Motors Due to mechanical constraints of the hand, the motors cannot reach their full range of motion.
    This function opens the hand by moving the motors to their maximum position values.
    
    Values for Maximum Pos of Each Finger (Open Hand)
    Thumb: 0
    Pointer: 2074
    Middle:1972
    Ring: 1974
    Pinky: 2475
    
    Values for Minimum Pos of Each Finger (Closed Fist)
    Thumb: 410
    Pointer: 2590
    Middle: 1537
    Ring: 2551
    Pinky: 1920
    
"""    




