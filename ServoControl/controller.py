#!/usr/bin/env python
"""
Simple PhidgetAdvancedServo Controller
For CS4222 Group Project



"""

__author__ = 'Kangwei'


#Basic imports
import subprocess
from ctypes import *
import os, sys, traceback, time, string
from time import sleep

try:
    os.environ['TOSROOT']
except KeyError:
    print("\tNo Environment Variable defined for TinyOS directory")
    print("\tYou need the defined environment variable $TOSROOT to proceed")
    print("\tInstructions:")
    print("\t\t1. Type echo $TOSROOT in any terminal (without root)")
    print("\t\t2. The directory should be printed e.g. /opt/tiny-os-2.1.1 ")
    print("\t\t3. As root user, enter export TOSROOT=<directory from step 2>")
    print("\t\t4. Run this program as Root")
    sys.exit(1)

if os.getenv("SUDO_USER") is None:
    print("Privilege Error: Must run as root!")
    sys.exit(1)    

#Phidget specific imports
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import AttachEventArgs, DetachEventArgs, ErrorEventArgs, CurrentChangeEventArgs, PositionChangeEventArgs, VelocityChangeEventArgs
from Phidgets.Devices.AdvancedServo import AdvancedServo
from Phidgets.Devices.Servo import ServoTypes

import listener


__all__ = ["Controller", "windMax", "windMin", "disengage", "close"]


class Controller():
    """Implements Phidget Controller"""
    def __init__(self):
        try:
            self.advancedServo = AdvancedServo()
            self.advancedServo.openPhidget()
            print("Waiting to be attached")
            self.advancedServo.waitForAttach(10000)
            print("Initialize Servo")
            self.advancedServo.setVelocityLimit(0, 180.00)
            self.advancedServo.setServoType(0, ServoTypes.PHIDGET_SERVO_HITEC_HS322HD)
            print("Engage Servo")
            self.advancedServo.setEngaged(0, True)
            print("Ready")
        except RuntimeError as e:
            print("Runtime Exception: %s" % e.details)
            print("Exiting....")
            exit(1)

   
    def windMax(self):
        self.advancedServo.setPosition(0, self.advancedServo.getPositionMax(0))

    def windMin(self):
        self.advancedServo.setPosition(0, self.advancedServo.getPositionMin(0))
        sleep(3)


    def disengage(self):
        self.advancedServo.setEngaged(0, False)
        sleep(3)

    def close(self):
        self.advancedServo.closePhidget()
            
            
        
        
def main(*args):
    control = Controller()
    control.windMin()
    sleep(2)
    control.windMax()
    sleep(2)

    # Shutdown operations
    control.disengage()
    control.close()
    exit(0)

if __name__=='__main__':
    main(*sys.argv)
        
        
    

    

    
