#Basic imports
from ctypes import *
import sys
from time import sleep
#Phidget specific imports
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import AttachEventArgs, DetachEventArgs, ErrorEventArgs, CurrentChangeEventArgs, PositionChangeEventArgs, VelocityChangeEventArgs
from Phidgets.Devices.AdvancedServo import AdvancedServo
from Phidgets.Devices.Servo import ServoTypes


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
        
        
    

    

    
