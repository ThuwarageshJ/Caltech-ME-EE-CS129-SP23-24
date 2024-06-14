"""
    Caltech ME/EE/CS129 - Experimental Robotics - Spring 23/24 
    Team Robo-TEd
    Contributors: Thuwaragesh Jayachandran, Edward Ju

    IRSensor class for handling the IR sensors as a system (Left, Middle, Right) for line detection

"""

import pigpio
import sys
import time
import traceback
from constants import *

# IR class to handle a single IR sensor
class IR:

    def __init__(self, pin, io):
        self.io=io
        self.pin=pin
        self.io.set_mode(self.pin, pigpio.INPUT)
    
    # Read the IR sensor output: 1=dark, 0=white
    def read(self):
        reading = self.io.read(self.pin)
        return reading 

# LineSensor class to handle all three IR sensors
class LineSensor:

    # Define left, middle, right IR sensors as parameters of the system class
    def __init__(self, IR_left_pin, IR_mid_pin, IR_right_pin, io):
        self.left_IR = IR(IR_left_pin,io)
        self.mid_IR = IR(IR_mid_pin,io)
        self.right_IR = IR(IR_right_pin,io)
    
    # Return the readings from all 3 sensors
    def read(self):
        irl = self.left_IR.read()
        irm = self.mid_IR.read()
        irr = self.right_IR.read()
        return (irl, irm, irr)

# Tester function 
if __name__ == "__main__":

    # Setup GPIOs
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    print("GPIO ready...")
    
    line_sensor=LineSensor(IR_left_pin, IR_mid_pin, IR_right_pin, io)       # Initialize line sensor

    try:
        while True:
            readings = line_sensor.read()
            print("IRs: L %d  M %d  R %d" % readings)
    except BaseException as ex:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # disable the ios
    io.stop()
