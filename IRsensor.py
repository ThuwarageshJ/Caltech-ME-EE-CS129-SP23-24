import pigpio
import sys
import time
import traceback
from constants import *

class IR:
    def __init__(self, pin, io):
        self.io=io
        self.pin=pin
        self.io.set_mode(self.pin, pigpio.INPUT)
    
    def read(self):
        reading = self.io.read(self.pin)
        return reading 

class LineSensor:
    def __init__(self, IR_left_pin, IR_mid_pin, IR_right_pin, io):
        self.left_IR = IR(IR_left_pin,io)
        self.mid_IR = IR(IR_mid_pin,io)
        self.right_IR = IR(IR_right_pin,io)
    
    def read(self):
        irl = self.left_IR.read()
        irm = self.mid_IR.read()
        irr = self.right_IR.read()
        return (irl, irm, irr)
    
if __name__ == "__main__":

    #setup GPIOs
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    print("GPIO ready...")
    
    #initialize line sensor
    line_sensor=LineSensor(IR_left_pin, IR_mid_pin, IR_right_pin, io)

    #main code
    try:
        while True:
            readings = line_sensor.read()
            print("IRs: L %d  M %d  R %d" % readings)
    except BaseException as ex:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    #disable the ios
    io.stop()
