"""
    Caltech ME/EE/CS129 - Experimental Robotics - Spring 23/24 
    Team Robo-TEd
    Contributors: Thuwaragesh Jayachandran, Edward Ju

    AngleSensor class for handling the magnetometer, calibrate it, and read angles (relative to the geographic North).

"""
import pigpio
import sys
import time
import traceback
import numpy as np
import matplotlib.pyplot as plt
from drivesystem import DriveSystem
from constants import *

# Storage for plotting magnetometer readings after adjustments ([-1,+1])
M1=[]
M2=[]

# Analog to digital converter
class ADC:

    def __init__(self, io): 
        self.io = io
        self.io.set_mode(LATCH, pigpio.OUTPUT) # LATCH 
        self.io.set_mode(ADDRESS, pigpio.OUTPUT) # ADDRESS 
        self.io.set_mode(READY, pigpio.INPUT) # READY 
        for i in range(8):  
            self.io.set_mode(PINS[i], pigpio.INPUT) # PINS to read the digital 8-bit integer
    
    # Read the 8-bit value from a specified channel in the magnetometer.
    def read(self, channel):

        # Take the reading
        self.io.write(LATCH,0)  # activate (Analog to Digital) conversion
        self.io.write(ADDRESS,channel)  # set channel
        self.io.write(LATCH,1)  # stop conversion
        self.io.write(LATCH,0)  # activate conversion
        self.io.write(LATCH,1)  # stop conversion
        while True:             
            if self.io.read(READY):    # wait for signal from READY
                break
        
        # Parse the reading
        reading=0
        for i in range(8):
            reading=(reading<<1)+ self.io.read(PINS[7-i])   # read from MSB to LSB
        
        # Return the 8-bit integer (ideally, 0-255)
        return reading  

# High level class for magnetometer
class AngleSensor:

    # Current max-min values output by the magnetometers (for scaling)
    maxes=[171, 199]
    mins=[62, 86]

    # Storage to keep track of max-min values output by magnetometers: needed for calibration 
    new_maxes=[0,0]
    new_mins=[1000,1000]

    # Initialize the ADC object
    def __init__(self, io):
        self.ADC = ADC(io)

    # Parse the 8-bit integer from both the channels to the angle relative to the geographic North
    def read_angle(self):

        global M1,M2 # storage for plotting during calibration

        # Read the 8-bit values from both the channels
        raw=[self.ADC.read(0),self.ADC.read(1)]    

        # Scale the values to [-1,+1]
        m1 = 2*(raw[0]-self.mins[0])/(self.maxes[0]-self.mins[0]) - 1
        m2 = 2*(raw[1]-self.mins[1])/(self.maxes[1]-self.mins[1]) - 1

        # For plotting during calibration
        M1.append(m1)
        M2.append(m2)

        # Update new maxes and mins
        for i in range (2):
            self.new_maxes[i]=max(self.new_maxes[i], raw[i])
            self.new_mins[i]=min(self.new_mins[i], raw[i])

        theta= 180*np.arctan2(m1, m2)/np.pi #convert to degrees

        return theta

# Function for live plotting while calibrating
def visualize():

    plt.plot(M1)
    plt.plot(M2)
    plt.ylabel("angle")
    plt.xlabel("time")
    plt.pause(.0001)

# Calibrate the magnetometer   
if __name__ == "__main__":

    # Setup GPIOs
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    print("GPIO ready...")

    angle_sensor= AngleSensor(io)      # Magnetometer object
    bot=DriveSystem(left_forward_leg, left_reverse_leg, right_forward_leg, right_reverse_leg,io, max_pwm, pwm_frequency) # Drivesystem object to spin the bot for calibration

    t0=time.time()
    t=t0

    try:
        while t-t0<10:
            bot.drive("left", "spin")   # spin the bot to the left for 10 seconds to calibrate
            angle=angle_sensor.read_angle() # angle relative to the geographic North in degrees
            t=time.time()
            print("I'm oriented ", angle, " relative to the North and my heading is ", round((angle/45))%8)
            visualize()

    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Update (Manually) angle_sensor.maxes and .mins to these new values after calibration
    print('new maxes are ', angle_sensor.new_maxes)
    print('new mins are ', angle_sensor.new_mins)
    
    # stop motors and disable the ios
    bot.stop()
    io.stop()
    

   
