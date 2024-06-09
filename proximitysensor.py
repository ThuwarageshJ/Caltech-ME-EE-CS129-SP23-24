"""
    Caltech ME/EE/CS129 - Experimental Robotics - Spring 23/24 
    Team Robo-TEd
    Contributors: Thuwaragesh Jayachandran, Edward Ju

    ProximitySensor class for handling the ultrasound sensors (left, middle, and right).

"""
import pigpio
import sys
import time
import traceback
from constants import *
import threading
import ctypes 
import numpy as np

# Ultrasound sensor class - handles single ultrasound sensor
class Ultrasound:

    # speed of sound calculated
    sound_speed=366

    def __init__(self, io, trig_pin, echo_pin):
        self.io=io
        self.trig_pin=trig_pin  # trigger pin
        self.echo_pin=echo_pin  # echo pin

        # Set up the two pins as output/input
        self.io.set_mode(trig_pin, pigpio.OUTPUT)
        self.io.set_mode(echo_pin, pigpio.INPUT)

        # Set up the callback functions
        self.cbrise = self.io.callback(self.echo_pin, pigpio.RISING_EDGE, self.rising)  # echo pin rising: signal sent
        self.cbfall = self.io.callback(self.echo_pin, pigpio.FALLING_EDGE, self.falling)    # echo pin failling: signal (echo) received back

        # Variables used for reading
        self.last_trig=time.time()     # latest trigger time
        self.trise=0                   # echo pin rising time
        self.distance=float('inf')      # latest known distance

        # Variable used for calculating moving average distances - NOT USED
        self.moving_average_distance=float('inf')   # latest known moving average distance - NOT USED 
        self.readings_sum=0       # moving sum
        self.readings_N=10      # no. of samples
        self.readings=[]       # list to store latest set of samples
    
    # Trigger the signal
    def trigger(self):
        if time.time()-self.last_trig>0.05:        # trigger only if the last trigger was more than 50 ms before
            if self.io is not None and self.io.connected:
                self.io.write(self.trig_pin, 1)     # initiate trigger
                self.io.write(self.trig_pin, 0)     # end trigger (trigger time >= 10 microseconds is ensured by the slowness of python)
                self.last_trig = time.time()        # update the latest trigger time
            else:
                print("GPIO interface is not connected.")

    # Callback for echo pin rising
    def rising(self, echo_pin, level, ticks):
        self.trise=ticks    # echo pin rise time: signal sent

    # Callback for echo pin falling
    def falling(self, echo_pin, level, ticks):

        deltatick=ticks-self.trise  # time of flight of the sound signal

        if (deltatick < 0):     # adjust: ticks is in mod 2^32
            deltatick += 2 ** 32
        
        self.distance=self.sound_speed*deltatick*10**(-6)/2 # update latest known distance in metres: ticks is in microseconds
        
        # calculate mocing average distance - NOT USED
        if len(self.readings)<=self.readings_N:
            if self.distance!=float('inf'):
                self.readings.append(self.distance)
                self.readings_sum +=self.distance
                self.moving_average_distance=(self.readings_sum/len(self.readings))
        else:
            first=self.readings.pop(0)
            self.readings_sum-=first
            self.readings.append(self.distance)
            self.readings_sum+=self.distance
            self.moving_average_distance=self.readings_sum/self.readings_N
    
    # Return latest known distance
    def read(self):
        return self.distance   

    # Return latest known moving averaged distance
    def read_moving_average(self):
        return self.moving_average_distance 

# Ultrasound sensor system class
class ProximitySensor:

    # Initiate ultrasound sensors and a thread to trigger the sensors every 50 ms.
    def __init__(self, io, trig_pins, echo_pins):

        self.io=io
        self.trig_pins=trig_pins
        self.echo_pins=echo_pins

        self.triggering = True             # flag for triggering thread
        self.last_trig=time.time()          # latest triggering time 

        # Setup each ultrasound sensor as parameters of this system class
        self.left_ultrasound = Ultrasound(self.io, self.trig_pins[0], self.echo_pins[0])   
        self.mid_ultrasound= Ultrasound(io, self.trig_pins[1], self.echo_pins[1])
        self.right_ultrasound =Ultrasound(io, self.trig_pins[2], self.echo_pins[2])

        # Initiate the triggering thread
        print("Starting triggering thread...")
        self.thread = threading.Thread(name="TriggerThread", target=self.run)
        self.thread.start()
        time.sleep(0.1) # Wait for the first measurements to arrive

    # Triggering thread function
    def run(self):  

        while self.triggering:          # run only if the flag is active
            self.trigger_ultrasounds()
            time.sleep(0.05)            # trigger every 50 ms
        print("I ended")
    
    # Function to terminate the triggering thread
    def shutdown(self):     

        self.triggering = False            # set the flag to false
        print("Waiting for triggering thread to finish...")
        self.thread.join()                  # join to main thread
        print("Triggering thread returned.")

    # Triggering function called by the triggering thread
    def trigger_ultrasounds(self):
        if time.time()-self.last_trig>0.05:     #extra safety: make sure 50 ms has elapsed after the last trigger

            # sequential triggering to avoid cross talks between the three ultrasound sensors
            self.left_ultrasound.trigger()
            time.sleep(0.01)
            self.mid_ultrasound.trigger()
            time.sleep(0.01)
            self.right_ultrasound.trigger()

            self.last_trig=time.time()
    
    # Return the latest known distances from all three sensors 
    def read_distances(self):  
        return (self.left_ultrasound.read(),self.mid_ultrasound.read(),self.right_ultrasound.read())

    # Return the latest known moving average distances from all three sensors 
    def read_moving_average_distances(self):
        return (self.left_ultrasound.read_moving_average(),self.mid_ultrasound.read_moving_average(),self.right_ultrasound.read_moving_average())

# Tester code to read distances from all three sensors
if __name__ == "__main__":

    # setup GPIOs
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    print("GPIO ready...")
    
    proximity_sensor=ProximitySensor(io, trig_pins, echo_pins)      # Proximity sensor object: initiates the triggering thread

    try:
        # Continually read distances - triggering executed by the triggering thread
        while True:
            distances = proximity_sensor.read_moving_average_distances()
            print("Distances = (%6.3fm, %6.3fm, %6.3fm)" % distances)

    except BaseException as ex:
        # Report the error, then continue with the normal shutdown
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    
    # terminate the thread and disable the ios
    proximity_sensor.shutdown()
    io.stop()