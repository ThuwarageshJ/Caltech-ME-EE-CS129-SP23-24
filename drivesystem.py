"""
    Drivesystem class and Motor class. Drivesystem controls the system of motors, and Motor controls single motors.
"""


import pigpio
import sys
import time
import traceback
from constants import *

class Motor:
    def __init__(self, forward_leg, reverse_leg, io, max_pwm, pwm_frequency): 
        self.max_pwm=max_pwm
        self.pwm_frequency=pwm_frequency
        self.io = io
        self.forward_leg = forward_leg
        self.reverse_leg = reverse_leg
        #set pins to output
        self.io.set_mode(self.forward_leg, pigpio.OUTPUT)
        self.io.set_mode(self.reverse_leg, pigpio.OUTPUT)
        #set max PWM
        self.io.set_PWM_range(self.forward_leg, self.max_pwm)
        self.io.set_PWM_range(self.reverse_leg, self.max_pwm)
        #set duty cycle
        self.io.set_PWM_frequency(self.forward_leg, self.pwm_frequency)
        self.io.set_PWM_frequency(self.reverse_leg, self.pwm_frequency)
        #clear pins
        self.stop()

    def setlevel(self,level):
        if level>0:
            self.io.set_PWM_dutycycle(self.forward_leg, self.max_pwm*level)
            self.io.set_PWM_dutycycle(self.reverse_leg, 0)
        elif level<0:
            self.io.set_PWM_dutycycle(self.forward_leg, 0)
            self.io.set_PWM_dutycycle(self.reverse_leg, self.max_pwm*abs(level))
        else:
            self.io.set_PWM_dutycycle(self.forward_leg, 0)
            self.io.set_PWM_dutycycle(self.reverse_leg, 0)
    
    def stop(self):
        self.setlevel(0)

class DriveSystem:
    #drive modes
    l=0.71
    r=0.75

    left_drives={
    "straight":(1.05*l,0.95*r),
    "veer":(0.95*l,1.03*r),
    "steer":(0.83*l,1.00*r),
    "turn":(0.74*l,1.18*r),
    "hook":(0,1.01*r),
    "spin":(-1.05*l,0.93*r),
    "reverse":(-1.03*l,-0.95*r)
    }
    right_drives={
    "straight":(1.05*l,0.95*r),
    "veer":(1.10*l,0.85*r),
    "steer":(1.15*l,0.75*r),
    "turn":(1.25*l,0.64*r),
    "hook":(1.09*l,0),
    "spin":(0.91*l,-0.94*r),
    "reverse":(-1.03*l,-0.95*r)
    }

    def __init__(self, left_forward_leg, left_reverse_leg, right_forward_leg, right_reverse_leg, io, max_pwm, pwm_frequency):
        self.left_motor=Motor(left_forward_leg, left_reverse_leg,io, max_pwm, pwm_frequency)
        self.right_motor=Motor(right_forward_leg, right_reverse_leg, io, max_pwm, pwm_frequency)

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def drive(self,direction,drive_mode):
        if direction=="left":
            self.left_motor.setlevel(self.left_drives[drive_mode][0])
            self.right_motor.setlevel(self.left_drives[drive_mode][1])
        elif direction=="right":
            self.left_motor.setlevel(self.right_drives[drive_mode][0])
            self.right_motor.setlevel(self.right_drives[drive_mode][1])
        else: 
            print("Incorrect driving mode!") 
    
    def pwm(self, left_pwm, right_pwm):
        self.left_motor.setlevel(left_pwm)
        self.right_motor.setlevel(right_pwm)

def flowerpower(bot):   #Tester function - Executes all the defined driving modes for 4 seconds
    input("enter")
    for direction in ["right","left"]:
        for drive_mode in ["straight", "veer", "steer", "turn", "hook", "spin"]:
            print(direction, drive_mode)
            bot.drive(direction, drive_mode)
            time.sleep(4)
            bot.stop()
            input("enter")

if __name__ == "__main__":
    #setup GPIOs
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    print("GPIO ready...")

    #initiate drivesystem
    bot=DriveSystem(left_forward_leg, left_reverse_leg, right_forward_leg, right_reverse_leg,io, max_pwm, pwm_frequency)

    #actual code
    try:
        #the flower power test
        flowerpower(bot)
        bot.drive("left","spin")
        time.sleep(4)
        bot.drive("right","spin")
        time.sleep(4)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    #stop motors and disable the ios
    bot.stop()
    io.stop()
