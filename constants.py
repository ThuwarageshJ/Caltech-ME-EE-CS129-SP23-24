"""
    Caltech ME/EE/CS129 - Experimental Robotics - Spring 23/24 
    Team Robo-TEd
    Contributors: Thuwaragesh Jayachandran, Edward Ju

    File of global constants - pin numbers, thresholds, file paths
"""
import numpy as np
import os
from enum import Enum

# Enumerated Mode class to handle mode switches by the user
class Mode(Enum):

    EXPLORE =0      # Undirected exploration
    DRIVETO =1      # Driving to a goal, known or unknwon (directed exploration)
    STEP=2          # Stepping executions while in PAUSE
    MANUAL=3        # Manual driving by the user
    PAUSE=4         # Pause while in EXPLORE or DRIVETO
    SAVE=5          # Save current map 
    LOAD=6          # Load a map, existing or new
    CLEAR=7         # Clear blockages in the map

    def __eq__(self, other):
        return self.value == other.value
    
# magnetometer pins
PINS=[9,10,11,12,22,23,24,25]
LATCH=27
ADDRESS=4
READY=17

# PWM contants for the motors
max_pwm=255
pwm_frequency=1000

# motor pins
left_forward_leg = 7
left_reverse_leg = 8
right_forward_leg = 5
right_reverse_leg =6

# IR pins
IR_left_pin=14
IR_mid_pin=15
IR_right_pin=18

# ultrasound pins (0: Left ; 1: Middle ; 2: Right)
trig_pins=[13,19,26]
echo_pins=[16,20,21]

# Time constants for line following detectors 
T_linefollow=np.array([0.05,0.065,0.08])    # (0: Side detector ; 1: End-of-line detector ; 2: Intersection detector)
T_turn = 0.065                              # Next street detector

# Threshold value for line following detectors 
THRESHOLD_linefollow=[0.65,0.65,0.80]       # (0: Side detector ; 1: End-of-line detector ; 2: Intersection detector) 
THRESHOLD_turn=0.65                         # Next street detector

# Pull forward drive time
T_drive= 0.315                              

# Threshold distances for herding behavior
THRESHOLD_herd= 0.2

# Threshold distances for blockages   
THRESHOLD_checkahead=[0.6, 0.84]            # Detection before and after a drive (0: Cardinal directions ; 1: Diagonal directions)
THRESHOLD_obstacle = 0.18                   # Detection during drive 
THRESHOLD_clear = 0.25                      # Clearing detection during drive

# file path to maps (pickles)
cur_dir=os.path.dirname(__file__)
map_filenames=cur_dir+'/maps' 

# gradient, intercept values for proportional error feedback driving (PWM vs. Error) (L: Left, R: Right)
KL = -2.6309780184664358
KR = 3.073844547782905
CL = 0.7135718873442881
CR = 0.6820376325747144