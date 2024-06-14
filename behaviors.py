"""
    Caltech ME/EE/CS129 - Experimental Robotics - Spring 23/24 
    Team Robo-TEd
    Contributors: Thuwaragesh Jayachandran, Edward Ju

    Definitions of behavior functions ( Line/Tunnel follow, Turning at interesections, Pullforward at intersection ) 
    and the raw value calculators for the associated detectors

"""
from constants import *
from drivesystem import DriveSystem
from anglesensor import AngleSensor
from linesensor import LineSensor
from proximitysensor import ProximitySensor
import time

# Calculate raw values for line following detectors: (0: Side ; 1: End-of-line ; 2: Intersection)
def calculate_raw_linefollow(readings):

    raw=[0,0,0]             # default raw value

    if readings==(0,0,0):       # end-of-line    
        raw[1]=1
    elif readings==(1,1,1):     # intersection
        raw[2]=1
    elif readings==(0,1,1):     # side: slightly pushed to the left
        raw[0]=0.5
    elif readings==(0,0,1):     # side: pushed to the left
        raw[0]=1
    elif readings==(1,0,0):     # side: pushed to the right
        raw[0]=-1
    elif readings==(1,1,0):     # side: slightly pushed to the right
        raw[0]=-0.5

    return raw

# Calculate raw values for pull forward detector
def calculate_raw_midsensor(readings):
    
    raw=0                   # default

    if readings[1]==1:      # street ahead
        raw=1

    return raw

# Calculte raw values for turning detector
def calculate_raw_turn(readings, direction):
    
    raw=0                  # default

    if direction== "right":                         # right turn- use right and middle IR sensor readings
        if readings[1]==1 and readings[2]==1:
            raw=1
        elif readings[2]==1:
            raw=0.5
    elif direction== "left":                        # left turn- use left and middle IR sensor readings
        if readings[1]==1 and readings[0]==1:
            raw=1
        elif readings[0]==1:
            raw=0.5

    return raw

# Check for blockages in front before and after a drive
def check_ahead(proximity_sensor, heading):

    distances = proximity_sensor.read_distances()

    if distances[1] < THRESHOLD_checkahead[heading%2]:     # threshold differs for cardinal and diagonal directions  
        return "OBSTACLE"
    
    return "CLEAR"

# Turning behavior: Turn the bot until a new street is found
def turn(bot,line_sensor,angle_sensor, direction, tunnel = False):   
    
    # if tunnel:
    #     return 0
    
    # Initialize detector parameters
    t=time.time()
    level=0.5                           # initial (assumed) level for turning detector
    offroad=False                       # flag to check if the bot left the current street completely

    angle_i=angle_sensor.read_angle()   # initial angle

    bot.drive(direction,"spin")         # spin the bot in the specified direction

    # Spin until the next street is detected
    while True:

        # dt calculation for detector
        tlast=t
        t=time.time()
        dt=t-tlast

        # Calculate raw values and update detector level using IR sensor readings
        readings = line_sensor.read()  
        raw = calculate_raw_turn(readings, direction) 
        level = level + dt* (raw - level)/T_turn

        if level>THRESHOLD_turn and offroad:    # found the new street

            bot.stop()                          # stop the bot from spinning
            angle_f=angle_sensor.read_angle()   # final angle
            turn_angle = (angle_f-angle_i)      # angle of turning

            return turn_angle 
         
        elif level<1-THRESHOLD_turn:            # exited the current street
            offroad=True
        
# Pullforward behavior: Drive straight after reaching an intersection to facilitate proper turning behavior
def pullforward(bot, line_sensor):

    # Initialize detector parameters
    t0=time.time()
    level=0         # initial (assumed) level for turning detector
    t=t0

    # Drive straight for a time of T_drive
    while True:

        # dt calculation for detector
        tlast=t
        t=time.time()
        dt=t-tlast

        # Calculate raw values and update detector level using IR sensor readings
        readings = line_sensor.read()               
        raw = calculate_raw_midsensor(readings)
        level = level + dt* (raw - level)/T_linefollow[1]

        if t < t0 + T_drive:                    
            bot.drive("left","straight")            
        else:                               # T_drive elapsed
            bot.stop()  
            if level>THRESHOLD_linefollow[1]:   # detected a street ahead (according to the pullforward detector)   
                return "STREET"
            else:                               # no street ahead
                return "NOSTREET"

# Line/Tunnelfollow behavior: drive the bot by following the black line, or by positioning in the middle of two side walls
def linefollow(bot,line_sensor, proximity_sensor):

    # Initialize detector parameters
    t=time.time()
    level=np.array([0,0.1,0])           # inital (assumed) levels for line following detectors:(0: Side ; 1: End-of-line ; 2: Intersection)
    tunnel = False                      # flag to indicate whether the bot is/ was in a tunnel: to ensure no turning in the tunnel

    while True:

        # dt calculation for detectors
        tlast=t
        t=time.time()
        dt=t-tlast

        # Readings from IR and ultrasound sensors
        readings = line_sensor.read()
        distances = proximity_sensor.read_distances()

        # Obstacle found while in motion- stop the bot and return
        if distances[1]<THRESHOLD_obstacle:       
            bot.stop()          
            return ("OBSTACLE", tunnel)

        # Updating the detectors
        if not(readings==(0,0,0) and abs(level[0])>THRESHOLD_linefollow[0]):    # Do not update if the bot is pushed to the side
            raw = calculate_raw_linefollow(readings) 
            level = level + dt* (raw - level)/T_linefollow

        if readings==(0,1,0):                               # well aligned with lines: straight drive
            bot.drive("left","straight")
        elif readings ==(0,1,1):                            # slightly to left: soft right drive to get back to line
            bot.drive("right","turn")
        elif readings ==(0,0,1):                            # to left: hard right drive to get back to line
            bot.drive("right","hook")
        elif readings ==(1,1,0):                            # slightly to right: soft left drive to get back to line
            bot.drive("left","turn")
        elif readings ==(1,0,0):                            # to right: hard left drive to get back to line
            bot.drive("left","hook")
        elif readings ==(0,0,0):
            
            if level[0]>THRESHOLD_linefollow[0]:            # pushed to the left: very strong right drive to get back to line
                bot.drive("right","spin")
            elif level[0]<(-1)*THRESHOLD_linefollow[0]:     # pushed to the right: very strong left drive to get back to line
                bot.drive("left","spin")
            elif level[1]>THRESHOLD_linefollow[1]:          # reached end of line

                if distances[0]+distances[2]<Tunnel_width or tunnel: # is/ was in a tunnel: follow the tunnel using ultrasound sensors 
                    
                    error = (distances[0]-distances[2])     # lateral deviation from the centre of the tunnel
                    
                    # PWM calculation for proportional error feedback based (custom) drive
                    PWM_L = CL + KL * error
                    PWM_R = CR + KR * error   
                    if PWM_L >= 1 or PWM_R >= 1 or PWM_L <= -1 or PWM_R <= -1:   # invalid PWM values
                        continue   
                    
                    bot.pwm(PWM_L, PWM_R)                   # drive according to the custom PWM values 
                    tunnel = True                           # in tunnel

                else:                                       # reached end of line
                    bot.stop()
                    return "END"
                
            elif level[1]<1-THRESHOLD_linefollow[1]:        # potholes: ignore and drive straight
                bot.drive("left","straight")

        elif readings ==(1,1,1):                            # all sensors on the black line

            if level[2]>THRESHOLD_linefollow[2]:            # detected an intersection
                tunnel = tunnel and distances[0]+distances[2]<Tunnel_width   # flag to decide whether this intersection is inside a tunnel
                bot.stop()
                return ("INTERSECTION", tunnel)   
            elif level[2]<1-THRESHOLD_linefollow[2]:        # not an intersection
                bot.drive("left","straight") 

        elif readings ==(1,0,1):                            # impossible case
            pass
            # if level[2]>THRESHOLD_linefollow[2]:            
            #     if distances[0]+distances[2]<0.5:
            #         tunnel = True
            #     bot.stop()
            #     return ("INTERSECTION", tunnel) 

# Smart-line/tunnel follow behavior; high level behavior to handle linefollow() and properly handle obstacles detected while driving
def smartlinefollow(bot,line_sensor, proximity_sensor, angle_sensor):

    # Execute linefollowing
    next_mode=linefollow(bot, line_sensor, proximity_sensor)
    
    if next_mode[0]=="INTERSECTION":    # reached intersection
        return ("INTERSECTION", next_mode[1])
    
    elif next_mode=="END":              # deadend

        turn_angle=turn(bot,line_sensor,angle_sensor,"right")  # spin and return to the previous intersection 
        while next_mode[0]!="INTERSECTION":                                         # wait for obstacles to move away
            next_mode=linefollow(bot, line_sensor, proximity_sensor)

        return ("END", turn_angle)
    
    elif next_mode[0]=="OBSTACLE":      # obstacle detected on the line

        distances=proximity_sensor.read_distances()

        t0=time.time()
        t=t0
        
        # Wait for 3 seconds for the obstacle to move away
        while distances[1]<=THRESHOLD_clear and t-t0<3:
            t=time.time()
            distances=proximity_sensor.read_distances()

        # Obstacle moved away in 3 seconds OR bot was/is in a tunnel: get to the next intersection
        if t-t0<3 or next_mode[1]:

            while next_mode[0]!="INTERSECTION":             # wait for obstacles to move away
                next_mode=linefollow(bot, line_sensor, proximity_sensor)

            return ("INTERSECTION", next_mode[1])
        
        # Turn back and go back to the originating intersection
        turn_angle=turn(bot,line_sensor,angle_sensor,"right")

        # #if inside a tunnel, turn_angle = 0. wait until the obstacle is removed.
        # if turn_angle==0:
        #     while next_mode[0]!="INTERSECTION":
        #         next_mode=linefollow(bot, line_sensor, proximity_sensor)
        #     return ("INTERSECTION", next_mode[1])
        
        while next_mode[0]!="INTERSECTION":                 # wait for obstacles to move away, avoid infinite turning
            next_mode=linefollow(bot, line_sensor, proximity_sensor)
            
        return ("OBSTACLE")
