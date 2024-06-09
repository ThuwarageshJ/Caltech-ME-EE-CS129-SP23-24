from constants import *
from proximitysensor import ProximitySensor
from drivesystem import DriveSystem
from anglesensor import AngleSensor
import time


def calculate_raw_linefollow(readings):
    #Raw values for line following detectors:(0: Side ; 1: End-of-line ; 2: Intersection)
    raw=[0,0,0] 

    if readings==(0,0,0):
        raw[1]=1
    elif readings==(1,1,1):
        raw[2]=1
    elif readings==(0,1,1):
        raw[0]=0.5
    elif readings==(0,0,1):
        raw[0]=1
    elif readings==(1,0,0):
        raw[0]=-1
    elif readings==(1,1,0):
        raw[0]=-0.5

    return raw

def calculate_raw_midsensor(readings):
    #Raw values for pull forward detector and turning detector
    raw=0
    if readings[1]==1:
        raw=1
    return raw

def calculate_raw_turn(readings, direction):
    #Raw values for pull forward detector and turning detector
    raw=0
    if direction== "right":
        if readings[1]==1 and readings[2]==1:
            raw=1
        elif readings[2]==1:
            raw=0.5
    if direction== "left":
        if readings[1]==1 and readings[0]==1:
            raw=1
        elif readings[0]==1:
            raw=0.5
    return raw

#Function to return 0 or 1 corresponding to the 20 cm range used for the herding behavior.
def digitalize(readings):

   digital_readings=[0,0,0]
   for i in range(3):
       if readings[i]<THRESHOLD_herd:  #Obstacle found within 20 cm
           digital_readings[i]=1
   if readings[1]<0.5*THRESHOLD_herd:  #Front ultrasound sees an obstacle within 10 cm
       digital_readings[1]=-1
   return tuple(digital_readings) 

#Function to check for blockages before and after an action
def check_ahead(proximity_sensor, heading):
    distances = proximity_sensor.read_distances()
    if distances[1] < THRESHOLD_checkahead[heading%2]:
        return "OBSTACLE"
    return "CLEAR"

#Turn behavior
def turn(bot,line_sensor,angle_sensor,proximity_sensor, direction, tunnel = False):   
    
    if tunnel:
        return 0
    
    readings=line_sensor.read()
    
    t=time.time()
    level=0.5
    offroad=False
    angle_i=angle_sensor.read_angle()
    bot.drive(direction,"spin")

    while True:

        tlast=t
        t=time.time()
        dt=t-tlast

        readings = line_sensor.read()

        #calculate raw values and update levels
        raw = calculate_raw_turn(readings, direction) 
        level = level + dt* (raw - level)/T_turn

        if level>THRESHOLD_turn and offroad:  #found a new street
            bot.stop()
            angle_f=angle_sensor.read_angle()
            turn_angle = (angle_f-angle_i)
            return turn_angle  
        elif level<1-THRESHOLD_turn:
            offroad=True
            bot.drive(direction,"spin")
        
#Pullforward behavior
def pullforward(bot, line_sensor):

    t0=time.time()
    level=0
    t=t0
    while True:
        tlast=t
        t=time.time()
        dt=t-tlast

        readings = line_sensor.read()
        raw = calculate_raw_midsensor(readings)
        level = level + dt* (raw - level)/T_linefollow[1]

        if t < t0 + T_drive:    #until T_drive has elapsed starting from t0
            bot.drive("left","straight")
        else:       #bot drove straight for a time of T_drive
            bot.stop()  
            if level>THRESHOLD_linefollow[1]:   #there's a street in the front      
                return "STREET"
            # elif level<1-THRESHOLD_linefollow[1]:   #there's no street in the front
            #     return "NOSTREET"
            else:       #for extra safety
                return "NOSTREET"

#Linefollow behavior
def linefollow(bot,line_sensor, proximity_sensor):
    t=time.time()
    level=np.array([0,0.1,0]) #Initialize levels for line following:(0: Side ; 1: End-of-line ; 2: Intersection)
    tunnel = False
    while True:

        tlast=t
        t=time.time()
        dt=t-tlast

        readings = line_sensor.read()

        distances = proximity_sensor.read_distances()

        if distances[1]<THRESHOLD_obstacle:
            bot.stop()          
            return ("OBSTACLE", tunnel)

        #stop updating the levels if you are pushed
        if not(readings==(0,0,0) and abs(level[0])>THRESHOLD_linefollow[0]):
            raw = calculate_raw_linefollow(readings) 
            level = level + dt* (raw - level)/T_linefollow

        if readings==(0,1,0):
            bot.drive("left","straight")
        elif readings ==(0,1,1):
            bot.drive("right","turn")
        elif readings ==(0,0,1):
            bot.drive("right","hook")
        elif readings ==(1,1,0):
            bot.drive("left","turn")
        elif readings ==(1,0,0):
            bot.drive("left","hook")
        elif readings ==(0,0,0):
            
            if level[0]>THRESHOLD_linefollow[0]:     #pushed to the left
                bot.drive("right","spin")
            elif level[0]>1-THRESHOLD_linefollow[0]:    #continue with previous command
                continue
            elif level[0]<(-1)*THRESHOLD_linefollow[0]:    #pushed to the right
                bot.drive("left","spin")
            elif level[0]<THRESHOLD_linefollow[0]-1:    #continue with previous command
                continue
            elif level[1]>THRESHOLD_linefollow[1]:       #reached end of line
                if distances[0]+distances[2]<0.5 or tunnel:
                    error = (distances[0]-distances[2])
                    #PWM calculation using the linear fit equation
                    PWM_L = CL + KL * error
                    PWM_R = CR + KR * error                
                    if PWM_L >= 1 or PWM_R >= 1 or PWM_L <= -1 or PWM_R <= -1:   #invalid PWM values
                        continue   
                    bot.pwm(PWM_L, PWM_R)
                    tunnel = True
                else:
                    bot.stop()
                    return "END"
            elif level[1]<1-THRESHOLD_linefollow[1]:     #potholes
                bot.drive("left","straight")
        elif readings ==(1,1,1):
            if level[2]>THRESHOLD_linefollow[2]:    #detected an intersection
                tunnel = tunnel and distances[0]+distances[2]<0.5
                bot.stop()
                return ("INTERSECTION", tunnel)   
            elif level[2]<1-THRESHOLD_linefollow[2]:    
                bot.drive("left","straight") 
        elif readings ==(1,0,1):                    #possibly a Y intersection
            if level[2]>THRESHOLD_linefollow[2]:    #detected an intersection
                if distances[0]+distances[2]<0.5:
                    tunnel = True
                bot.stop()
                return ("INTERSECTION", tunnel) 

#smart-linefollow behavior; high level to the linefollow behavior - handles obstacles
def smartlinefollow(bot,line_sensor, proximity_sensor, angle_sensor):

    #execute linefollowing
    next_mode=linefollow(bot, line_sensor, proximity_sensor)
    
    if next_mode[0]=="INTERSECTION":    #reached intersection

        return ("INTERSECTION", next_mode[1])
    
    elif next_mode=="END":     #deadend

        #spin and go back to the originating intersection
        turn_angle=turn(bot, line_sensor, angle_sensor, proximity_sensor, "right")  
        while next_mode[0]!="INTERSECTION":
            next_mode=linefollow(bot, line_sensor, proximity_sensor)
        return ("END", turn_angle)
    
    elif next_mode[0]=="OBSTACLE":  #obstacle detected on the line

        #read real time distances
        distances=proximity_sensor.read_distances()
        t0=time.time()
        t=t0
        
        #wait for 3 seconds for the obstacle to move away
        while distances[1]<=THRESHOLD_clear and t-t0<3:
            t=time.time()
            distances=proximity_sensor.read_distances()

        #obstacle moved away: get to the next intersection
        if t-t0<3 or next_mode[1]:
            while next_mode[0]!="INTERSECTION":
                next_mode=linefollow(bot, line_sensor, proximity_sensor)
            return ("INTERSECTION", next_mode[1])
        
        #obstacle didn't move away, try to turn back and go back to the originating intersection
        turn_angle=turn(bot, line_sensor, angle_sensor, proximity_sensor, "right", next_mode[1])

        #if inside a tunnel, turn_angle = 0. wait until the obstacle is removed.
        if turn_angle==0:
            while next_mode[0]!="INTERSECTION":
                next_mode=linefollow(bot, line_sensor, proximity_sensor)
            return ("INTERSECTION", next_mode[1])
        
        #else, just go back to the originating intersection
        while next_mode[0]!="INTERSECTION":
            next_mode=linefollow(bot, line_sensor, proximity_sensor)
        return ("OBSTACLE")
