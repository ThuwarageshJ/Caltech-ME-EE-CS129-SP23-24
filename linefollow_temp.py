import pigpio
import sys
import time
import traceback
from IRsensor import LineSensor
from drivesystem import DriveSystem
from anglesensor import AngleSensor
from intersection import STATUS, Map
from proximitysensor import ProximitySensor
import numpy as np
import pickle
import os
from constants import *
from behaviors import *
import threading
import ctypes
from enum import Enum
import os
import matplotlib.pyplot as plt
from ros import runros

class SharedData:
    def __init__(self):
        self.active = False
        self.dir = "straight"
        self.lock=threading.Lock()
        self.goal = (np.nan, np.nan)
        self.explore = False
        self.driveto = False
        self.step = False
        self.mode = Mode.PAUSE
        self.prev_mode =Mode.EXPLORE
        self.filename=''
        self.newpose=False
        self.robotx=0
        self.roboty=0
        self.robotheading=0
        self.newcommand=False
    def acquire(self):
        return self.lock.acquire()
    def release(self):
        self.lock.release()

def autonomous_drive(cmap, shared, proximity_sensor, line_sensor, angle_sensor, bot):
    #cmap.turned=False
    
    while shared.driveto:

        if (cmap.xpose,cmap.ypose)==cmap.goal: #goal is reached
            cmap.show()   #visualize
            cmap.cleartree()
            cmap.goal=(np.nan,np.nan)
            cmap.show()   #visualize
            return (1, "done")


        if cmap.getintersection().direction in range(8):
            if cmap.getintersection().blocked[cmap.getintersection().direction]:      
                # cmap.clear()  
                # cmap.dijkstras(goal, to_check)
                return (0, "blocked")

        if cmap.getintersection().direction==None or cmap.getintersection().cost == float('inf'):
            #cmap.clear()
            # # cmap.dijkstras(goal)
            # print("No path to goal right now!! Remove the blocks and initiate later!!")
            return (0, "none")
        
        to_turn =cmap.getintersection().direction-cmap.heading

        if abs(to_turn)==4: #need to turn 180 degrees. Prioritize by the turn which would minimize the no. of turns
            lblocks=0   #streets blocking turning, if turning left
            rblocks=0   #streets blocking turning, if turning right

            #count the blocking streets in each direction
            for h in range(cmap.heading+1, cmap.heading+4 ):
                if cmap.getintersection().streets[h%8]==STATUS.CONNECTED or cmap.getintersection().streets[h%8]==STATUS.DEADEND:
                    lblocks+=1
            for h in range(cmap.heading-1, cmap.heading-4, -1):
                if cmap.getintersection().streets[h%8]==STATUS.CONNECTED or cmap.getintersection().streets[h%8]==STATUS.DEADEND:
                    rblocks+=1

            if rblocks>lblocks: #more right blocks, so turn left
                to_turn=4
            else:   #else turn right
                to_turn=-4     

        elif to_turn>0:
            to_turn = to_turn-8 if abs(to_turn)>4 else to_turn
        elif to_turn<0:
            to_turn = 8+to_turn if abs(to_turn)>4 else to_turn

        if cmap.getintersection().direction in range(8):
            if to_turn>0 and not cmap.turned:   #turn left
                execute_drive('left', cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
            elif to_turn<0 and not cmap.turned: #turn right
                execute_drive('right', cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
            elif to_turn == 0 or cmap.turned:   #if to_turn=0, bot is in the correct robotheading, jsut go straight
                execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)

        if shared.step:
            shared.driveto=False
            shared.explore=False
            shared.active=False
    return (0, "none")

def dot_product(cmap):
    robot_vector = cmap.delta_xy[cmap.heading]
    goal_vector = [cmap.goal[0]-cmap.xpose,cmap.goal[1]-cmap.ypose]
    goal_vector_adjusted=[cmap.goal[1]-cmap.ypose, -cmap.goal[0]+cmap.xpose]
    dot_product = robot_vector[0]*goal_vector_adjusted[0]+robot_vector[1]*goal_vector_adjusted[1]
    if dot_product==0: 
        if robot_vector[0]*goal_vector[0]>0: #goal is straight ahead
            return (dot_product, 1)
        else:
            return (dot_product, 0)
    return (dot_product,1)    


def directed_explore1(cmap, shared, proximity_sensor, line_sensor, angle_sensor, bot):

    while shared.driveto:
        #cmap.show()
        #Make a list of all intersections that might contain some information
        to_check=[]
        for intersection in cmap.intersections.values():
            if not intersection.all_explored():
                unexplored_intersection =(intersection.x,intersection.y)
                to_check.append(unexplored_intersection)
        
        if not to_check:
            cmap.clear()
            for intersection in cmap.intersections.values():
                if not intersection.all_explored():
                    unexplored_intersection =(intersection.x,intersection.y)
                    to_check.append(unexplored_intersection)
        
        #run the dijkstras algorithm; initialize on_deck with the to_check list
        cmap.dijkstras(to_check)
        
        #drive according to the tree. this will try to get the robot to the goal; if goal is not known yet, then the robot will end up in the cost optimized unexplored intersection
        reached=autonomous_drive(cmap, shared, proximity_sensor, line_sensor, angle_sensor, bot)
        
        if reached[0]: 
            print("I FOUND IT")
            return
        if reached[1]=="blocked": #handle thissssss
            #print("blocked")
            execute_drive("right", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
            continue
    
        cur=cmap.getintersection()

        if (cmap.xpose,cmap.ypose)==cmap.goal: #goal is reached
            cmap.show()   #visualize
            cmap.cleartree()
            cmap.goal=(np.nan,np.nan)
            cmap.show()   #visualize
            return

        goal_angle = np.arctan2(cmap.goal[0]-cmap.xpose, cmap.goal[1]-cmap.ypose)*180/np.pi
       # print("goal_angle: ", goal_angle)
        robot_angle = cmap.heading * 45

        robot_angle = -robot_angle if robot_angle<180 else 360-robot_angle
       # print("robot angle ", robot_angle)
        diff = robot_angle-goal_angle

        if diff>180:
            diff = -(360-diff)
        elif diff<-180:
            diff = 360-abs(diff)

        #count the blocking streets in each direction
        l=0   #streets blocking turning, if turning left
        r=0   #streets blocking turning, if turning right
        l_con=0
        r_con=0
        l_unexp=0
        r_unexp=0
        unknown_h=[None,None]
        connected_h=[None,None]
        
        for h in range(cmap.heading+1, cmap.heading+4 ):
            if cmap.getintersection().streets[h%8]==STATUS.UNKNOWN:
                l+=1
                if connected_h[0]==None:
                    connected_h[0]=h%8
            if cmap.getintersection().streets[h%8]==STATUS.CONNECTED:
                l_con+=1
                if connected_h[0]==None:
                    connected_h[0]=h%8
        for h in range(cmap.heading-1, cmap.heading-4, -1):
            if cmap.getintersection().streets[h%8]==STATUS.UNKNOWN:
                r+=1
                if connected_h[1]==None:
                    connected_h[1]=h%8
            if cmap.getintersection().streets[h%8]==STATUS.CONNECTED:
                r_con+=1
                if connected_h[1]==None:
                    connected_h[1]=h%8
        if l==0 and r==0:   #if no blocking streets, count the unexplored streets
            for h in range(cmap.heading+1, cmap.heading+4 ):
                if cmap.getintersection().streets[h%8]==STATUS.UNEXPLORED and not cmap.getintersection().blocked[h%8]:
                    l+=1
                    if connected_h[0]==None:
                        connected_h[0]=h%8
            for h in range(cmap.heading-1, cmap.heading-4, -1):
                if cmap.getintersection().streets[h%8]==STATUS.UNEXPLORED and not cmap.getintersection().blocked[h%8]:
                    r+=1
                    if connected_h[1]==None:
                        connected_h[1]=h%8

        if not shared.driveto:
            break
        if diff==0: #straight ahead
            #print("goal straight ahead")
            if cmap.getintersection().streets[cmap.heading] in [STATUS.UNEXPLORED, STATUS.CONNECTED] and not cmap.getintersection().blocked[cmap.heading] :
                execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
            else:
                if l>0:
                    execute_drive("left", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                    execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                else:
                    execute_drive("right", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                    execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
        elif diff>0: #turn lefts
            #print("goal to the left")
            if l>0:
                execute_drive("left", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
            elif l_con>0:
                if not cmap.getintersection().blocked[connected_h[0]]:
                    execute_drive("left", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                    execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                elif cmap.getintersection().streets[cmap.heading] in [STATUS.UNEXPLORED, STATUS.CONNECTED] and not cmap.getintersection().blocked[cmap.heading] :
                    execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                else:
                    execute_drive("right", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                    execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
            elif cmap.getintersection().streets[cmap.heading] in [STATUS.UNEXPLORED, STATUS.CONNECTED] and not cmap.getintersection().blocked[cmap.heading] :
                execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
            elif r>0:
                execute_drive("right", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
            else:
                execute_drive("right", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
        elif diff<0:
            #print("goal to the right")
            if r>0:
                execute_drive("right", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
            elif r_con>0:
                if not cmap.getintersection().blocked[connected_h[1]]:
                    execute_drive("right", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                    execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                elif cmap.getintersection().streets[cmap.heading] in [STATUS.UNEXPLORED, STATUS.CONNECTED] and not cmap.getintersection().blocked[cmap.heading] :
                    execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                else:
                    execute_drive("left", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                    execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
            elif cmap.getintersection().streets[cmap.heading] in [STATUS.UNEXPLORED, STATUS.CONNECTED] and not cmap.getintersection().blocked[cmap.heading] :
                execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
            elif l>0:
                execute_drive("left", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
            else:
                execute_drive("left", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)

        if (cmap.xpose,cmap.ypose)==cmap.goal: #goal is reached
            cmap.show()   #visualize
            cmap.cleartree()
            cmap.goal=(np.nan,np.nan)
            cmap.show()   #visualize
            return
        
        if shared.step:
            shared.driveto=False
            shared.explore=False
            shared.active=False
        
      
        # while True:
        #     if cur.all_known():
        #         break
        #     execute_drive("right", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)

        # min_dist=1e4
        # best=None
        
        # for h in range(cmap.heading, cmap.heading+8):
        #     if cur.streets[h%8] in [STATUS.UNEXPLORED] and not cur.blocked[h%8]:
        #         projected_intersection=(cur.x+cmap.delta_xy[h%8][0],cur.y+cmap.delta_xy[h%8][1])
        #         dist = (projected_intersection[0]-cmap.goal[0])**2+(projected_intersection[1]-cmap.goal[1])**2
        #         if dist<min_dist:
        #             min_dist=dist
        #             best=h
        
        # if best==None:
        #     if cur in to_check:
        #         to_check.remove(cur)
        #     continue
        
        # if best-cmap.heading>=4:
        #     while cmap.heading!=(best%8):
        #         execute_drive("right", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
        # else:
        #     while cmap.heading!=(best%8):
        #         execute_drive("left", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
        
        # execute_drive("straight", cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)

def explore(cmap, shared, proximity_sensor, line_sensor, angle_sensor, bot):

    cur=cmap.getintersection()  #start with current intersection

    while shared.explore:

        if not cur.all_explored() and not cmap.turned:  #not recently turned, free to choose any action

            #count the blocking streets in each direction
            l=0   #streets blocking turning, if turning left
            r=0   #streets blocking turning, if turning right
            for h in range(cmap.heading+1, cmap.heading+5):
                if cmap.getintersection().streets[h%8]==STATUS.UNKNOWN:
                    l+=1
            for h in range(cmap.heading-1, cmap.heading-4, -1):
                if cmap.getintersection().streets[h%8]==STATUS.UNKNOWN:
                    r+=1
            if l==0 and r==0:   #if no blocking streets, count the unexplored streets
                for h in range(cmap.heading+1, cmap.heading+5):
                    if cmap.getintersection().streets[h%8]==STATUS.UNEXPLORED and not cmap.getintersection().blocked[h%8]:
                        l+=1
                for h in range(cmap.heading-1, cmap.heading-4, -1):
                    if cmap.getintersection().streets[h%8]==STATUS.UNEXPLORED and not cmap.getintersection().blocked[h%8]:
                        r+=1
            
            if l==0 and r==0:   #no streets to explore on the sides. go straight
                execute_drive("straight",cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                cur=cmap.getintersection()
            elif l>r:   #more left streets to explore. turn left
                execute_drive('left',cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
            else:   #more right tstreets to explore. turn right
                execute_drive('right',cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)

        elif not cur.all_explored() and cmap.turned:    #recently turned, so go straight
            execute_drive("straight",cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
            cur=cmap.getintersection()

        elif cur.all_explored():  
            #reached= False
            #while not reached and shared.explore:
            min_dist = 1e4
            next = None
            cmap.goal=(cur.x, cur.y)
            cmap.dijkstras()
            to_check=0
            for intersection in cmap.intersections.values():    #find closest unexplored intersection
                if not intersection.all_explored():
                    to_check+=1
                    dist = intersection.cost
                    dir = intersection.direction
                    if dir != None:
                        if dist<min_dist and dist!=0:
                            next = intersection
                            min_dist=dist

            cmap.cleartree()
            cmap.goal = (np.nan, np.nan)
            if to_check==0:         #all all_explored
                print("All explored!")
                cmap.show()
                return
            
            if next== None: #there are unexploreds but no paths
                cmap.clear()
            else:
                cmap.goal = (next.x, next.y)
                cmap.dijkstras()
                shared.driveto=True
                reached = autonomous_drive(cmap, shared, proximity_sensor, line_sensor, angle_sensor, bot)
                cmap.goal = (np.nan, np.nan)
                cur=cmap.getintersection()
            
        if shared.step:
            shared.explore=False
            shared.driveto=False
            shared.active=False

#save the intersections dictionary as a .pickle file
def savepickle(intersections, filename):
    if filename[len(filename)-7:]!='.pickle':   #add extension
        filename+='.pickle'
    # Save the map to file.
    print("\n Saving the map as %s..." % filename)
    filepath=os.path.join(cur_dir, 'maps/'+filename)
    with open(filepath, 'wb') as file:
        pickle.dump(intersections, file)

#retrieve a pre-existing .pickle file (map)
def getpickle(filename):
    # Load the map from file.
    maps= os.listdir(map_filenames) 
    if filename not in maps:
        print("\n"+filename+ " does not exist!")
        return None
    filepath=os.path.join(cur_dir, 'maps/'+filename)
    print("\n Loading the map from %s..." % filename)
    with open(filepath, 'rb') as file:
        intersections = pickle.load(file)
    return intersections

#high level function executing turns and straight drives
def execute_drive(dir, cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared):

    if dir in ["left", "right"]:
        cmap.turned= True
        if cmap.getintersection().tunnel:
            return
        hold=cmap.heading    #old robotheading
        turn_angle=turn(bot, line_sensor, angle_sensor,proximity_sensor, dir)
        cmap.calcturn(turn_angle, dir)  
        

        #start end parameters for the loop below
        start=hold
        end=cmap.heading
        if dir =="left" and cmap.heading<=hold:
            end+=8
        elif dir=="right" and cmap.heading>=hold:
            start+=8

        i= 1 if dir=="left" else -1

        #update all robotheadings in between hold and robotheading as nonexistent
        for h in range(start+i, end, i):
            #if cmap.getintersection(x,y).streets[h%8]==STATUS.UNKNOWN:   #extra safety
            if cmap.getintersection().streets[h%8] in [STATUS.CONNECTED, STATUS.UNEXPLORED, STATUS.DEADEND]:
                print("I MESSED UP THE MAP. PLEASE RESET THE POSE!!!!!!!!")
                if shared.acquire():
                    shared.driveto=False
                    shared.explore=False
                    shared.active =False
                    shared.newcommand=True
                    shared.release()

                return
            cmap.getintersection().streets[h%8] = STATUS.NONEXISTENT  
        
        #ASSUMING NO DOUBLE TURNS!!!
        cmap.getintersection().streets[(cmap.heading+1)%8] = STATUS.NONEXISTENT  
        cmap.getintersection().streets[(cmap.heading-1)%8] = STATUS.NONEXISTENT
        #ASSUMING NO DOUBLE TURNS!!!

        #mark current robotheading as unexplored
        if cmap.getintersection().streets[cmap.heading] == STATUS.UNKNOWN:
            cmap.getintersection().streets[cmap.heading]=STATUS.UNEXPLORED
        
        obstacle_ahead=check_ahead(proximity_sensor,cmap.heading)
        cmap.update_using_checkahead(obstacle_ahead)
    
    else:
        cmap.turned= False
        obstacle_ahead=check_ahead(proximity_sensor,cmap.heading)
        cmap.update_using_checkahead(obstacle_ahead)
    
        #skip if the map says there's no street ahead or blocked
        if cmap.getintersection().streets[cmap.heading]==STATUS.NONEXISTENT or cmap.getintersection().blocked[cmap.heading]:
            cmap.show()
            return 
        
        result = smartlinefollow(bot, line_sensor, proximity_sensor, angle_sensor)

        if result[0]=="INTERSECTION":
            street_ahead=pullforward(bot, line_sensor) #pull forward if an intersection is detected

            cmap.getintersection().streets[cmap.heading] = STATUS.CONNECTED  #previous x,y has an intersection in this robotheading
            cmap.getintersection().blocked[cmap.heading] = False
            #ASSUMING NO DOUBLE TURNS!!!
            cmap.getintersection().streets[(cmap.heading+1)%8] = STATUS.NONEXISTENT  
            cmap.getintersection().streets[(cmap.heading-1)%8] = STATUS.NONEXISTENT

            cmap.calcmove()

            cmap.getintersection().streets[(cmap.heading+4)%8] = STATUS.CONNECTED  #current x,y has an intersection in the opposite robotheading
            cmap.getintersection().blocked[(cmap.heading+4)%8] = False
            #ASSUMING NO DOUBLE TURNS
            cmap.getintersection().streets[(cmap.heading+3)%8] = STATUS.NONEXISTENT  
            cmap.getintersection().streets[(cmap.heading-3)%8] = STATUS.NONEXISTENT

            if result[1]:    #in tunnel
                street_ahead="STREET"
                cmap.getintersection().tunnel= True
                for i in range(2):
                    cmap.getintersection().streets[(cmap.heading+i+1)%8] = STATUS.NONEXISTENT  
                    cmap.getintersection().streets[(cmap.heading-i-1)%8] = STATUS.NONEXISTENT
                
            cmap.update_using_pullforward(street_ahead)

            obstacle_ahead=check_ahead(proximity_sensor,cmap.heading)
            cmap.update_using_checkahead(obstacle_ahead)
        
        elif result[0]=="END":
            street_ahead = pullforward(bot, line_sensor)
            cmap.getintersection().streets[cmap.heading]=STATUS.DEADEND
            cmap.calcturn(-180, "right", True)      #make use of actual turn angle for error detection
            cmap.update_using_pullforward(street_ahead)   

        elif result=="OBSTACLE":
            street_ahead = pullforward(bot, line_sensor)
            cmap.update_using_checkahead("OBSTACLE")
            cmap.calcturn(-180, "right", True)
            cmap.update_using_pullforward(street_ahead) 
            obstacle_ahead=check_ahead(proximity_sensor,cmap.heading)
            cmap.update_using_checkahead(obstacle_ahead)
    
    if shared.acquire():
        shared.robotx=cmap.xpose
        shared.roboty=cmap.ypose
        shared.robotheading=cmap.heading
        shared.release()

    count_streets = 0
    for h in range(8):
        if cmap.getintersection().streets[h] not in [STATUS.NONEXISTENT]:
            count_streets+=1
    if count_streets==0:
        print("I AM LOST!!!!!")
    cmap.show()

def ui(shared):
    try:
        running = True
        while running:
            command=input("Enter a command: 'explore', 'goal', 'pause', 'step', 'resume', 'left', 'right', 'straight', 'save', 'load', 'pose', 'quit', 'clear'")
            if shared.acquire():
                if command == 'explore':
                    shared.prev_mode=shared.mode
                    shared.explore=True
                    shared.driveto=False
                    shared.mode = Mode.EXPLORE
                    shared.active = True
                    shared.step=False
                    shared.newcommand=True
                elif command.split()[0] == 'goal':
                    shared.prev_mode=shared.mode
                    shared.explore=False
                    shared.driveto=False
                    shared.goal = tuple(map(int, command.split()[1:]))
                    shared.mode = Mode.DRIVETO
                    shared.step=False
                    shared.active = True
                    shared.newcommand=True
                elif command == 'pause':
                    shared.driveto=False
                    shared.explore=False
                    shared.active =False
                    shared.newcommand=True
                elif command == 'step':
                    if not shared.driveto and not shared.explore:
                        if shared.mode == Mode.EXPLORE:
                            shared.explore=True
                            shared.active =True
                        elif shared.mode == Mode.DRIVETO:
                            shared.driveto=True
                            shared.active =True
                        shared.step=True
                    shared.newcommand=True
                elif command == 'resume':
                    if not shared.driveto and not shared.explore:
                        if shared.mode == Mode.EXPLORE:
                            shared.explore=True
                            shared.active =True
                            shared.newcommand=True
                        elif shared.mode == Mode.DRIVETO:
                            shared.driveto=True
                            shared.active =True
                            shared.newcommand=True
                        shared.step=False
                elif command in ['left', 'right', 'straight']:
                    shared.dir=command
                    shared.prev_mode=shared.mode
                    shared.mode = Mode.MANUAL
                    shared.driveto=False
                    shared.explore=False
                    shared.step=False
                    shared.active =True
                    shared.newcommand=True
                elif command.split()[0] == 'save':
                    shared.filename = command.split()[1]
                    shared.prev_mode=shared.mode
                    shared.mode=Mode.SAVE
                    shared.active=True
                    shared.newcommand=True
                    shared.driveto=False
                    shared.explore=False
                    shared.newcommand=True
                elif command.split()[0] == 'load':
                    shared.prev_mode=shared.mode
                    shared.filename = command.split()[1]
                    if shared.filename =='n':
                        x, y, robotheading = tuple(map(int, command.split()[2:]))
                        shared.robotx = x
                        shared.roboty = y
                        shared.robotheading = robotheading
                        shared.newpose=True
                    shared.mode=Mode.LOAD
                    shared.active=True
                    shared.newcommand=True
                elif command.split()[0] == 'pose':
                    x, y, robotheading = tuple(map(int, command.split()[1:]))
                    shared.robotx = x
                    shared.roboty = y
                    shared.robotheading = robotheading
                    shared.newpose=True
                elif command == 'quit':
                    shared.driveto=False
                    shared.explore=False
                    shared.step=False
                    running=False
                elif command == 'clear':
                    shared.prev_mode=shared.mode
                    shared.mode = Mode.CLEAR
                    shared.active=True
                    shared.newcommand=True
                    shared.active=True
                    shared.newcommand=True
                    shared.driveto=False
                    shared.explore=False
                    shared.newcommand=True
                else:
                    shared.newcommand=False
                    print('Illegal command '+ command)
                shared.release()
    except BaseException as ex:
        print("Ending UI due to exception: %s" % repr(ex))

def worker(shared, bot, angle_sensor, line_sensor, cmap, proximity_sensor):

    cmap.show()
    goal = (np.nan, np.nan)
    dir ="straight"
    active = False
    newpose=False
    robotx=0
    roboty=0
    robotheading=0

    try:
        while True:
            if shared.acquire():
                mode=shared.mode
                prev_mode=shared.prev_mode
                dir=shared.dir
                goal= shared.goal
                if shared.newcommand:
                    active=shared.active
                    shared.newcommand=False
                else:
                    active=False
                filename=shared.filename
                if shared.newpose:
                    newpose=True
                    robotx=shared.robotx
                    roboty=shared.roboty
                    robotheading=shared.robotheading
                    shared.newpose=False
                shared.release() 
            
            if newpose:
                cmap.xpose=robotx
                cmap.ypose=roboty
                cmap.heading=robotheading
                cmap.show()   
                newpose=False

            if not active:
                bot.stop()
            elif mode ==Mode.SAVE:
                cmap.cleartree()
                savepickle(cmap.intersections, filename)
                if prev_mode == Mode.EXPLORE:
                    if shared.acquire():
                        shared.mode=Mode.EXPLORE
                        shared.explore=True
                        shared.active =True
                        shared.newcommand=True
                        shared.release()
                elif prev_mode == Mode.DRIVETO:
                    if shared.acquire():
                        shared.mode =Mode.DRIVETO
                        shared.driveto=True
                        shared.active =True
                        shared.newcommand=True
                        shared.release()
            elif mode==Mode.LOAD:
                if filename == 'n':
                    cmap=Map({})
                    street_ahead=initialize_robot(cmap, bot, line_sensor, proximity_sensor)
                    cmap.xpose=robotx
                    cmap.ypose=roboty
                    cmap.heading=robotheading
                    _=cmap.getintersection()  #initialize intersection at current pose
                    cmap.update_using_pullforward(street_ahead) #update current intersection using pull forward data
                    cmap.show()
                elif getpickle(filename)!= None:
                    cmap=Map(getpickle(filename))
                    cmap.cleartree()
                    cmap.show()
            elif mode == Mode.MANUAL:
                execute_drive(dir, cmap, proximity_sensor, line_sensor, angle_sensor, bot, shared)
                cmap.cleartree()
            elif mode == Mode.DRIVETO:
                if goal in cmap.intersections:
                    cmap.clear()
                    cmap.goal=goal
                    cmap.dijkstras()
                    shared.driveto=True
                    reached = (0, "dummy")
                    while shared.driveto:
                        reached = autonomous_drive(cmap, shared, proximity_sensor, line_sensor, angle_sensor, bot)
                        if reached[0]:
                            break
                        if reached[1]=='none':
                            
                            directed_explore1(cmap, shared, proximity_sensor, line_sensor, angle_sensor, bot)
                        else:
                            cmap.dijkstras()
                else:  #unhandled yet
                    cmap.goal=goal
                    shared.driveto=True
                    cmap.clear()
                    directed_explore1(cmap, shared, proximity_sensor, line_sensor, angle_sensor, bot)
                cmap.goal=(np.nan, np.nan)
            elif mode == Mode.EXPLORE:
                cmap.clear()
                cmap.show()
                explore(cmap, shared, proximity_sensor, line_sensor, angle_sensor, bot)       
            elif mode == Mode.CLEAR:
                cmap.clear()
                cmap.show()
                if prev_mode == Mode.EXPLORE:
                    if shared.acquire():
                        shared.mode=Mode.EXPLORE
                        shared.explore=True
                        shared.active =True
                        shared.newcommand=True
                        shared.release()
                elif prev_mode == Mode.DRIVETO:
                    if shared.acquire():
                        shared.mode =Mode.DRIVETO
                        shared.driveto=True
                        shared.active =True
                        shared.newcommand=True
                        shared.release()
    except BaseException as ex:
        print("Ending Worker due to exception: %s" % repr(ex))

def initialize_robot(cmap, bot, line_sensor, proximity_sensor):
    next_mode = linefollow(bot,line_sensor, proximity_sensor)
    while next_mode[0]!= "INTERSECTION":    #linefollow until you find an intersection
        turn_angle= turn(bot, line_sensor, angle_sensor,proximity_sensor, "right")   #spin back to line
        next_mode = linefollow(bot,line_sensor, proximity_sensor) 
    street_ahead=pullforward(bot, line_sensor)  #now you've reached an intersection
    return street_ahead

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
    angle_sensor=AngleSensor(io)
    line_sensor=LineSensor(IR_left_pin, IR_mid_pin, IR_right_pin, io)
    proximity_sensor = ProximitySensor(io, trig_pins, echo_pins)
    shared=SharedData()
    cmap = Map({})

    cmap.cleartree()   
        
    #Initialization 
    street_ahead = initialize_robot(cmap, bot, line_sensor, proximity_sensor)
    #userinput for the current position of the bot
    pose = tuple(int(i) for i in input("Enter my x-coordinate, y-coordinate, and robotheading: ").split())   #user input for current pose
    while len(pose)!=3:     #validate user input for current pose
        pose = tuple(int(i) for i in input('Input valid x, y and robotheading').split())
    (cmap.xpose,cmap.ypose,cmap.heading)=pose[0],pose[1],pose[2]   #set pose
    if shared.acquire():
        (shared.robotx, shared.roboty, shared.robotheading)=cmap.xpose,cmap.ypose,cmap.heading
        shared.release()

    _=cmap.getintersection()  #initialize intersection at current pose
    cmap.update_using_pullforward(street_ahead) #update current intersection using pull forward data

    
    # Start the ROS worker thread.
    rosthread = threading.Thread(name="ROSThread", target=runros, args=(shared,))
    rosthread.start()

    # Create and start the worker thread
    worker_thread = threading.Thread(target=worker, args=(shared, bot, angle_sensor, line_sensor, cmap, proximity_sensor))
    worker_thread.start()
    ui(shared)

    #send keyboard exception to worker thread
    ctypes.pythonapi.PyThreadState_SetAsyncExc(
            ctypes.c_long(worker_thread.ident),
            ctypes.py_object(KeyboardInterrupt))
    
    # Wait for the worker thread to finish
    worker_thread.join()
    
    # End the ROS thread (send the KeyboardInterrupt exception).
    ctypes.pythonapi.PyThreadState_SetAsyncExc(
    ctypes.c_long(rosthread.ident), ctypes.py_object(KeyboardInterrupt))
    rosthread.join()

    #stop the motors and disable the ios
    proximity_sensor.shutdown()
    bot.stop()
    io.stop()