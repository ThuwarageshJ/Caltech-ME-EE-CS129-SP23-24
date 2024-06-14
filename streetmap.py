"""
    Caltech ME/EE/CS129 - Experimental Robotics - Spring 23/24 
    Team Robo-TEd
    Contributors: Thuwaragesh Jayachandran, Edward Ju

    Map class for street map handling, visualization, and executing graph algorithms. (Manhattan style maps)

"""
import pigpio
import sys
import time
import traceback
import numpy as np
import matplotlib.pyplot as plt
from enum import Enum
import bisect
from constants import *
import math

# Enumerations to indicate street status
class STATUS(Enum):

    UNKNOWN = 0             # no information about the street
    NONEXISTENT = 1         # no street
    UNEXPLORED = 2          # street exists, but unexplored
    DEADEND = 3             # street exists- deadend
    CONNECTED = 4           # street exists- connected to another intersection

    def __str__(self):
        return self.name

    def __eq__(self, other):
        return self.value == other.value

# Intersection class to handle 'Nodes' of the street map
class Intersection:

    def __init__(self, x, y):
       
        # Caretesian coordinates of the intersection
        self.x = x                
        self.y = y

        # Status of all streets in the 8 directions of the intersection
        self.streets = [STATUS.UNKNOWN for i in range(8)] 
        self.blocked = [False for i in range(8)]            # True if there is a street and it is blocked 

        # Cost and Direction to a specified goal: used when Dijkstra's algorithm is run
        self.cost = float('inf')
        self.direction = None
        
        self.tunnel = False         # True if the intersection is inside a tunnel
    
    def reset_cost(self):
       self.cost = float('inf')

    def set_cost(self, cost):
       self.cost = cost

    def set_direction(self, direction):
       self.direction=direction

    def reset_direction(self):
       self.direction=None
    
    # For sorting of intersections in Dijkstra's algorithm
    def __lt__(self, other):
        return self.cost < other.cost
    
    # Check if there are driveable (non-blocked) UNEXPLORED or UNKNOWN streets
    def all_explored(self):

        explored=True       # no driveable (non-blocked) UNEXPLORED or UNKNOWN streets
        
        for h in range(8):
            if ((self.streets[h] in [STATUS.UNEXPLORED, STATUS.UNKNOWN]) and not self.blocked[h]):
                explored= False
                break

        return explored

    # Check if there are UNKNOWN streets remaining
    def all_known(self):

        known=True          # no unknown streets

        for h in range(8):
            if ((self.streets[h] in [STATUS.UNKNOWN])):
                known= False
                break

        return known

# Map class to handle the Manhattan style street maps as a collection of intersections/ nodes (objects of Intersection class)
class Map:

    # Define the colors for each status.
    colors = {
        STATUS.UNKNOWN.name: 'black',
        STATUS.NONEXISTENT.name: 'lightgray',
        STATUS.UNEXPLORED.name: 'blue',
        STATUS.DEADEND.name: 'red',
        STATUS.CONNECTED.name: 'green'
    }

    # Change in x and y coordinates for a single movement in each heading - used for plotting and calculating bot's new coordinates
    delta_xy={
        0:[0,1],
        1:[-1,1],
        2:[-1,0],
        3:[-1,-1],
        4:[0,-1],
        5:[1,-1],
        6:[1,0],
        7:[1,1]
    }

    # Initialize the map with a dictionary of Intersection objects ([x,y]:Intersection(x,y))
    def __init__(self, intersections):

        self.intersections = intersections
        
        # Current x, y, and heading of the robot
        self.xpose=0
        self.ypose=0
        self.heading=0

        self.turned=False               # flag to determine if the bot turned recently
        self.goal =(np.nan, np.nan)     # coordinates of the goal set for the bot

    # Clear all the blockages in the map
    def clear(self):
        for intersection in self.intersections.values():
            intersection.blocked=[False for i in range (8)]

    # Retrieve an intersection object of the map
    def getintersection(self, x=None, y=None):

        if x==None and y==None:             # return the current intersction
            x=self.xpose
            y=self.ypose

        # Create and add the intersection object to the dictionary, if it's not in the map already
        if (x, y) not in self.intersections:        
            self.intersections[(x, y)] = Intersection(x, y)

        return self.intersections[(x, y)]
    
    # Update bot's x (xpose), y (ypose) for a single step in the current heading
    def calcmove(self):     
        self.xpose=self.xpose+self.delta_xy[self.heading][0]
        self.ypose=self.ypose+self.delta_xy[self.heading][1]
    
    # Update bot's heading after turning
    def calcturn(self, turn_angle, direction, spin = False): 
        
        if turn_angle==0:
            return
        
        start=self.heading

        # For an 180 degree spin
        if spin:
            self.heading=(self.heading+4)%8
            return
        
        # Update based on map if the streets of this intersection are already known
        if direction =='left':
            for h in range(start+1, (start+7)):
                if self.getintersection().streets[h%8] in [STATUS.UNKNOWN]: # no information on map
                    break
                if self.getintersection().streets[h%8] not in [STATUS.NONEXISTENT, STATUS.UNKNOWN]: # very next street for a left turn
                    self.heading=h%8
                    return
        else:
            for h in range(start-1, (start-7),-1):
                if self.getintersection().streets[h%8] in [STATUS.UNKNOWN]: # no information on map
                    break
                if self.getintersection().streets[h%8] not in [STATUS.NONEXISTENT, STATUS.UNKNOWN]:  # very next street for a right turn
                    self.heading=h%8
                    return
        
        # Update using turn_angle if not enough data on map
        self.heading=(self.heading+round(turn_angle/45))%8

    # Update map with the intersection found after a straight drive
    def update_intersection(self, tunnel):
        
        # Originating intersection has a connected street in the current heading
        self.getintersection().streets[self.heading] = STATUS.CONNECTED     
        self.getintersection().blocked[self.heading] = False

        ### ASSUMING NO TWO STREETS AT 45 DEGREE GAP !!! ###
        self.getintersection().streets[(self.heading+1)%8] = STATUS.NONEXISTENT  
        self.getintersection().streets[(self.heading-1)%8] = STATUS.NONEXISTENT
        ### ASSUMING NO TWO STREETS AT 45 DEGREE GAP !!! ###

        self.calcmove()     # update the x (xpose),y (ypose) positions of the bot

        # The new intersection has a connected street in the opposite heading
        self.getintersection().streets[(self.heading+4)%8] = STATUS.CONNECTED  
        self.getintersection().blocked[(self.heading+4)%8] = False

        ### ASSUMING NO TWO STREETS AT 45 DEGREE GAP !!! ###
        self.getintersection().streets[(self.heading+3)%8] = STATUS.NONEXISTENT  
        self.getintersection().streets[(self.heading-3)%8] = STATUS.NONEXISTENT
        ### ASSUMING NO TWO STREETS AT 45 DEGREE GAP !!! ###

        if tunnel:                                      # intersection inside a tunnel

            self.getintersection().tunnel= True         # set flag
            for i in range(2):                          # mark all side streets as non existent
                self.getintersection().streets[(self.heading+i+1)%8] = STATUS.NONEXISTENT  
                self.getintersection().streets[(self.heading-i-1)%8] = STATUS.NONEXISTENT
    
    # Update map with the street found after executing a turn
    def update_using_turn(self, direction, turn_angle, hold):
        
        self.calcturn(turn_angle, direction)            # update heading 

        # Set loop parameters based on the direction of turn 
        start=hold                  # previous robot heading before the turn
        end=self.heading            # updated robot heading after the turn

        # Adjustments for mod 8
        if direction =="left" and self.heading<=hold:
            end+=8
        elif direction=="right" and self.heading>=hold:
            start+=8

        i= 1 if direction=="left" else -1       # loop increment

        # Update all robotheadings in between hold and robotheading as NONEXISTENT
        for h in range(start+i, end, i):
            if self.getintersection().streets[h%8]==STATUS.UNKNOWN:     # update only if previously UNKNOWN
                self.getintersection().streets[h%8] = STATUS.NONEXISTENT  
        
        ### ASSUMING NO TWO STREETS AT 45 DEGREE GAP !!! ###
        self.getintersection().streets[(self.heading+1)%8] = STATUS.NONEXISTENT  
        self.getintersection().streets[(self.heading-1)%8] = STATUS.NONEXISTENT
        ### ASSUMING NO TWO STREETS AT 45 DEGREE GAP !!! ###

        if self.getintersection().streets[self.heading] == STATUS.UNKNOWN:           # mark current robotheading as UNEXPLORED
            self.getintersection().streets[self.heading]=STATUS.UNEXPLORED

    # Update blockages before and after a drive   
    def update_using_checkahead(self, obstacle_ahead):

        if self.getintersection().streets[self.heading] not in [STATUS.UNKNOWN, STATUS.NONEXISTENT] and obstacle_ahead== ("OBSTACLE"):  # existing street blocked
            
            self.getintersection().blocked[self.heading] = True     # update blockage
            self.turned= False                                      # update flag to allow multiple turns if the street is blocked

            # Update blockage for the entire CONNECTED street (if exists) 
            if (self.xpose+self.delta_xy[self.heading][0], self.ypose+self.delta_xy[self.heading][1]) in self.intersections:
                self.getintersection(self.xpose+self.delta_xy[self.heading][0], self.ypose+self.delta_xy[self.heading][1]).blocked[(self.heading+4)%8]=True
        
        elif obstacle_ahead == ("CLEAR") and self.getintersection().blocked[self.heading] ==True:   # previously blocked, now unblocked

            self.getintersection().blocked[self.heading] = False    

            # Update blockage for the entire CONNECTED street (if exists)
            if (self.xpose+self.delta_xy[self.heading][0], self.ypose+self.delta_xy[self.heading][1]) in self.intersections:
                self.getintersection(self.xpose+self.delta_xy[self.heading][0], self.ypose+self.delta_xy[self.heading][1]).blocked[(self.heading+4)%8]=False
    
    # Update information about the existence of a street ahead using information obtained from the pullforward detector
    def update_using_pullforward(self, street_ahead):
    
        if street_ahead=="STREET":  # street exists

            if self.getintersection().streets[self.heading] not in [STATUS.CONNECTED,STATUS.DEADEND]:   # only update if previously UNKNOWN
                self.getintersection().streets[self.heading]=STATUS.UNEXPLORED

            ### ASSUMING NO TWO STREETS AT 45 DEGREE GAP !!! ###
            self.getintersection().streets[(self.heading+1)%8] = STATUS.NONEXISTENT  
            self.getintersection().streets[(self.heading-1)%8] = STATUS.NONEXISTENT
            ### ASSUMING NO TWO STREETS AT 45 DEGREE GAP !!! ###

        elif street_ahead=="NOSTREET":  # no street ahead 

            self.getintersection().streets[self.heading] = STATUS.NONEXISTENT
    
    # Visualize map: Saves latest image in map.png
    def show(self):  

        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.gca().set_xlim(-5.5, 4.5)
        plt.gca().set_ylim(-1.5, 4.5)
        plt.gca().set_aspect('equal')

        # Show all the possible locations.
        for x in range(-5, 5):
            for y in range(-1, 5):
                plt.plot(x, y, color='lightgray', marker='o', markersize=8)

        plt.plot(self.xpose, self.ypose, color='black', marker='o', markersize=8)   # current location of the bot

        # Arrow to show current heading of the bot
        plt.arrow(self.xpose-0.25*self.delta_xy[self.heading][0], self.ypose-0.25*self.delta_xy[self.heading][1], 0.5*self.delta_xy[self.heading][0], 0.5*self.delta_xy[self.heading][1],
                    width=0.2,
                    head_width=0.3,
                    head_length=0.1,
                    color='magenta')
        
        # Draw lines for each possible street.
        for (x, y) in self.intersections:
           
            intersection = self.intersections[(x, y)]
           
            if intersection.direction in [0,1,2,3,4,5,6,7]: # direction exists if a dijkstra had been run. 

                # Show directions set by dijkstras   
                plt.arrow(x, y, 0.5*self.delta_xy[intersection.direction][0], 0.5*self.delta_xy[intersection.direction][1],
                        width=0.1,
                        head_width=0.2,
                        head_length=0.1,
                        color='orange')

            # Show each of the 8 streets with the associated colors 
            for h in range(8):      
                dx= 0.5*self.delta_xy[h][0]
                dy= 0.5*self.delta_xy[h][1]
                xfrom, yfrom = x, y
                xto, yto = x + dx, y + dy
                c = self.colors[intersection.streets[h].name]
                if intersection.blocked[h]:
                    c='orange'
                plt.plot([xfrom, xto], [yfrom, yto], color=c)

        # Plot the goal location   
        if self.goal!=(np.nan, np.nan):  
            plt.plot(self.goal[0], self.goal[1], color='blue', marker='o', markersize=9)

        # Save the graph
        plt.savefig('map.png')
        
    # Clear existing treees (costs, directions) of all intersections
    def cleartree(self): 
        for intersection in self.intersections.values():
            intersection.reset_cost()
            intersection.reset_direction()
    
    # Dijkstra's algorithm: 
    def dijkstras(self, to_check=[]): 
        '''
        to_check is a list of tuples of yet-to-be explored intersections with possible paths to the goal
        goal is the tuple of coordinates of the actual goal
        '''
        self.cleartree()    # clear existing trees
        on_deck=[]          # on_deck queue

        if self.goal in self.intersections:  # goal exists and known, proceed normally
            on_deck = [self.getintersection(self.goal[0], self.goal[1])]   # initialize on deck queue with the goal intersection
            self.getintersection(self.goal[0], self.goal[1]).set_cost(0)   # set goal cost to zero
            self.getintersection(self.goal[0], self.goal[1]).set_direction(None)   # set goal direction to None

        # For use in directed exploration
        for (x,y) in to_check:

            #if abs(self.goal[0]-x)==abs(self.goal[1]-y) or abs(self.goal[0]-x)==0 or abs(self.goal[1]-y)==0:  #there might be a direct path
            cost=math.sqrt((self.goal[0]-x)**2+(self.goal[1]-y)**2) # guess Euclidean distance
            # else: #go with manhattan distance
            #cost=abs(self.goal[0]-x)+abs(self.goal[1]-y)
            self.getintersection(x,y).set_cost(cost)
            self.getintersection(x, y).set_direction(None) 
            on_deck.append(self.getintersection(x, y))

        on_deck.sort()  # sort by cost

        while on_deck:

            intersection = on_deck.pop(0)   # pop the first element from the on_deck queue - it surely has its optimal cost
            
            for h in range(8): 
                
                # Implement the algorithm for all intersections with CONNECTED and unblocked streets from the current
                if not intersection.blocked[h] and intersection.streets[h] == STATUS.CONNECTED:   

                    neighbor=self.getintersection(intersection.x+self.delta_xy[h][0],intersection.y+self.delta_xy[h][1])   # get the CONNECTED neighbor 
                    neighborcost = 1.4 if h%2 else 1    # cost to travel to neighbor from the current intersection 
                    cost = intersection.cost + neighborcost     # potential cost of neighbor from the goal
                    direction = (h+4)%8     # potential direction from neighbor to current intersection

                    if cost < neighbor.cost:        # better cost found
                        neighbor.set_cost(cost)
                        neighbor.set_direction(direction)
                        if neighbor in on_deck:
                            on_deck.remove(neighbor)    # temporarily remove if it's already on on_deck
                        bisect.insort(on_deck, neighbor)

        self.show() # visualize map with Dijkstra's arrows

if __name__ == "__main__":
   
   # setup GPIOs
   print("Setting up the GPIO...")
   io = pigpio.pi()
   if not io.connected:
       print("Unable to connection to pigpio daemon!")
       sys.exit(0)
   print("GPIO ready...")


        

    