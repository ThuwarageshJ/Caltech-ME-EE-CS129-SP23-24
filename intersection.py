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

class STATUS(Enum):
    UNKNOWN = 0
    NONEXISTENT = 1
    UNEXPLORED = 2
    DEADEND = 3
    CONNECTED = 4

    def __str__(self):
        return self.name

    def __eq__(self, other):
        return self.value == other.value
    
class Intersection:
    def __init__(self, x, y):
       self.x = x
       self.y = y
       self.streets = [STATUS.UNKNOWN for i in range(8)] 
       self.cost = float('inf')
       self.direction = None
       self.blocked = [False for i in range(8)]
       self.tunnel = False
    
    def reset_cost(self):
       self.cost = float('inf')

    def set_cost(self, cost):
       self.cost = cost

    
    def set_direction(self, direction):
       self.direction=direction

    def reset_direction(self):
       self.direction=None
    
    def __lt__(self, other):
        return self.cost < other.cost
    
    def all_explored(self):
        explored=True
        for h in range(8):
            if ((self.streets[h] in [STATUS.UNEXPLORED, STATUS.UNKNOWN]) and not self.blocked[h]):
                explored= False
                break
        return explored

    def all_known(self):
        known=True
        for h in range(8):
            if ((self.streets[h] in [STATUS.UNKNOWN])):
                known= False
                break
        return known
    
class Map:
    #Define the colors for each status.
    colors = {
        STATUS.UNKNOWN.name: 'black',
        STATUS.NONEXISTENT.name: 'lightgray',
        STATUS.UNEXPLORED.name: 'blue',
        STATUS.DEADEND.name: 'red',
        STATUS.CONNECTED.name: 'green'
    }

    #change in x and y coordinates - used for plotting and calculating bot's new coordinates
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

    def __init__(self, intersections):
        self.intersections = intersections
        #initialize current x and y positions of the robot (to make sure the arrow is not on the plot during initialization)
        self.xpose=0
        self.ypose=0
        self.heading=0
        self.turned=False     #Flag to determine if the bot turned recently
        self.goal =(np.nan, np.nan)

    def clear(self):
        for intersection in self.intersections.values():
            intersection.blocked=[False for i in range (8)]

    def getintersection(self, x=None, y=None):
        if x==None and y==None:
            x=self.xpose
            y=self.ypose
        if (x, y) not in self.intersections:
            self.intersections[(x, y)] = Intersection(x, y)
        return self.intersections[(x, y)]
    
    def update_using_checkahead(self, obstacle_ahead):

        if self.getintersection().streets[self.heading] not in [STATUS.UNKNOWN, STATUS.NONEXISTENT] and obstacle_ahead== ("OBSTACLE"):
            self.getintersection().blocked[self.heading] = True
            self.turned= False 
            if (self.xpose+self.delta_xy[self.heading][0], self.ypose+self.delta_xy[self.heading][1]) in self.intersections:
                self.getintersection(self.xpose+self.delta_xy[self.heading][0], self.ypose+self.delta_xy[self.heading][1]).blocked[(self.heading+4)%8]=True
        elif obstacle_ahead == ("CLEAR") and self.getintersection().blocked[self.heading] ==True:
            self.getintersection().blocked[self.heading] = False
            if (self.xpose+self.delta_xy[self.heading][0], self.ypose+self.delta_xy[self.heading][1]) in self.intersections:
                self.getintersection(self.xpose+self.delta_xy[self.heading][0], self.ypose+self.delta_xy[self.heading][1]).blocked[(self.heading+4)%8]=False
    
    def update_using_pullforward(self, street_ahead): #use data from pullforward behavior to update details about the street ahead
    
        if street_ahead=="STREET":
            if self.getintersection().streets[self.heading]!=STATUS.CONNECTED and self.getintersection().streets[self.heading]!=STATUS.DEADEND:
                self.getintersection().streets[self.heading]=STATUS.UNEXPLORED
            #ASSUMING NO DOUBLE TURNS!!!
            self.getintersection().streets[(self.heading+1)%8] = STATUS.NONEXISTENT  
            self.getintersection().streets[(self.heading-1)%8] = STATUS.NONEXISTENT
            #ASSUMING NO DOUBLE TURNS!!!
        elif street_ahead=="NOSTREET":
            self.getintersection().streets[self.heading] = STATUS.NONEXISTENT

    def calcmove(self):     #update xpose and ypose
        self.xpose=self.xpose+self.delta_xy[self.heading][0]
        self.ypose=self.ypose+self.delta_xy[self.heading][1]

    def calcturn(self, turn_angle, direction, spin = False): #update heading after turning
        
        if turn_angle==0:
            return
        start=self.heading
        if spin:#verify here
            self.heading=(self.heading+4)%8
            return
        
        if direction =='left':
            for h in range(start+1, (start+7)):
                if self.getintersection().streets[h%8] in [STATUS.UNKNOWN]:
                    break
                if self.getintersection().streets[h%8] not in [STATUS.NONEXISTENT, STATUS.UNKNOWN]:
                    self.heading=h%8
                    return
        else:
            for h in range(start-1, (start-7),-1):
                if self.getintersection().streets[h%8] in [STATUS.UNKNOWN]:
                    break
                if self.getintersection().streets[h%8] not in [STATUS.NONEXISTENT, STATUS.UNKNOWN]:
                    self.heading=h%8
                    return
        self.heading=(self.heading+round(turn_angle/45))%8
    
    def show(self):  #function to visualize the map
        plt.clf()
        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.gca().set_xlim(-5.5, 4.5)
        plt.gca().set_ylim(-0.5, 4.5)
        plt.gca().set_aspect('equal')
        # Show all the possible locations.
        for x in range(-5, 5):
            for y in range(0, 5):
                plt.plot(x, y, color='lightgray', marker='o', markersize=8)

        plt.plot(self.xpose, self.ypose, color='black', marker='o', markersize=8)
        #arrow to show current position of the bot
        plt.arrow(self.xpose-0.25*self.delta_xy[self.heading][0], self.ypose-0.25*self.delta_xy[self.heading][1], 0.5*self.delta_xy[self.heading][0], 0.5*self.delta_xy[self.heading][1],
                    width=0.2,
                    head_width=0.3,
                    head_length=0.1,
                    color='magenta')
        
        # Draw lines for each possible street.
        for (x, y) in self.intersections:
           
            intersection = self.intersections[(x, y)]
           
            if intersection.direction in [0,1,2,3,4,5,6,7]: #direction exists if a dijkstra had been run. 

                #show directions set by dijkstras   
                plt.arrow(x, y, 0.5*self.delta_xy[intersection.direction][0], 0.5*self.delta_xy[intersection.direction][1],
                        width=0.1,
                        head_width=0.2,
                        head_length=0.1,
                        color='orange')
            for h in range(8):      #visualize each street
                dx= 0.5*self.delta_xy[h][0]
                dy= 0.5*self.delta_xy[h][1]
                xfrom, yfrom = x, y
                xto, yto = x + dx, y + dy
                c = self.colors[intersection.streets[h].name]
                if intersection.blocked[h]:
                    c='orange'
                plt.plot([xfrom, xto], [yfrom, yto], color=c)
                
        if self.goal!=(np.nan, np.nan):  #plot the goal
            plt.plot(self.goal[0], self.goal[1], color='blue', marker='o', markersize=9)
        # Show the graph and continue.
        plt.savefig('map.png')
        

    def cleartree(self): #reset costs and directions of all intersections
        for intersection in self.intersections.values():
            intersection.reset_cost()
            intersection.reset_direction()
        
    def dijkstras(self, to_check=[]): #function to implement the DIjkstra's algorithm
        #remove all costs and directions
        '''
        to_check is a list of tuples of yet-to-be explored intersections with possible paths to the goal
        goal is the tuple of coordinates of the actual goal
        '''
        self.cleartree()
        on_deck=[]

        if self.goal in self.intersections:  #goal exists and known, proceed normally
            on_deck = [self.getintersection(self.goal[0], self.goal[1])]   #initialize on deck queue with the goal intersection
            self.getintersection(self.goal[0], self.goal[1]).set_cost(0)   #set goal cost to zero
            self.getintersection(self.goal[0], self.goal[1]).set_direction(None)   #set goal direction to None
          #directed explore
        for (x,y) in to_check:
            #if abs(self.goal[0]-x)==abs(self.goal[1]-y) or abs(self.goal[0]-x)==0 or abs(self.goal[1]-y)==0:  #there might be a direct path
            cost=math.sqrt((self.goal[0]-x)**2+(self.goal[1]-y)**2)
            
            # else: #go with manhattan distance
            #cost=abs(self.goal[0]-x)+abs(self.goal[1]-y)
            self.getintersection(x,y).set_cost(cost)
            self.getintersection(x, y).set_direction(None) 
            on_deck.append(self.getintersection(x, y))

        on_deck.sort()

        while on_deck:
            intersection = on_deck.pop(0)   #pop the first element from the on_deck queue - it surely has its optimal cost
            
            for h in range(8):
                if not intersection.blocked[h] and intersection.streets[h] == STATUS.CONNECTED:   
        
                    neighbor=self.getintersection(intersection.x+self.delta_xy[h][0],intersection.y+self.delta_xy[h][1])   #get the connected neighbor 
                    neighborcost = 1.4 if h%2 else 1    #cost to travel to neighbor from the current intersection 
                    cost = intersection.cost + neighborcost     #potential cost of neighbor from the goal
                    direction = (h+4)%8     #potential direction from neighbor to current intersection
                    if cost < neighbor.cost:    
                        neighbor.set_cost(cost)
                        neighbor.set_direction(direction)
                        if neighbor in on_deck:
                            on_deck.remove(neighbor)    #temporarily remove if it's already on on_deck
                        bisect.insort(on_deck, neighbor)

        self.show()

if __name__ == "__main__":
   #setup GPIOs
   print("Setting up the GPIO...")
   io = pigpio.pi()
   if not io.connected:
       print("Unable to connection to pigpio daemon!")
       sys.exit(0)
   print("GPIO ready...")


        

    