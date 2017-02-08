"""
    Implementation of bug2 navigation from Principle of Robot Motion
    Author Matthias Hessels
    Feb 2017
    
"""
import numpy as np
import time
import math
from PIL import Image as img

class Bug2Algorithm:
    def __init__(self):
        self.im = img.open("bug_test.png").convert('L')
        self.current = (0,0)
        self.array = np.array(self.im)
        self.start = (159 , 16)
        self.end = (286 , 174)
        dx = (float(self.end[1])-self.start[1])/(self.end[0]-self.start[0])
        dy = (float(self.end[0])-self.start[0])/(self.end[1]-self.start[1])
        if(dx >= 1.0):    
            self.line = (1 , dy)
        else:
            self.line = (dx , 1)
            
        self.front = (int(math.floor(self.line[0])),int(math.floor(self.line[1])))
        print(self.line)
        self.side = (-1, -1)
        self.get_side_from_front()
        self.last_wall_hit = (-1,-1)
        
        self.turning_dir = 'p'
        
            
    def move(self,direction):
        temp = (self.current[0]+direction[1] , self.current[1]+direction[0])
        self.current = temp
        self.array[int(self.current[1])][int(self.current[0])] = 100
    
    def run_bug(self):

        #Move along line.
        self.current = self.start
        #array = np.full((im.size[1],im.size[0]), 255)
        while (int(self.current[0]) is not self.end[0]) and (int(self.current[1]) is not self.end[1]):
            cur = (self.current[0] + self.line[1] , self.current[1] + self.line[0])
            #time.sleep(0.5)
            if( not (self.array[int(cur[1])][int(cur[0])] == 255)):
                self.follow_wall()
            else:
                self.move(self.line)
        self.visualize()    
                
    def visualize(self):
        self.im = img.fromarray(self.array)
        self.im = self.im.convert('RGB')
        self.im.save("movement.png")
        #time.sleep(0.5)


    def follow_wall(self):
        print("Wall following")
        self.last_wall_hit = self.current
        while(True):
            if self.front_occupied() and self.side_occupied():
                self.side = self.front
                self.turning_dir = 'p'
                self.change_dir()
            elif (not self.front_occupied()) and (not self.side_occupied()):
                self.front = self.side
                self.turning_dir = 'n'
                self.change_dir()
            else:
                self.move(self.front)
            self.visualize()
            if(self.inLine()):
                return
        
    def inLine(self):
        d1 = math.sqrt((self.current[0] - self.start[0])**2+(self.current[1] - self.start[1])**2)
        #print(d1)
        d2 = math.sqrt((self.current[0] - self.end[0])**2+(self.current[1] - self.end[1])**2)
        #print(d2)
        d3 = math.sqrt((self.end[0] - self.start[0])**2+(self.end[1] - self.start[1])**2)
        #print(d3)
        
        delta_d = abs(d3 - (d1 + d2))
        if((delta_d < 0.01) and (not (self.last_wall_hit == self.current))):
            print("On the line again")
            return True
        #Define distance function
        return False
    
    def front_occupied(self):
        if self.array[int(self.current[1]) + self.front[0]][int(self.current[0]) + self.front[1]] == 0:
            return True
        else:
            return False
    
    def side_occupied(self):
        if self.array[int(self.current[1]) + self.side[0]][int(self.current[0]) + self.side[1]] == 0:
            return True
        else:
            return False

    
    def change_dir(self):
        if(self.front == (0,1)):                  # EAST
            if(self.turning_dir == 'p'): 
                self.front = (-1,1)
            else:
                self.side = (1,1)
        elif(self.front == (-1,1)):                # NORTH EAST 
            if(self.turning_dir == 'p'):
                self.front = (-1,0)
            else:
                self.side = (0,1)
        elif(self.front == (-1,0)):                 # NORTH
            if(self.turning_dir == 'p'):
                self.front = (-1,-1)
            else:
                self.side = (-1,1)
        elif(self.front == (-1,-1)):                 # NORTH WEST
            if(self.turning_dir == 'p'):
                self.front = (0,-1)
            else:
                self.side = (-1,0)
        elif(self.front == (0,-1)):                 # WEST
            if(self.turning_dir == 'p'):
                self.front = (1,-1)
            else:
                self.side = (-1,-1)
        elif(self.front == (1,-1)):                 # SOUTH WEST
            if(self.turning_dir == 'p'):
                self.front = (1,0)
            else:
                self.side = (0,-1)
        elif(self.front == (1,0)):                 # SOUTH
            if(self.turning_dir == 'p'):
                self.front = (1,1)
            else:
                self.side = (1,-1)
        elif(self.front == (1,1)):                 # SOUTH EAST
            if(self.turning_dir == 'p'):
                self.front = (0,1)
            else:
                self.side = (1,0)
        else:
            print("You have screwed up")
            
    def get_side_from_front(self):
        if(self.front == (0,1)):                  # EAST
            self.side = (1,1)
        elif(self.front == (-1,1)):                # NORTH EAST 
            self.side = (0,1)
        elif(self.front == (-1,0)):                 # NORTH
            self.side = (-1,1)
        elif(self.front == (-1,-1)):                 # NORTH WEST
            self.side = (-1,0)
        elif(self.front == (0,-1)):                 # WEST
            self.side = (-1,-1)
        elif(self.front == (1,-1)):                 # SOUTH WEST
            self.side = (0,-1)
        elif(self.front == (1,0)):                 # SOUTH
            self.side = (1,-1)
        elif(self.front == (1,1)):                 # SOUTH EAST
            self.side = (1,0)
        else:
            print("NOPE")
 