#!/usr/bin/env python
"""
    Author: Matthias Hessels <matthiashessels@hotmail.com>
    Year: 2017
    This file is part of the master project of Matthias Hessels.
    This file can be copied and/or distributed without the express
    permission of Matthias Hessels, however an email would be appreciated
    
"""


#
import random
import numpy
import math
from PIL import Image

class FrobomindMap:
    def __init__(self,FieldSizeX = 100,FieldSizeY = 100, LocalMapSize = 15,\
                 gridSize = 0.05, fileExtension = '.png',visualize = False):
        #The following numbers are in meters    
        self.FieldSizeX = FieldSizeX  
        self.FieldSizeY = FieldSizeY
        self.LocalMapSize = LocalMapSize
        self.gridSize = gridSize
        
        #Members needed for keeping track of the robot
        self.current_map_center_x = 0
        self.current_map_center_y = 0
        
        self.fileExtension = fileExtension
        
        self.x_size = math.ceil(self.FieldSizeX / float(self.LocalMapSize))
        self.y_size = math.ceil(self.FieldSizeY / float(self.LocalMapSize))

        
        self.localMapPixelCount = (int)(self.LocalMapSize/self.gridSize)
        self.globalMapPixelCountX = (int)((self.LocalMapSize*self.x_size)/self.gridSize)
        self.globalMapPixelCountY= (int)((self.LocalMapSize*self.y_size)/self.gridSize)
        
        self.visualize = visualize
        if visualize:
            self.viz_array = numpy.zeros((3*self.localMapPixelCount,3*self.localMapPixelCount))
        
        self.local_map = numpy.zeros((3*self.localMapPixelCount,3*self.localMapPixelCount))
        
        self.genMap() # Generate the maps on startup. 

        
    def genMap(self): 
        
        #Allocate memory for the map. Since all local maps have the same size,
        # this can be defined once and overwritten
        w = self.localMapPixelCount
        h = self.localMapPixelCount
        Matrix = numpy.zeros((w,h))
        map_number = 0
        im = Image.fromarray(Matrix)
        im = im.convert('RGB')
        for local_map_number_x in range(0,int(self.x_size)):
            for local_map_number_y in range(0,int(self.y_size)):
                filename = "../maps/maps/local_map_" + str(local_map_number_x) + "_" \
                    + str(local_map_number_y) + self.fileExtension

                im.save(filename)
                map_number = map_number + 1
                print("Generated " + str(map_number) + " of " + str((int)(self.x_size * self.y_size)))

    #Gets the local map x_y and puts it into array
    def getLocalMap(self,x_position,y_position,array):
        filename_to_open = "../maps/maps/local_map_" + str(x_position) + "_" + str(y_position) + self.fileExtension
        try:
            im = Image.open(filename_to_open).convert('L')
        except IOError:
            print("Map you are trying to get does not exist")
            return 1
        pix = im.load()
        if not (im.size[0] == im.size[1] == (int)(self.LocalMapSize/self.gridSize)):
            print("Size of the map/image you are trying to load is wrong")
            return -1
        if not (len(array) == len(array[0]) == (int)(self.LocalMapSize/self.gridSize)):
            print("Size of array that you are trying to put picture into is wrong size")
            return -1
        for i in range(0,im.size[0]):
            for j in range(0,im.size[1]):
                if(pix[i,j] > 127):
                    array[i][j] = 127
                else:
                    array[i][j] = pix[i,j]
        return 0
    
    #Debugging function for visualization purposes
    def getLocalMapIdent(self,x_position,y_position,array):
        filename_to_open = "../maps/maps_ident/local_map_" + str(x_position) + "_" + str(y_position) + self.fileExtension
        try:
            im = Image.open(filename_to_open).convert('L')
        except IOError:
            print("Map you are trying to get does not exist")
            return 1 # But still make it a successful call
        
        if not (im.size[0] == im.size[1] == (int)(self.LocalMapSize/self.gridSize)):
            print("Size of the map/image you are trying to load is wrong")
            return -1
        if not (len(array) == len(array[0]) == (int)(self.LocalMapSize/self.gridSize)):
            print("Size of array that you are trying to put picture into is wrong size")
            return -1
        pix = im.load()
        for i in range(0,im.size[0]):
            for j in range(0,im.size[1]):
                if(pix[i,j] > 127):
                    array[i][j] = 127
                else:
                    array[i][j] = pix[i,j]
        return 0
        
        
    def getLocalMapWithOffset(self,x_position,y_position,array,x_offset,y_offset):
        filename_to_open = "../maps/maps/local_map_" + str(x_position) + "_" + str(y_position) + self.fileExtension
        im = Image.open(filename_to_open).convert('L')
        pix = im.load()
        if not (im.size[0] == im.size[1] == (int)(self.LocalMapSize/self.gridSize)):
            print("Size of the map/image you are trying to load is wrong")
            return -1
        if(x_offset < 0 or y_offset < 0):
            print("Offsets cannot be negative, x_offset: " + str(x_offset) +\
                  ", y_offset: " + str(y_offset))
            return -1
        if((x_offset + self.localMapPixelCount > self.globalMapPixelCountX) \
            or (y_offset + self.localMapPixelCount > self.globalMapPixelCountY)):
            print("These offsets will make the map go out of bounds.")
            return -1
        
        for i in range(0,im.size[0]):
            for j in range(0,im.size[1]):
                try:
                    array[i + x_offset][j + y_offset] = pix[j,i] # Switch i and j, otherwise the picture will be flipped 90 degrees
                except IndexError:
                    print("Array index out of bounds. Check if array is big enough for the data to be loaded")
                    return - 1
        return 0
    
        def getLocalMapWithOffsetIdent(self,x_position,y_position,x_offset,y_offset,array):
            filename_to_open = "../maps/maps_ident/local_map_" + str(x_position) + "_" + str(y_position) + self.fileExtension
            im = Image.open(filename_to_open).convert('L')
            pix = im.load()
            if not (im.size[0] == im.size[1] == (int)(self.LocalMapSize/self.gridSize)):
                print("Size of the map/image you are trying to load is wrong")
                return -1
            if(x_offset < 0 or y_offset < 0):
                print("Offsets cannot be negative, x_offset: " + str(x_offset) +\
                      ", y_offset: " + str(y_offset))
                return -1
            if((x_offset + self.localMapPixelCount > self.globalMapPixelCountX) \
                or (y_offset + self.localMapPixelCount > self.globalMapPixelCountY)):
                print("These offsets will make the map go out of bounds.")
                return -1
            
            for i in range(0,im.size[0]):
                for j in range(0,im.size[1]):
                    try:
                        array[i + x_offset][j + y_offset] = pix[j,i] # Switch i and j, otherwise the picture will be flipped 90 degrees
                    except IndexError:
                        print("Array index out of bounds. Check if array is big enough for the data to be loaded")
                        return - 1
            return 0
    
    
    def writeLocalMap(self,x_position,y_position,array):
        filename_to_open = "../maps/maps/local_map_" + str(x_position) + "_" + str(y_position) + self.fileExtension
        Matrix = numpy.zeros((self.localMapPixelCount,self.localMapPixelCount))
        for i in range(0,self.localMapPixelCount):
            for j in range(0,self.localMapPixelCount):
                Matrix[i][j] = array[j][i]
        try:
            im = Image.fromarray(Matrix)
            im = im.convert('RGB')
            im.save(filename_to_open)
        except IOError:
            print("Local map cannot be witten, x_position {} or y_position {} might be wrong".format(x_position,y_position))
            return -1
        
        return 0
    
    def visualize_now(self,center_x,center_y):
        if not self.visualize:
            print("Visualization is not possible for this instance of the FrobomindMap object.\
                  \nSpecifiy in constructor that visualization is wanted")
            return -1
        Matrix = numpy.zeros((self.localMapPixelCount,self.localMapPixelCount))
        for i in range(-1,2):
            for j in range(-1,2):
                status = self.getLocalMapIdent(center_x+i,center_y+j,Matrix) # Takes from Ident at the moment for debugging purposes.
                if (status is not 0):
                    Matrix = numpy.zeros((self.localMapPixelCount,self.localMapPixelCount))
                self.__add_data(self.localMapPixelCount*(i+1),self.localMapPixelCount*(j+1),Matrix)
        im = Image.fromarray(self.viz_array)
        im = im.convert('RGB')
        im.save("current_map_in_memory.png")
        
    def __add_data(self,pos_x,pos_y,data):
        for i in range(0,self.localMapPixelCount):
            for j in range(0,self.localMapPixelCount):
                self.viz_array[i + pos_x,j + pos_y] = data[i][j]
    
    # Define a service that calls this funtion, that takes the current position as input and responds with the pixel and map number
    def getPixelFromPosition(self,x,y): 
        return_list =  []
        g_x = math.floor(x / self.gridSize)
        g_y = math.floor(y / self.gridSize)
        
        # Position 0,1 correspond to map_number [x,y], position 2,3 are the pixel offset in those maps
        return_list.append(math.floor(g_x / self.localMapPixelCount))
        return_list.append(math.floor(g_y / self.localMapPixelCount))
        return_list.append(g_x % self.localMapPixelCount)
        return_list.append(g_y % self.localMapPixelCount)
        
        return return_list


