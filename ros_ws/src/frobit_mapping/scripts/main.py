#!/usr/bin/env python
from map_gen import FrobomindMap
import rospy
import numpy

from std_msgs import *
from nav_msgs.msg import *
from geometry_msgs import *
from frobit_mapping.srv import *
from std_msgs.msg import *


class Node:
    def __init__(self):
        self.Map = FrobomindMap(LocalMapSize=15)    
        rospy.init_node('map_database_server')
        metaDataService = rospy.Service('metaDataService',MetaData,self.handle_meta_data_req)
        pixelPosService = rospy.Service('pixelFromPosService',GetPixelFromPosition,self.handle_pixelpos_req)
        getMapSerivice = rospy.Service('getMapService',GetMapService,self.handle_getmap_req)
        setMapSerivice = rospy.Service('setMapService',SetMapService,self.handle_setmap_req)
    def handle_meta_data_req(self,req):
        return MetaDataResponse(self.Map.LocalMapSize,self.Map.gridSize)
    
    def handle_pixelpos_req(self,req):
        res = self.Map.getPixelFromPosition(req.position.translation.x,req.position.translation.y)
        return GetPixelFromPositionResponse(res)
    
    def handle_getmap_req(self,req):
        ret_map = numpy.zeros((self.Map.localMapPixelCount,self.Map.localMapPixelCount),dtype=numpy.int8)
        result = self.Map.getLocalMapIdent(req.x,req.y,ret_map)
        print("Got request for map: %d , %d",req.x,req.y)
        m = OccupancyGrid()
        succes = False
        if result == 0:
            m.info.resolution = self.Map.gridSize
            m.info.width = self.Map.localMapPixelCount
            m.info.height = self.Map.localMapPixelCount
            m.data = list(ret_map.reshape((self.Map.localMapPixelCount**2,)))
            succes = True
            
        else:
            sucess = False
        return {'map':m,'succes':succes}
    
    def handle_setmap_req(self,req):
        data = numpy.array()
        data = req.map.data
        data.reshape(req.map.height,req.map.width)
        print(data)

    
    def run(self):
        rospy.spin()
        
if __name__ == '__main__':
    n = Node()
    n.run()