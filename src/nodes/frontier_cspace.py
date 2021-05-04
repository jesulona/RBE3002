#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler 
from priority_queue import PriorityQueue #importing PriorityQueue class to be used
from copy import deepcopy

class Frontier:
     
     def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("frontier", anonymous = True)

        ## subscribe to the map topic
        ## When a message is published, update the map
        rospy.Subscriber('/map', OccupancyGrid,self.updateMap)

        ## Publish GridCells to the cspace topic
        self.pubCSpace = rospy,Publisher('cspaceTopic', GridCells, queue_size=10)
        
        ## Create a service for calculating cspace
        ## Use type GetMap and call calc_cspace
        self.cSpaceService = rospy.Service('cspace', GetMap, self.calc_cspace)

        
    def updateMap(self, msg):
        '''
        Set the map attribute to the current version of the occupancy grid
        :param msg [OccupandyGrid] The new map to be stored
        '''
        self.map = msg
    
    def getFrontier(self):
        THRESH = 50     #The threshold for something being considered an obstacle
        frontierWorldCoords = []   #initialize a list for storing frontier cell values
        #Get a copy of the current map with the calculated cspace for further editing
        frontierMap = deepcopy(self.calc_cspace())

        frontierMap = list(frontierMap) #Make a list out of it so its changable
        
        #Iterate over the stored map
        for y in self.map.info.height:
            for x in self.map.info.width:
                #If any cells have an unknown neighbor
                if self.hasUnknownNeighbor(x,y):
                    #See if the unkown cell has any walkable neigbors
                    for each in self.neighbors_of_4(x,y):
                        #If the cell is walkable
                        if self.is_cell_walkable(each[0], each[1]):
                            frontierMap[self.grid_to_index(x,y)] = 100              #Set the cell to an obstacle
                            frontierWorldCoords.append(self.grid_to_world(x,y))     #Add the coordinates of the unknown cell to the woorld coords grid

        msg = GridCells()                                   #Create GridCells Object
        msg.cell_height = mapdata.info.resolution           #dims are equal to map resolution
        msg.cell_width = mapdata.info.resolution
        msg.cells = frontierWorldCoords                     #Set cell data to the world coordinates of frontier cells
        msg.header.frame_id = self.map.header.frame_id      #Copy over frame id
        self.pubFrontierLine.publish(msg)                   #Publish to topic for visualization in Rviz


