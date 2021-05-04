#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap, GetMapResponse
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler 
from priority_queue import PriorityQueue #importing PriorityQueue class to be used
from copy import deepcopy

class Frontier:
     
    def __init__(self):
        rospy.init_node("frontier")

        ## subscribe to the map topic
        ## When a message is published, update the map
        rospy.Subscriber('/map', OccupancyGrid ,self.updateMap)

        rospy.sleep(.5)

        ## Publish GridCells to the cspace topic
        self.pubCSpace = rospy.Publisher('/path_planning/cspace', GridCells, queue_size=10)
        self.cSpaceService = rospy.Service('cspace', GetMap, self.calc_cspace)
        self.pubFrontierLine = rospy.Publisher('frontierLine', GridCells, queue_size=10)

        rospy.sleep(.5)

        rospy.loginfo('Init completed')

        
        
    def updateMap(self, msg):
        '''
        Set the map attribute to the current version of the occupancy grid
        :param msg [OccupandyGrid] The new map to be stored
        '''
        print('updating the map')
        self.map = msg
    


    def getFrontier(self):
        THRESH = 50     #The threshold for something being considered an obstacle
        frontierWorldCoords = []   #initialize a list for storing frontier cell values
        #Get a copy of the current map with the calculated cspace for further editing
        frontierMap = deepcopy(self.calc_cspace(None))

        frontierMap = list(frontierMap.data) #Make a list out of it so its changable
        
        #Iterate over the stored map
        for y in range(self.map.info.height):
            for x in range(self.map.info.width):
                #If any cells have an unknown neighbor
                if self.hasUnknownNeighbor(x,y):
                    #See if the unkown cell has any walkable neigbors
                    for each in self.neighbors_of_4(x,y):
                        #If the cell is walkable
                        if self.is_cell_walkable(each[0], each[1]):
                            frontierMap[self.grid_to_index(x,y)] = 100              #Set the cell to an obstacle (frontier line)
                            frontierWorldCoords.append(self.grid_to_world(x,y))     #Add the coordinates of the unknown cell to the woorld coords grid

        msg = GridCells()                                   #Create GridCells Object
        msg.cell_height = mapdata.info.resolution           #dims are equal to map resolution
        msg.cell_width = mapdata.info.resolution
        msg.cells = frontierWorldCoords                     #Set cell data to the world coordinates of frontier cells
        msg.header.frame_id = self.map.header.frame_id      #Copy over frame id
        self.pubFrontierLine.publish(msg)                   #Publish to topic for visualization in Rviz

        frontierMap.data = frontierMap  #Set the copy maps data to the data showing only frontiers

        return frontierMap  #Return for use in other functions



    def grid_to_index(self, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        index = y * self.map.info.width + x
        return index



    def calc_cspace(self,msg):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        try:
            padding = 1
            THRESH = 50                     #Threshold for shading of a cell
            cspaceMap = list(self.map.data)     #Create a copy of existing map to expand obstacles with. Make list so its changeable
            worldCoordinates = []           #Initialize world coordinate list of obstacles

            rospy.loginfo("Calculating C-Space")

            ## Determine cspace for each layer of padding
            for i in range(padding):
                #print(i)
                ## Go through each cell in the occupancy grid (range used to start on row/col 0)
                for y in range(self.map.info.height):
                    for x in range(self.map.info.width):
                        ## Inflate the obstacles where necessary
                        if self.map.data[self.grid_to_index(x, y)] >= THRESH: 
                            cspaceMap[self.grid_to_index(x, y)] = 100       #Set to 100 to make it 100% an obstacle
                            neighbors = self.neighbors_of_8(x, y)           #Get all walkable cells that neighbor main cell
                            for each in neighbors:
                                cspaceMap[self.grid_to_index(each[0], each[1])] = 100  #Set cell to an obstacle in the map copy

                print('Found all the cspace for padding layer ' + str(i+1) + ' out of ' + str(padding))
                self.map.data = deepcopy(cspaceMap)   #Set the mapdata to the new map for use in recursion. 
            
            ## Convert cspace coordinates to world coordinates
            for y in range(self.map.info.height):
                for x in range(self.map.info.width):
                    if cspaceMap[self.grid_to_index(x, y)] >= THRESH:  #If an obstacle is detected
                        worldPoint = self.grid_to_world(x, y)          #Make a worldpoint out of it
                        worldCoordinates.append(worldPoint)            #append to list in order
            
            ## Create a GridCells message and publish it
            msg = GridCells()                               #Create GridCells Object
            msg.cell_height = self.map.info.resolution      #dims are equal to map resolution
            msg.cell_width = self.map.info.resolution
            msg.cells = worldCoordinates                    #Set cell data to the world coordinates of obstacles
            msg.header.frame_id = self.map.header.frame_id  #Copy over frame id
            self.pubCSpace.publish(msg)                     #Publish to topic

            occGridCSpace = OccupancyGrid()
            occGridCSpace.header.frame_id = self.map.header.frame_id
            occGridCSpace.info = self.map.info
            occGridCSpace.data = cspaceMap
            self.c_space = occGridCSpace

            ## Return the C-space
            return occGridCSpace
        except Exception as e:
            print('failed on calc_cspace')
            print(e)
            return None



    def neighbors_of_4(self, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        ### REQUIRED CREDIT
        
        #if the input values are greater than the mapdata, or less than 0, then
        # an exception is thrown
        if (x<0 or x> self.map.info.width-1 or y<0 or y>self.map.info.height-1):
            raise ValueError("Out of Bounds!")

        availibleSpaces = []

        #If x is not the value next to the boarder
        if (x!=self.map.info.width-1):
            #Check is cell is walkable
            if (self.is_cell_walkable(x+1, y)):
                #If cell can be reached, add it to the list of avaible spaces
                availibleSpaces.append((x+1,y))

        #If the x val is not the 0 boundary
        if (x!=0):
            #Check is cell is walkable
            if(self.is_cell_walkable(x-1, y)):
                #If cell can be reached, add it to the list of avaible spaces
                availibleSpaces.append((x-1,y))
        
        #If y is not the value next to the boarder
        if (y!=self.map.info.height-1):
            #Check is cell is walkable
            if (self.is_cell_walkable(x, y+1)):
                #If cell can be reached, add it to the list of avaible spaces
                availibleSpaces.append((x,y+1))

        #If the y val is not the 0 boundary
        if (y!=0):
            #Check is cell is walkable
            if(self.is_cell_walkable(x, y-1)):
                #If cell can be reached, add it to the list of avaible spaces
                availibleSpaces.append((x,y-1))

        return availibleSpaces



    def neighbors_of_8(self, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        ### REQUIRED CREDIT

        availibleSpaces = self.neighbors_of_4(x, y)

        if(x!=0 and y!=0):
            if(self.is_cell_walkable(x-1,y-1)):
                availibleSpaces.append((x-1,y-1))

        if(x!=self.map.info.width-1 and y!=self.map.info.height-1):
            if(self.is_cell_walkable(x+1,y+1)):
                availibleSpaces.append((x+1,y+1))

        if(x!=self.map.info.width-1 and y!=0):
            if(self.is_cell_walkable(x+1,y-1)):
                availibleSpaces.append((x+1,y-1))

        if(x!=0 and y!=self.map.info.height-1):
            if(self.is_cell_walkable(x-1,y+1)):
                availibleSpaces.append((x-1,y+1))

        return availibleSpaces



    def grid_to_world(self, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        #XyCoordinates to be returned
        xyCoord = Point()

        #Assign coord values from grid values
        xyCoord.x = (x + 0.5) * self.map.info.resolution + self.map.info.origin.position.x
        xyCoord.y = (y + 0.5) * self.map.info.resolution + self.map.info.origin.position.y
        return xyCoord



    def is_cell_walkable(self, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        xLim = self.map.info.width -1
        yLim = self.map.info.height -1
        
        xRange = range(0,xLim)
        yRange = range(0,yLim)
        
        freeThreshold = 10

        if(x in xRange and y in yRange):
            index = self.grid_to_index(x,y)
            if self.map.data[index] < freeThreshold:
                return True
        
        return False



    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        print('Running frontier_cspace.py')
        self.getFrontier()
        print('getFrontier Ran')
        rospy.spin()



if __name__ == '__main__':
    Frontier().run()
