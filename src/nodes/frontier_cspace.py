#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap, GetMapResponse
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler 
from priority_queue import PriorityQueue #importing PriorityQueue class to be used
from copy import deepcopy
from rbe3002_lab3.srv import frontierList, frontierListResponse, frontierListRequest


class Frontier:
     
    def __init__(self):
        rospy.init_node("frontier")

        rospy.Subscriber('/map', OccupancyGrid ,self.updateMap)

        self.pubCSpace = rospy.Publisher('/path_planning/cspace', GridCells, queue_size=10)     #c space in rviz simulation
        self.pubFrontierLine = rospy.Publisher('frontierLine', GridCells, queue_size=10)        #Frontier in rviz simulation
        self.pubCPoint = rospy.Publisher('/centroid/point',GridCells,queue_size=10)             #centroids of frontier in rviz
        
        rospy.Service('cspace', GetMap, self.calc_cspace)                       #Service used for publishing cspace
        rospy.Service('getFrontiers', frontierList, self.returnCentroids)       #Service used for publishing frontiers
        rospy.sleep(.5)

        rospy.loginfo('Init completed')

    def updateMap(self, msg):
        '''
        Set the map attribute to the current version of the occupancy grid
        :param msg [OccupandyGrid] The new map to be stored
        '''
        self.map = msg


    def returnCentroids(self,msg):
        '''
        Used for centroid service call
        '''
        frontiers = self.getFrontier()
        dilatedFrontiers = self.dilateAndErode(frontiers)
        frontierGroups = self.splitFrontiers(dilatedFrontiers)
        centroidList = self.getCentroids(frontierGroups)
        centroidList = frontierListResponse(centroidList)
        return centroidList
    

    def dilateAndErode(self, frontierGrid):
        '''
        Function dilates existing frontier cells
        :param frontierGrid [Occupancy Grid] The grid of frontier cells to dilate
        :return [Occupancy Grid]
        '''
        padding = 0                                     #Padding for the cells
        dilutionMap = deepcopy(self.map)                #Copy of the map
        dilutionMapData = [0] * len(dilutionMap.data)   #Change every value to 0 for manipulation
        dilWorldCoordinates = []                        #initialize world coordinate list

        THRESH = 90
        #Dilute the cells
        for i in range(padding):
            for y in range(self.map.info.height):
                for x in range(self.map.info.width):
                    if frontierGrid.data[self.grid_to_index(x,y)] >= THRESH:
                        dilutionMapData[self.grid_to_index(x,y)] = 100
                        dilWorldCoordinates.append(self.grid_to_world(x,y))
                        for each in self.neighbors_of_8(x,y,dilutionMap):
                            dilutionMapData[self.grid_to_index(each[0],each[1])] = 100
            frontierGrid.map.data = deepcopy(dilutionMapData)

        #Still add frontier line if padding is 0
        if padding == 0:
            for y in range(self.map.info.height):
                for x in range(self.map.info.width):
                    if frontierGrid.data[self.grid_to_index(x,y)] >= THRESH:
                        dilutionMapData[self.grid_to_index(x,y)] = 100
                        dilWorldCoordinates.append(self.grid_to_world(x,y))
        
        ##Erosion
        # Search through frontier dilution map
        # if its on the edge (aka)
            # One of the neighbors of 8 is empty
        # delete it from the occupancy grid

        #Add frontier cells to the world coordinates list
        for i in range(padding):
            for y in range(self.map.info.height):
                for x in range(self.map.info.width):
                    if dilutionMapData[self.grid_to_index(x,y)] >= THRESH:
                        dilWorldCoordinates.append(self.grid_to_world(x,y))

        ## Check and remove any duplicates from the list of world coords
        dilFrontCoords = []
        for i  in dilWorldCoordinates:
            if i not in dilFrontCoords:
                dilFrontCoords.append(i)  
        
        ## Create a GridCells message and publish it (This is used only for Rviz Visualization)
        msg = GridCells()                               #Create GridCells Object
        msg.cell_height = self.map.info.resolution      #dims are equal to map resolution
        msg.cell_width = self.map.info.resolution
        msg.cells = dilFrontCoords                      #Set cell data to the world coordinates of obstacles
        msg.header.frame_id = self.map.header.frame_id  #Copy over frame id
        self.pubFrontierLine.publish(msg)

        fixedFrontiers = deepcopy(self.map)                       #Create a new occupancy grid object
        fixedFrontiers.data = dilutionMapData
        self.c_space = fixedFrontiers

        ## Return the frontiers after they have been diluted and eroded
        return fixedFrontiers
        

    def getFrontier(self):
        '''
        Function determines the frontier edges present in the map
        Publishes cspace, frontiers and centroids to rviz as well
        :returns occupancy grid with frontiers set to obstacles
        '''
        THRESH = 50     #The threshold for something being considered an obstacle
        frontierWorldCoords = []   #initialize a list for storing frontier cell values
        #Get a copy of the current map with the calculated cspace for further editing
        frontierMap = deepcopy(self.calc_cspace(None))  #Map + cspace
        frontierMapData = [0] * len(frontierMap.data)

        #Iterate over the stored map
        for y in range(self.map.info.height):
            for x in range(self.map.info.width):
                #If any cells are known
                if self.is_cell_walkable(x,y,self.c_space):
                    #See if the unkown cell has any unknwon neigbors
                    if self.has_unknown_neighbors_of_8(x,y):
                        frontierMapData[self.grid_to_index(x,y)] = 100                         #Set the cell to an obstacle (frontier line)
                        frontierWorldCoords.append(self.grid_to_world(x,y))     #Add the coordinates of the unknown cell to the woorld coords grid
        
        # Remove duplicate points from the world coordinate list
        frontCoords = []
        for i  in frontierWorldCoords:
            if i not in frontCoords:
                frontCoords.append(i)    
        
        msg = GridCells()                                 #Create GridCells Object
        msg.cell_height = self.map.info.resolution        #dims are equal to map resolution
        msg.cell_width = self.map.info.resolution
        msg.cells = frontCoords                           #Set cell data to the world coordinates of frontier cells
        msg.header.frame_id = self.map.header.frame_id    #Copy over frame id
        self.pubFrontierLine.publish(msg)                 #Publish to topic for visualization in Rviz
        
        frontierMap.data = frontierMapData
        return frontierMap  #Return for use in other functions


    def index_to_grid(self, index, mapdata):
        '''
        Function converts an occupancy grid index to x and y coordinates
        :param index [int] The index of the variable in the list
        :param mapdata [occupancyGrid] The occupandy grid
        :return [x,y] where x and y are integers representing the coordinates
        '''
        x = index % mapdata.info.width          #Calc x value within list
        y = int(index / mapdata.info.height)    #Calc y value within list
        return [x,y]
   
        
    def recursiveDFS(self, gridMap, point, visitedList, groupList):
        '''
        Function recursively searches nodes for frontier indices using a DFS search algorithm
        :param gridMap [OccupancyGrid] The map to search
        :param point [int, int] A point to seach from
        :param visitedList [list<int>] A list containint all visited indices
        :param grouList [list<list<int>>] A list containing current frontiers and their indices
        :return Nothing, it recursivy searches and updates the lists as it goes
        '''
        index = self.grid_to_index(point[0],point[1])       #Turn the point into a list index
        
        #If the index is a frontier and hasnt been visited
        if gridMap.data[index] == 100 and index not in visitedList:
            groupList[-1].append(index)     #Append to the most recent frontier recorded
            visitedList.append(index)       #Add the index to the visited points
            # Calculate the neighbors of 8 for this cell (DFS)
            for each in self.frontier_neighbors_of_8(point[0],point[1], gridMap):
                self.recursiveDFS(gridMap, each, visitedList, groupList)        #Trust the natural recursion


    def splitFrontiers(self,occGrid):
        '''
        Function takes an occupancy grid and calculates the different frontiers within it.
        Search is modeled after a Depth First Search, and recursively searches map for frontier indexes until all have been logged
        :param occGrid [OccupancyGrid] The occupancy grid containing ONLY frontier data
        :return TBD
        '''
        frontierMap = occGrid.data      #extract frontier data from the map
        visitedCells = []               #keep track of visited cells
        frontierGroups = []             #keep track of current frontier groups
        
        #Iterare through the occupancy grid data
        for each in range(len(frontierMap)):
            #If an index is set to 100 and hasn't been visited
            if frontierMap[each] == 100 and each not in visitedCells:
                frontierGroups.append([])                   #Initialize a new frontier list
                frontierGroups[-1].append(each)             #Append to the end of the current frontier list                    
                visitedCells.append(frontierMap[each])      #Add the index of the frontier
                cell = self.index_to_grid(each, occGrid)    #calculate the x and y coordinates of the index
                #Calculate the neighbors of 8 that are set to 100
                for each in self.frontier_neighbors_of_8(cell[0], cell[1], occGrid):
                    self.recursiveDFS(occGrid, each, visitedCells, frontierGroups)      #Run DFS Algorithm down nodes
        
        print(str(len(frontierGroups)) + ' Found in the current map!')
        return frontierGroups

    def getCentroids(self, list):
        '''
        Function find the centroids in a list of frontiers
        :param  list [list<list<int>>] A list of frontiers according to their index
        :return list [list<Point>]     A list of centroids in the world frame
        '''
        print(list)
        centCells = GridCells()
        centCells.cell_width = self.map.info.resolution
        centCells.cell_height = self.map.info.resolution
        centCells.header.frame_id = self.map.header.frame_id
        centList = []
        for each in list:
            xSum = 0
            ySum = 0
            for i in each:
                cell = self.index_to_grid(i, self.map)
                #worldCoord = self.grid_to_world(cell[0],cell[1])
                xSum = xSum + cell[0] #worldCoord.x
                ySum = ySum + cell[1] #worldCoord.y
            xCentroid = xSum/len(each)
            yCentroid = ySum/len(each)
            worldCentroid = self.grid_to_world(xCentroid, yCentroid)

            point = Point()
            point.x = worldCentroid.x #xCentroid
            point.y = worldCentroid.y #yCentroid
            point.z = len(each)
            if len(each) > 3: #get rid of all frontiers that are only 1 square long
                centCells.cells.append(point)

        self.pubCPoint.publish(centCells)

        return centCells.cells
                


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
            padding = 3
            THRESH = 50                         #Threshold for shading of a cell
            worldCoordinates = []               #Initialize world coordinate list of obstacles
            self.c_space = deepcopy(self.map)
            cspaceMap = list(self.c_space.data)     #Create a copy of existing map to expand obstacles with. Make list so its changeable
            rospy.loginfo("Calculating C-Space")

            ## Determine cspace for each layer of padding
            for i in range(padding):
                #print(i)
                ## Go through each cell in the occupancy grid (range used to start on row/col 0)
                for y in range(self.map.info.height):
                    for x in range(self.map.info.width):
                        ## Inflate the obstacles where necessary
                        if self.c_space.data[self.grid_to_index(x, y)] >= THRESH: 
                            cspaceMap[self.grid_to_index(x, y)] = 100       #Set to 100 to make it 100% an obstacle
                            neighbors = self.neighbors_of_8(x, y, self.map)           #Get all walkable cells that neighbor main cell
                            for each in neighbors:
                                cspaceMap[self.grid_to_index(each[0], each[1])] = 100  #Set cell to an obstacle in the map copy
                #cspaceMap = tuple(cspaceMap)
                self.c_space.data = deepcopy(cspaceMap)   #Set the mapdata to the new map for use in recursion. 
            
            ## Convert cspace coordinates to world coordinates (to avoid duplicates)
            for y in range(self.map.info.height):
                for x in range(self.map.info.width):
                    if cspaceMap[self.grid_to_index(x, y)] >= THRESH:       #If an obstacle is detected
                        worldCoordinates.append(self.grid_to_world(x, y))   #append to list in order
            
            ## Create a GridCells message and publish it
            ## This is used only for Rviz Visualization
            msg = GridCells()                               #Create GridCells Object
            msg.cell_height = self.map.info.resolution      #dims are equal to map resolution
            msg.cell_width = self.map.info.resolution
            msg.cells = worldCoordinates                    #Set cell data to the world coordinates of obstacles
            msg.header.frame_id = self.map.header.frame_id  #Copy over frame id
            self.pubCSpace.publish(msg)                     #Publish to topic

            occGridCSpace = deepcopy(self.c_space)    #Create new Occupancy Grid Object
            occGridCSpace.data = cspaceMap
            self.c_space = occGridCSpace

            ## Return the C-space
            #rospy.sleep(.25)
            return occGridCSpace
        except Exception as e:
            print('failed on calc_cspace')
            print(e)
            return None


    def neighbors_of_4(self, x, y, mapdata):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x, y    [int]           The X and Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        #if the input values are greater than the mapdata, or less than 0, then an exception is thrown
        if not self.isInBounds(x,y):
            raise ValueError("Out of Bounds!")

        availibleSpaces = []

        #If x is not the value next to the boarder
        if (x!=mapdata.info.width-1) and self.is_cell_walkable(x+1, y, mapdata):
            availibleSpaces.append((x+1,y))     #If cell can be reached, add it to the list of avaible spaces

        #If the x val is not the 0 boundary
        if (x!=0) and self.is_cell_walkable(x-1, y,mapdata):
            availibleSpaces.append((x-1,y))     
        
        #If y is not the value next to the boarder
        if (y!=mapdata.info.height-1) and self.is_cell_walkable(x, y+1,mapdata):
            availibleSpaces.append((x,y+1))

        #If the y val is not the 0 boundary
        if (y!=0) and self.is_cell_walkable(x, y-1,mapdata):
            availibleSpaces.append((x,y-1))

        return availibleSpaces


    def frontier_neighbors_of_4(self, x, y, mapdata):
            """
            Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
            :param mapdata [OccupancyGrid] The map information.
            :param x       [int]           The X coordinate in the grid.
            :param y       [int]           The Y coordinate in the grid.
            :return        [[(int,int)]]   A list of walkable 4-neighbors.
            """
            #if the input values are greater than the mapdata, or less than 0, then an exception is thrown
            if not self.isInBounds(x,y):
                raise ValueError("Out of Bounds!")

            availibleSpaces = []

            #If x is not the value next to the boarder
            if (x!=mapdata.info.width-1) and not self.is_cell_walkable(x+1, y, mapdata):
                availibleSpaces.append((x+1,y))     #If cell can be reached, add it to the list of avaible spaces

            #If the x val is not the 0 boundary
            if (x!=0) and not self.is_cell_walkable(x-1, y,mapdata):
                availibleSpaces.append((x-1,y))
            
            #If y is not the value next to the boarder
            if (y!=mapdata.info.height-1) and not self.is_cell_walkable(x, y+1,mapdata):
                availibleSpaces.append((x,y+1))

            #If the y val is not the 0 boundary
            if (y!=0) and not self.is_cell_walkable(x, y-1,mapdata):
                availibleSpaces.append((x,y-1))

            return availibleSpaces


    def has_unknown_neighbors_of_4(self, x, y):
        """
        Returns true if any neighbors of 4 are unknown
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        #if the input values are greater than the mapdata, or less than 0, then an exception is thrown
        if not self.isInBounds(x,y):
            raise ValueError("Out of Bounds!")

        unAvailibleSpaces = []

        #If x is not the value next to the boarder
        if (x!=self.map.info.width-1) and (self.is_cell_not_walkable(x+1, y)):
            unAvailibleSpaces.append((x+1,y))

        #If the x val is not the 0 boundary
        if (x!=0) and (self.is_cell_not_walkable(x-1, y)):
            unAvailibleSpaces.append((x-1,y))
        
        #If y is not the value next to the boarder
        #Note: Code Doesnt work if this formatting is changed. Not sure why
        if (y!=self.map.info.height-1) and (self.is_cell_not_walkable(x, y+1)):
            if (self.is_cell_not_walkable(x, y+1)):
                unAvailibleSpaces.append((x,y+1))

        #If the y val is not the 0 boundary
        if (y!=0) and (self.is_cell_not_walkable(x, y-1)):
            unAvailibleSpaces.append((x,y-1))

        return len(unAvailibleSpaces) is not 0


    def neighbors_of_8(self, x, y,mapdata):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        availibleSpaces = self.neighbors_of_4(x, y, mapdata)

        if(x!=0 and y!=0) and (self.is_cell_walkable(x-1,y-1,mapdata)):
            availibleSpaces.append((x-1,y-1))

        if(x!=mapdata.info.width-1 and y!=mapdata.info.height-1) and (self.is_cell_walkable(x+1,y+1,mapdata)):
            availibleSpaces.append((x+1,y+1))

        if(x!=mapdata.info.width-1 and y!=0) and (self.is_cell_walkable(x+1,y-1,mapdata)):
            availibleSpaces.append((x+1,y-1))

        if(x!=0 and y!=mapdata.info.height-1) and (self.is_cell_walkable(x-1,y+1,mapdata)):
            availibleSpaces.append((x-1,y+1))

        return availibleSpaces


    def frontier_neighbors_of_8(self, x, y,mapdata):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        availibleSpaces = self.frontier_neighbors_of_4(x, y, mapdata)

        if(x!=0 and y!=0) and (not self.is_cell_walkable(x-1,y-1,mapdata)):
            availibleSpaces.append((x-1,y-1))

        if(x!=mapdata.info.width-1 and y!=mapdata.info.height-1) and (not self.is_cell_walkable(x+1,y+1,mapdata)):
            availibleSpaces.append((x+1,y+1))

        if(x!=mapdata.info.width-1 and y!=0) and (not self.is_cell_walkable(x+1,y-1,mapdata)):
            availibleSpaces.append((x+1,y-1))

        if(x!=0 and y!=mapdata.info.height-1) and (not self.is_cell_walkable(x-1,y+1,mapdata)):
            availibleSpaces.append((x-1,y+1))

        return availibleSpaces

    
    def has_unknown_neighbors_of_8(self, x, y):
        """
        Returns true if any neighbors of 8 are unknown
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]
        """
        if self.has_unknown_neighbors_of_4(x, y):
            return True
        else:
            notAvailibleSpaces = []
            
            if(x!=0 and y!=0) and (self.is_cell_not_walkable(x-1,y-1)):
                notAvailibleSpaces.append((x-1,y-1))

            if(x!=self.map.info.width-1 and y!=self.map.info.height-1) and (self.is_cell_not_walkable(x+1,y+1)):
                notAvailibleSpaces.append((x+1,y+1))

            if(x!=self.map.info.width-1 and y!=0) and (self.is_cell_not_walkable(x+1,y-1)):
                notAvailibleSpaces.append((x+1,y-1))

            if(x!=0 and y!=self.map.info.height-1) and (self.is_cell_not_walkable(x-1,y+1)):
                notAvailibleSpaces.append((x-1,y+1))

            return len(notAvailibleSpaces) is not 0  


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



    def is_cell_walkable(self, x, y, mapdata):
        """
        Function returns true if a cell is within the boundary, not unknown, and not an obstacle
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        return self.isInBounds(x,y) and ((mapdata.data[self.grid_to_index(x,y)] == 0) and (mapdata.data[self.grid_to_index(x,y)] is not -1))
    

    def is_cell_not_walkable(self, x, y):
        """
        Function returns true if a cell is within the grid boundaries and is unknown / an obstacle
        :param x, y       [int]        The X and y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        return self.isInBounds(x,y) and ((self.map.data[self.grid_to_index(x,y)] is -1))


    def isInBounds(self, x, y):
        '''
        A cell is in bounds if its within the range of the occupancy grid
        :param x,y [int] The x and y coordinates in the grid
        '''
        return (x in range(0,self.map.info.width - 1)) and (y in range(0,self.map.info.height - 1))


    def world_to_grid(self,mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        try:
            mapPos = mapdata.info.origin.position       #location of pos data in mapdata variable
            mapRes = mapdata.info.resolution            #location of map resolution in variable 
            xPos = int((wp.x - mapPos.x) / mapRes)      #Eq taken from lab document
            yPos = int((wp.y - mapPos.y) / mapRes)
            output = (xPos, yPos)   #Create coordinate for function output
            return output
        except Exception as e:
            print(e)
            print('Failed on world_to_grid()')


    def run(self):  
        rospy.spin()    #Required for ROS


if __name__ == '__main__':
    Frontier().run()
