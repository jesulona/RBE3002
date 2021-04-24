#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler, 
from priority_queue import PriorityQueue #importing PriorityQueue class to be used

class PathPlanner:
    
    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner", anonymous = False)
        
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        self.serv = rospy.Service('plan_path', GetPlan, self.plan_path())
        
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.pubCSpace = rospy.Publisher('/path_planner/cspace', GridCells, queue_size = 10)
        
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.pubAStar = rospy.Publisher('/path_planner/a_star_planning', GridCells, queue_size = 10)
        
        ## Initialize the request counter
        self.requestCounter = 0
        
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")



    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        index = y * mapdata.info.width + x
        return index
       


    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        return math.sqrt((x2-x1)**2+(y2-y1)**2) #Using Pythagorian Thm to find the total travelled distance
        


    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        #XyCoordinates to be returned
        xyCoord = Point()

        #Assign coord values from grid values
        xyCoord.x = (x + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
        xyCoord.y = (y + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y

        return xyCoord
        



    @staticmethod
    def world_to_grid(mapdata, wp):
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


        
    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """

        PoseStampedList = []

        for pathI in range(len(path)):
            yaw = 0

            if pathI < len(path) -1:
                currentPoint = pathI
                nextPoint = pathI +1

                #Calculate distance to final pose
                dX = nextPoint[0] - currentPoint[0]
                dY = nextPoint[1] - currentPoint[1]

                #Calculate initial turn angle 
                angToDest = math.atan2(dY,dX)
            
            #XYZ and QuatStuff
            xyPos = self.grid_to_world(mapdata,everyPath[0],everyPath[1])
            quatArray = quaternion_from_euler(0,0,angToDest)
            quatObj = Quaternion(*quatArray)

            #generate and populate a pose
            aPose = PoseStamped()
            aPose.pose.position = xyPos
            aPose.pose.orientation = quatObj
            aPose.header = mapdata.header
            PoseStampedList.append(aPose)

        return PoseStampedList


    

    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        xLim = mapdata.info.width -1
        yLim = mapdata.info.height -1
        
        xRange = range(0,xLim)
        yRange = range(0,yLim)
        
        freeThreshold = 0.1

        if(x in xRange and y in yRange):
            index = self.grid_to_index(mapdata,x,y)
            if mapdata.data[index] < freeThreshold:
                return True
        
        return False

               

    @staticmethod
    def neighbors_of_4(mapdata, x, y):
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
        if (x<0 or x> mapdata.info.width-1 or y<0 or y>mapdata.info.height-1):
            raise ValueError("Out of Bounds!")

        availibleSpaces = []

        #If x is not the value next to the boarder
        if (x!=mapdata.info.width-1):
            #Check is cell is walkable
            if (PathPlanner.is_cell_walkable(mapdata, x+1, y)):
                #If cell can be reached, add it to the list of avaible spaces
                availibleSpaces.append(x+1,y)

        #If the x val is not the 0 boundary
        if (x!=0):
            #Check is cell is walkable
            if(PathPlanner.is_cell_walkable(mapdata, x-1, y)):
                #If cell can be reached, add it to the list of avaible spaces
                availibleSpaces.append(x-1,y)
        
        #If y is not the value next to the boarder
        if (y!=mapdata.info.height-1):
            #Check is cell is walkable
            if (PathPlanner.is_cell_walkable(mapdata, x, y+1)):
                #If cell can be reached, add it to the list of avaible spaces
                availibleSpaces.append(x,y+1)

        #If the y val is not the 0 boundary
        if (y!=0):
            #Check is cell is walkable
            if(PathPlanner.is_cell_walkable(mapdata, x, y-1)):
                #If cell can be reached, add it to the list of avaible spaces
                availibleSpaces.append(x,y-1)

        return availibleSpaces
        
    
    
    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        ### REQUIRED CREDIT

        availibleSpaces = PathPlanner.neighbors_of_4(mapdata, x, y)

        
        if(x!=0 and y!=0):
            if(PathPlanner.is_cell_walkable(mapdata, x-1,y-1)):
                availibleSpaces.append(x-1,y-1)

        if(x!=mapdata.info.width-1 and y!=mapdata.info.height-1):
            if(PathPlanner.is_cell_walkable(mapdata, x+1,y+1)):
                availibleSpaces.append(x+1,y+1)

        if(x!=mapdata.info.width-1 and y!=0):
            if(PathPlanner.is_cell_walkable(mapdata, x+1,y-1)):
                availibleSpaces.append(x+1,y-1)

        if(x!=0 and y!=mapdata.info.height-1):
            if(PathPlanner.is_cell_walkable(mapdata, x-1,y+1)):
                availibleSpaces.append(x-1,y+1)

        return availibleSpaces


    
    
    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        rospy.loginfo("Requesting the map")     #log info
        try:
            mapServer = rospy.ServiceProxy('static_map', GetMap)    #Request data from the map server
            return mapServer.map    #Get the map parameter from the mapServer object
        except Exception as e:
            print(e)
            print('Failed on world_to_grid()')
            return None
        



    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Calculating C-Space")
        ## Go through each cell in the occupancy grid
        ## Inflate the obstacles where necessary
        # TODO
        ## Create a GridCells message and publish it
        # TODO
        ## Return the C-space
        pass


    
    def a_star(self, mapdata, start, goal):
        """
        Using A* algorithm to calculate an path.
        :param mapdata  [OccupancyGrid] The map data.
        :param start    [(int, int)]    The initial point (a grid coordinate).
        :param goal     [(int, int)]    The goal point (a grid coordinate).
        :return         [[(int, int)]]  The path as a list of tuples (grid coordinates)
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))

        #creating a frontier to use the priority queue class to follow the sudo code
        mapFrontier = PriorityQueue()
        #place start position at the start of the point given
        mapFrontier.put(start,0)

        #Initialize a came_from dict
        came_from = {}
        came_from[start] = None
        #Initialize a cost
        cost= {}
        cost[start] =0 
        
        while (mapFrontier.empty() is False):
            #Get the top Priority from the frontier
            topPriority = mapFrontier.get()

            #generate the 8 neighbors of topPriority
            #for each neighbor:
            for Neighbor in PathPlanner.neighbors_of_8(mapdata, topPriority[0], topPriority[1]):
                gVal = cost[topPriority] #add the topPriority to the cost of where you've been
                #calculate how much it would cost to get to neighbor
                hVal = PathPlanner.euclidean_distance(topPriority[0],topPriority[1],Neighbor[0],Neighbor[1])
                #calculate new total cost
                totalCost = gVal + hVal

                #if neighbor is goal, stop search 
                if (topPriority == goal):
                    break
                
                #if the neighbor is not currently in the path travelled, or the total cost of this neighbor
                #less than the previous paths in cost list
                #expand like a spider web
                if Neighbor not in cost or totalCost < cost[Neighbor]:
                    #set the current neighbor to the totalCost
                    cost[Neighbor] = totalCost
                    #recalculate the hVal
                    hVal = PathPlanner.euclidean_distance(Neighbor[0],Neighbor[1],goal[0],goal[1])
                    #recalulate the total cost as a new variable to not override the previous
                    priority = totalCost + hVal
                    #put the neighbor into the priority list based on the new totalCost, aka it's priority
                    mapFrontier.put(Neighbor, priority)
                    #add the node ot the came_from
                    came_from[Neighbor] = topPriority

        #starting at the goal, making that your current position
        currentPos = goal

        #make a list for the path
        path = []
        #add the start to your path
        path.append(goal)

        while currentPos != start:
            currentPos = came_from[currentPos] 
            path.append(currentPos)

        path.reverse(path)

        return path

    
    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")

        

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Returning a Path message")


        
    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        ## Request the map
        ## In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        ## Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, 1)
        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)


    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
