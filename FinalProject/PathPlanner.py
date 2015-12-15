__author__ = 'Troy Hughes'

from turtleExceptions import PathPlannerException
import Queue
import tools
import rospy

class PathPlanner():
    def __init__(self, name):
        self._name_ = name
        print "Path Planner Initialized"


    def canTravelTo(self, givenMap, currentPosition,goalPosition):
        """
        This function returns a boolean if there is no safe path between the goal and the current location
        :param givenMap: The map to search for a location
        :param currentPosition: The current position of the robot on the givenMap
        :param goalPosition: The goal position on the givenMap.
        :return: Boolean of if it can travel or not.
        """
        try:
            val = self._getPath(givenMap, currentPosition,goalPosition)
            return True
        except PathPlannerException,e:
            print "Failed in canTravelTo, exception: ", e
            return False


    def frontierToTravelPoint(self, givenMap, currentPosition, goalPosition):
        """
        This function takes in a map, the robot's position and the goal.
        Returns a point that is along the generated path, towards the goalPosition.

        :param givenMap: The current map
        :param currentPosition: The current position of the robot
        :param goalPosition: The goal the robot is trying to path to
        :return travelPoint: Returns the point we're actually trying to go to.
        """
        try:
            path = self._getPath(givenMap, currentPosition, goalPosition)
            path.reverse()

            # f = open("pathInfo.txt","r+")
            # for i in path:
            #     x,y = i
            #     f.write("The point was: "+str(i)+" and it's value was "+str(givenMap[y][x]))
            # f.close()

            if len(path) > 40: backUpDist = 20
            else: backUpDist = 10

            print "The passed point is: ", currentPosition
            travelPoint = path.pop()
            x,y = travelPoint
            counter = 0
            while len(path) > 0 and not rospy.is_shutdown():
                print "Looking for a value..."
                if givenMap[int(y)][int(x)] >= 0:
                    counter+=1
                    if counter > backUpDist:
                        print "Your point is: ", (int(x),int(y)), givenMap[int(y)][int(x)]
                        return travelPoint
                else:
                    counter = 0
                travelPoint = path.pop()
                x,y = travelPoint
            raise PathPlannerException("Failed to find a path")
        except Exception,e:
            print "Something went wrong: "
            raise e


    def _getWaypoint(self, givenMap, start, goal):
        """
        This function gets waypoints where there are changes in direction along a pathway.
        Depreciated code, because Troy loves Odell

        :param givenMap:
        :param start:
        :param goal:
        """

        pathToNodify = self._getPath(givenMap, start, goal) # Get path from A*
        nodePath = []  # What will become the path of only relevant nodes.
        prevSlope = 1337.0  # Create an impossible to match previous slope

        for node in range (0, len(pathToNodify)-1): # For every node in the path created by A*:
            if pathToNodify[node] != goal: # Do not try to calculate a slope for the last node
                #calculate current slope, from current position to next position.
                deltax = 1.0*(pathToNodify[node][0]-pathToNodify[node+1][0])#Calcualte delta x based on node and the node ahead
                deltay = 1.0*(pathToNodify[node][1]-pathToNodify[node+1][1])#calculate delta y
                currentSlope = deltax/(deltay+0.000001) #So that we don't have devide by 0 errors. Dunno how else to fix
                if currentSlope != prevSlope: # If we see a slope change
                    nodePath.append(pathToNodify[node])# Add the new node
                    prevSlope=currentSlope # Update th slope

        #Append the very last node, so that we have a complete path.
        nodePath.append(pathToNodify[len(pathToNodify)-1])
        return nodePath


    def _getPath(self, givenMap, start, goal):
        """
        This function uses A* to generate a from start to end and then returns it.
        :param start:
        :return: list of points (x,y touples) that is the path to take to the goal
        """
        try:
            came_from = self._aStarSearch(givenMap, start, goal)          ## Get dictionary from astar
        except PathPlannerException,e:
            print "Path planning exception when trying to find path"
            return false

        path = [goal]                      ## Initialize path

        ## Prime the statements for iterating to the end of the path
        current_location = goal
        prev = came_from[current_location]
        path.append(prev)
        current_location = prev

        ## Traverse the tree from leaves to trunk
        while prev is not None:
            prev = came_from[current_location]
            if prev is None:
                continue
            path.append(prev)
            current_location = prev

        path.reverse()                              ## Reverse the list so it goes from start to finish
        return path

    def _aStarSearch(self, givenMap, start, goal, threshold = 70):
        """
        This function generates a dictionary of paths where the shortest path can be found through
        traversing from the goal back to the start.

        This version of A* uses tools.distFormula as the heuristic. Could be changed out.

        :param start: (x,y) touple of the location in the grid.
        :return: Dictionary of <end point> : <starting Point> as key,value
        """
        frontier = Queue.PriorityQueue()
        frontier.put((0, start))            ## Put takes a touple of priority, value
        goalX, goalY = goal

        came_from = {}
        cost_so_far = {}

        came_from[start] = None
        cost_so_far[start] = 0
        found_goal = False

        while not frontier.empty():
            current_touple = frontier.get()                                             ## Returns the (priority, (x,y)
            _,current = current_touple                                                  ## sets current to (x,y)
            x,y = current

            if x == goalX and y == goalY:
                found_goal = True
                break #We're there!

            #unpack current
            neighbors = tools.getNeighbors(x,y,givenMap,threshold=threshold)
            # print neighbors
            for next in neighbors:                                         ## Get list of touples (points)
                # print "AStar",next,current
                cost_next = cost_so_far[current] + tools.distFormula(current, next)
                if next not in cost_so_far or cost_so_far[next] > cost_next:
                    if next not in cost_so_far:
                        frontier.put((cost_next + tools.distFormula(next, goal), next))## Put takes a touple of priority, value
                        nx,ny = next
                    cost_so_far[next] = cost_next
                    came_from[next] = current


        if found_goal:
            return came_from
        else:
            raise PathPlannerException("The path is not travelable")

