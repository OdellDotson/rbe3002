__author__ = 'rbe'


import tools
import math
import Queue
import rospy
from turtleExceptions import FrontierException


class FME():
    def __init__(self, cellSize, robotSize):
        self.numDialations = int(math.ceil(robotSize/cellSize))+2
        self._minFrontierSize = 5
        self._frontierCacheName = 'FrontierFiles.txt'

    def virtualGetFrontierList(self,fileName):
        """
        This function retrieves the frontiers from a given file and returns the
        :param:fileName This is the file that we will retrieve our frontiers from. (Probably 'FrontierFiles.txt' if you forget.)
        """
        frontierFile = open(fileName, 'r')
        frontiers = f.read()
        frontierFile.close()
        return frontiers


    def getFrontierList(self,givenMap,cache=True):
        """
        This function finds and creates the list of distinct frontiers and returns it.
        :return: List of Frontiers, each frontier being a list of <(x,y) touples >
        :return: Raise FrontierException (defined in turtleException file) when there are no more frontiers
        """
        result = []
        for y, row in enumerate(givenMap):
            for x, elt in enumerate(row):
                # check to see if a given node is adjacent to the opposite kind of node
                # and check that it isn't in any frontier yet
                isAlreadyFound = False

                for list in result:
                    if (x, y) in list:
                        isAlreadyFound = True


                if self.isNodeFrontier(x, y, givenMap) and not isAlreadyFound:
                    frontier = []

                    nodesToExplore = []
                    nodesToExplore.append((x,y))

                    while len(nodesToExplore) > 0:
                        currentNode = nodesToExplore.pop()
                        frontier.append(currentNode)

                        for n in self.getNeighborsFrontier(currentNode[0], currentNode[1], givenMap):
                            if not n in frontier and not n in nodesToExplore:
                                nodesToExplore.append(n)

                    result.append(frontier)

        print "Generated Frontier List of length ", len(result)
        frontierList = []
        for frontier in result:
            if len(frontier) > self._minFrontierSize:
                frontierList.append(frontier)
        if cache:
            f = open(self._frontierCacheName,'r+')
            f.write(str(frontierList))
            f.close()


        return frontierList, givenMap

    def frontierSize(self, targetFrontier):
        """
        :param targetFrontier: The frontier, a list of (x,y) points that make up a frontier

        :return: <int> representing the number of 'points' in the frontier, the total size of the frontier.
        """
        return len(targetFrontier)

    def pickFrontier(self, frontierList, heuristic, locationTouple):
        """
        :param frontierList: List of Frontiers , list of list of <(x,y) touple > in grid cell location on the map
        :param heuristic: function that only requires a single frontier to make a decision.
        :return:an (x,y) touple of the point to go to in grid cell location on the map
        """
        cx,cy = locationTouple
        targetFrontier = frontierList[0]

        print "Trying to find the best frontier"

        if len(frontierList) == 0:  # If there are no frontiers
            raise Exception("Passed in an empty frontier list!")

        ## Select the largest frontier
        elif len(frontierList) != 1:
            for elt in frontierList:# Assuming a larger Heuristic is better,
                if heuristic(targetFrontier) < heuristic(elt):# Check if the next frontier is a better candidate for travel
                    targetFrontier = elt

        print "Best frontier found, contains ", len(targetFrontier), " nodes"

        ## Find the closest point on the frontier
        currentTarget = targetFrontier[0]
        for elt in targetFrontier:
            if tools.distFormula(elt, (cx,cy)) < tools.distFormula(currentTarget, (cx,cy)):
                currentTarget = elt # Update which is the closest element in the target frontier.
        print "Closest point found at: ", currentTarget
        return currentTarget

    def isNodeFrontier(self, x, y, givenMap):
        """testSquares
        Returns true if the given coordinate specific a node that:
            Is known reachable space
            Is adjacent to unknown space
        :param x:
        :param y:
        :return:
        """

        if -1 < givenMap[y][x] < 50:
            neighbors = tools.getNeighbors(x, y, givenMap)

            for n in neighbors:
                x, y = n
                if givenMap[y][x] == -1:
                    return True
        return False


    def getNeighborsFrontier(self, x, y, givenMap):
        """
        Returns all the neighbors of a given node that are frontier nodes
        :param x:
        :param y:
        :return:
        """
        goodNeighbors = tools.getNeighbors(x, y, givenMap)

        edgeNeighbors = []

        for n in goodNeighbors:
            nodeX, nodeY = n
            if self.isNodeFrontier(nodeX, nodeY, givenMap):
                edgeNeighbors.append(n)

        return edgeNeighbors



    def findMidpoints(self,listOfFrontiers):
        """
        Given a list of frontiers, this function will return a list of the midpoints of those frontiers, in the same order.

        :param listOfFrontiers: The list of frontiers for which to find midpoints.
        """
        x_sum,y_sum = 0,0
        total = 0
        midpoint_list = []

        if len(listOfFrontiers) == 0:
            raise FrontierException("When trying to find midpoints, there were no frontiers to search from.")

        for frontier in listOfFrontiers:
            for i,point in enumerate(frontier):
                total = i
                x,y = point
                x_sum = x_sum + x
                y_sum = y_sum + y
            midpoint_list.append((math.floor(x_sum/total),
                                  math.floor(y_sum/total)))
            x_sum,y_sum = 0,0
            total = 0
        return midpoint_list


    def sortFrontiers(self, frontierList, heuristic, givenMap, verbose = True):
        """
        :param frontierList: List of Frontiers , list of list of <(x,y) touple > in grid cell location on the map
        :param heuristic: function that only requires a single frontier to make a decision.
        :return:an (x,y) touple of the point to go to in grid cell location on the map
        """

        if verbose: print "Starting ot sort the frontiers"
        if len(frontierList) == 0:  raise Exception("Passed in an empty frontier list!")

        x_sum,y_sum = 0,0
        total = 1
        pq = Queue.PriorityQueue()

        for frontier in frontierList:
            for i,point in enumerate(frontier):
                total = i
                x,y = point
                x_sum = x_sum + x
                y_sum = y_sum + y
            midP = (int(math.floor(x_sum/total)),int(math.floor(y_sum/total)))

            safeMidP = self.findSafePoint(midP,givenMap)

            pq.put((-heuristic(frontier),(safeMidP,frontier)))
            x_sum,y_sum = 0,0
            total = 0

        if verbose: print "A priority queue of frontier midpoints has been created"
        return pq



    def findSafePoint(self, point, givenMap):
        x,y = point
        neighborList = tools.getNeighbors(x,y,givenMap,102)

        for elt in neighborList:
            print "I'm here man... "
            tx, ty = elt
            if givenMap[ty][tx] == 100 or givenMap[ty][tx] ==-1:
                nn=tools.getNeighbors(tx,ty,givenMap,102)
                for i in nn:
                    if not (i in neighborList):
                        neighborList.append(i)
            else:
                return (tx, ty)
        raise FrontierException("Explored the whole map and found no safe spots. Get rekt")



    def queueMaker(self, frontierList, givenMap, currentPosition, verbose = False):
        if verbose: print "Starting ot sort the frontiers"
        if len(frontierList) == 0:  raise Exception("Passed in an empty frontier list!")

        x_sum,y_sum = 0,0
        total = 1
        pq = Queue.PriorityQueue()

        for frontier in frontierList:
            for i,point in enumerate(frontier):
                total = i
                x,y = point
                x_sum = x_sum + x
                y_sum = y_sum + y
            midP = (int(math.floor(x_sum/total)),int(math.floor(y_sum/total)))
            hVal = len(frontier) + tools.distFormula(currentPosition, midP)

            pq.put((-hVal,(midP,frontier)))
            x_sum,y_sum = 0,0
            total = 0

        if verbose: print "A priority queue of frontier midpoints has been created"
        return pq


# a = FME()
# a.test()
