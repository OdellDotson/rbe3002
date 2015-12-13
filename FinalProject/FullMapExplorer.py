__author__ = 'rbe'


import tools
import math
import rospy
from turtleExceptions import FrontierException


class FME():
    def __init__(self, cellSize, robotSize):
        self.numDialations = int(math.ceil(robotSize/cellSize))+2
        self._minFrontierSize = 5
        self._frontierCacheName = 'FrontierFiles.txt'


    def virtualGetFrontierList(self,fileName):
        f = open(fileName, 'r')
        stuff = f.read()
        f.close()
        return stuff


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


                if (self.isNodeFrontier(x, y, givenMap) and not isAlreadyFound):
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

        :return: <int> representing the number of 'points' in the frontier
        """
        return len(targetFrontier)
        # TODO: Currently this just picks whichever point in the frontier is closest and goes there. Maybe not the best?

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
            for elt in frontierList:
                # Check if the next frontier is a better candidate for travel
                # Assuming a larger Heuristic is better
                if heuristic(targetFrontier) < heuristic(elt):
                    targetFrontier = elt

        print "Found the best frontier, finding the closest point"
        print "Frontier contains ", len(targetFrontier), " nodes"

        ## Find the closest point on the frontier
        currentTarget = targetFrontier[0]
        for elt in targetFrontier:
            # TODO: replace once frontier testing is done
            if tools.distFormula(elt, (cx,cy)) < tools.distFormula(currentTarget, (cx,cy)):
                # if tools.distFormula(elt, (0, 0)) < tools.distFormula(currentTarget, (0, 0)):
                currentTarget = elt
                # TODO: Currently this just picks whichever point in the frontier is closest and goes there. Maybe not the best?

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

        if (givenMap[y][x] > -1 and givenMap[y][x] < 50):
            neighbors = tools.getNeighbors(x, y, givenMap)

            for n in neighbors:
                x, y = n
                if (givenMap[y][x] == -1):
                    return True

        return False


    def getNeighborsFrontier(self, x, y, givenMap):
        """
        Returns all the neighbors of a given node that are frontier nodes
        :param x:
        :param y:
        :return:
        """
        goodNeigbhors = tools.getNeighbors(x, y, givenMap)

        edgeNeighbors = []

        for n in goodNeigbhors:
            nodeX, nodeY = n

            if self.isNodeFrontier(nodeX, nodeY, givenMap):
                edgeNeighbors.append(n)

        return edgeNeighbors


    def findSafePoint(self,goalLocation, givenLocation, givenMap, threshold = 50):
        """
        This function finds a safe point to path to on the givenMap that is below the given threshold.
        If it is not, this will get neighbots until there is a safe value found to map to.
        If it looks for too long, the function will just fail and give back the goal location after raising an error.
        """
        cx,cy = givenLocation
        gx,gy = goalLocation

        return ((cx+gx)/2,(cy+gy)/2)


    def findMidpoints(self,listOfFrontiers):
        x_sum,y_sum = 0,0
        total = 0
        midpoint_list = []
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



    def test(self):
        """
        This function was used for testing to build the prior function from.
        """
        print ""
        print "Starting"
        print ""

        tempMap = [
                [-1, -1, 100, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
                [-1, -1, 100, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
                [-1, -1, 100, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
                [-1, -1, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [-1, -1, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [-1, -1, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [-1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [-1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, 0, 0, 0, 0, 0],
                [-1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, 0, 0, 0, 0, 0],
                [-1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, 0, 0, 0, 0, 0],
                [-1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [-1, -1, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [-1, -1, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [-1, -1, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [-1, -1, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1],
                [-1, -1, 100, 0, 0, 0, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, -1, -1],
                [-1, -1, 100, 0, 0, 0, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, -1, -1, -1],
                [-1, -1, 100, 0, 0, 0, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, -1, -1, -1, -1],
                [-1, -1, 100, 0, 0, 0, -1, -1, -1, -1, -1, -1, 0, 0, 0, -1, -1, -1, -1, -1]
            ]

        self.getFrontierList(tempMap)

        print ""
        print "Finished!"
        print ""

#
# a = FME()
# a.test()
