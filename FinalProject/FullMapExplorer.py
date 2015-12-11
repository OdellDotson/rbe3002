__author__ = 'rbe'


import tools
import math


class FME():
    def __init__(self, cellSize, robotSize):
        self.numDialations = int(math.ceil(robotSize/cellSize))+2



    def getFrontierList(self,givenMap):
        """
        This function finds and creates the list of distinct frontiers and returns it.
        :return: List of Frontiers, each frontier being a list of <(x,y) touples >
        :return: Raise FrontierException (defined in turtleException file) when there are no more frontiers
        """
        newMap = tools.dialateOccupancyMap(givenMap,len(givenMap[0]),len(givenMap))

        for i in xrange(self.numDialations-1):
            newMap = tools.dialateOccupancyMap(newMap,len(givenMap[0]),len(givenMap))

        result = []

        for y, row in enumerate(newMap):
            for x, elt in enumerate(row):
                # check to see if a given node is adjacent to the opposite kind of node
                # and check that it isn't in any frontier yet
                isAlreadyFound = False

                for list in result:
                    if (x, y) in list:
                        isAlreadyFound = True


                if (self.isNodeFrontier(x, y, newMap) and not isAlreadyFound):
                    frontier = []

                    nodesToExplore = []
                    nodesToExplore.append((x,y))

                    while nodesToExplore.__len__() > 0:
                        currentNode = nodesToExplore.pop()
                        frontier.append(currentNode)

                        for n in self.getNeighborsFrontier(currentNode[0], currentNode[1], newMap):
                            if not n in frontier and not n in nodesToExplore:
                                nodesToExplore.append(n)

                    result.append(frontier)

        print "Generated Frontier List of ", result.__len__()

        return result

    def frontierSize(self, targetFrontier):
        """
        :param targetFrontier: The frontier, a list of (x,y) points that make up a frontier

        :return: <int> representing the number of 'points' in the frontier
        """
        return len(targetFrontier)
        # TODO: Currently this just picks whichever point in the frontier is closest and goes there. Maybe not the best?

    def pickFrontier(self, frontierList, heuristic):
        """
        :param frontierList: List of Frontiers , list of list of <(x,y) touple > in grid cell location on the map
        :param heuristic: function that only requires a single frontier to make a decision.
        :return:an (x,y) touple of the point to go to in grid cell location on the map
        """
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
        print "Frontier contains ", targetFrontier.__len__(), " nodes"

        ## Find the closest point on the frontier
        currentTarget = targetFrontier[0]
        for elt in targetFrontier:
            # TODO: replace once frontier testing is done
            # if tools.distFormula(elt, (self.current_x, self.current_y)) < tools.distFormula(currentTarget, (
            if tools.distFormula(elt, (0, 0)) < tools.distFormula(currentTarget, (0, 0)):
                currentTarget = elt
                # TODO: Currently this just picks whichever point in the frontier is closest and goes there. Maybe not the best?

        print "Closest point found at: ", currentTarget

        return currentTarget

    def mapLocationMeters(self, mapLocationGridCells, x,y):
        """
        :param mapLocationGridCells: (x,y) touple of the location in grid cells
        :return:(x,y) touple of the location in meters
        """
        gridx, gridy = mapLocationGridCells

        midx = tools.degmapifyValue(float(gridx)+float(x))/2.0
        midy = tools.degmapifyValue(float(gridy)+float(y))/2.0

        # print "The location you're about to go to is", gridx,gridy,"on the grid and",midx,midy,'on the map'
        return (midx, midy)

    def isNodeFrontier(self, x, y, givenMap):
        """
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


    def findSafePoint(self,goalLocation, givenLocation, givenMap):
        cx,cy = givenLocation
        gx,gy = goalLocation

        neighbors = tools.getNeighbors(gx,gy,givenMap,101)

        return (gx,gy)
        



    def test(self):
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
