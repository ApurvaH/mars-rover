from bfs import bfs_algorithm
from ucs import ucs_algorithm, childEvaluationForUCS
from a_star import astar_algorithm, childEvaluationForAStar

class MarsPathFinder:
    # Common data structures
    visitedNodeSet = set()  # List of path coordinates that have already been visited
    nodeToParentMap = {}  # Dictionary containing node as keys and their corresponding parent coordinates as values
    priorityQueue = []  # Priority queue for UCS and A*

    def __init__(self, inputFilePath):
        # Read parameters from input file
        self.columns, self.rows = [int(num) for num in inputFilePath.readline().split()]
        self.landing_y, self.landing_x = [int(num) for num in inputFilePath.readline().split()]  # Landing site co-ordinates
        self.maxElevation = int(inputFilePath.readline())
        self.numOfTargetSites = int(inputFilePath.readline())

        self.targetList = []
        for eachTarget in range(0, self.numOfTargetSites):
            self.targetList.append([int(num) for num in inputFilePath.readline().split()])

        self.elevationMap = []
        for eachRow in range(0, self.rows):
            self.elevationMap.append([int(num) for num in inputFilePath.readline().split()])

        self.currentTarget_x = 0
        self.currentTarget_y = 0

    def cleanup(self):
        # Utility to clear Data Structures before processing any new target site
        self.nodeToParentMap.clear()
        self.visitedNodeSet.clear()
        if hasattr(self, 'priorityQueue'):
            self.priorityQueue.clear()

    def goalTest(self, x, y):
        # Utility to test if parameters (x,y) corresponds to the coordinates of goal node
        if x == self.currentTarget_x and y == self.currentTarget_y:
            return True  # Target site reached
        else:
            return False  # Not target site

    def generatePathToRootNode(self, x, y, outputFilePath, isLastLine=False):
        # Utility to generate path string from goal node to parent node for printing in output file
        pathString = ""
        # Check if current node is reachable
        if (x, y) in self.nodeToParentMap.keys():
            # Iterate till you reach root node from current node (x,y)
            while (x, y) != (None, None):  # Parent of root node does not exist i.e. (None, None)
                pathString = "{a},{b} ".format(a=y, b=x) + pathString
                x, y = self.nodeToParentMap[(x, y)]
        else:
            pathString = "FAIL"
        pathString = pathString.strip()
        # Write result in output file
        if not isLastLine:
            outputFilePath.write(pathString + "\n")
        else:
            outputFilePath.write(pathString)

    def findPath(self, executeAlgorithm):
        # Utility to compute path using specified algorithm
        outputFilePath = open("output.txt", "w")

        # Due to the property of UCS to provide shortest path to all nodes reachable from start node,
        # we will execute this algorithm only once in the beginning from the landing site to obtain the
        # nodeToParentMap. Then we will use this information to generate paths for all the target sites one by one.
        if executeAlgorithm.lower() == "UCS".lower():
            ucs_algorithm()
            result = True

        currLine = 0
        lastLine = len(self.targetList)
        for target_y, target_x in self.targetList:
            currLine += 1
            self.currentTarget_x = target_x
            self.currentTarget_y = target_y

            # Execute BFS / A* separately for all target sites
            if executeAlgorithm.lower() == "BFS".lower():
                result = bfs_algorithm()
            elif executeAlgorithm.lower() == "A*".lower():
                result = astar_algorithm()

            isLastLine = False
            if currLine == lastLine:
                isLastLine = True

            if result:
                self.generatePathToRootNode(self.currentTarget_x, self.currentTarget_y, outputFilePath, isLastLine)
            else:
                if isLastLine:
                    outputFilePath.writelines("FAIL")
                else:
                    outputFilePath.writelines("FAIL\n")
            # Cleanup before every iteration for BFS and A*
            if executeAlgorithm.lower() == "BFS".lower() or executeAlgorithm.lower() == "A*".lower():
                self.cleanup()
        # Final cleanup before exit
        self.cleanup()

    def discoverChildren(self, currentNodeCost, currentNode, isUCS=False, isAStar=False):
        # Utility to process all the adjacent co-ordinates from current node i.e. (N, S, W, E, NE, NW, SE, SW)
        # for UCS and A* algorithm.
        currentElevation = self.elevationMap[currentNode[0]][currentNode[1]]

        # Limits to check for boundary nodes
        lowest_x = currentNode[0] - 1
        lowest_y = currentNode[1] - 1
        highest_x = currentNode[0] + 1
        highest_y = currentNode[1] + 1

        # North
        north = (lowest_x, currentNode[1])
        if lowest_x > -1 and north not in self.visitedNodeSet:
            if isUCS:
                childEvaluationForUCS(north, currentElevation, currentNodeCost, currentNode, costFactor=10)
            elif isAStar:
                childEvaluationForAStar(north, currentElevation, currentNodeCost, currentNode, costFactor=10)

        # South
        south = (highest_x, currentNode[1])
        if highest_x < self.rows and south not in self.visitedNodeSet:
            if isUCS:
                childEvaluationForUCS(south, currentElevation, currentNodeCost, currentNode, costFactor=10)
            elif isAStar:
                childEvaluationForAStar(south, currentElevation, currentNodeCost, currentNode, costFactor=10)

        # West
        west = (currentNode[0], lowest_y)
        if lowest_y > -1 and west not in self.visitedNodeSet:
            if isUCS:
                childEvaluationForUCS(west, currentElevation, currentNodeCost, currentNode, costFactor=10)
            elif isAStar:
                childEvaluationForAStar(west, currentElevation, currentNodeCost, currentNode, costFactor=10)

        # East
        east = (currentNode[0], highest_y)
        if highest_y < self.columns and east not in self.visitedNodeSet:
            if isUCS:
                childEvaluationForUCS(east, currentElevation, currentNodeCost, currentNode, costFactor=10)
            elif isAStar:
                childEvaluationForAStar(east, currentElevation, currentNodeCost, currentNode, costFactor=10)

        # North West
        northwest = (lowest_x, lowest_y)
        if lowest_x > -1 and lowest_y > -1 and northwest not in self.visitedNodeSet:
            if isUCS:
                childEvaluationForUCS(northwest, currentElevation, currentNodeCost, currentNode, costFactor=14)
            elif isAStar:
                childEvaluationForAStar(northwest, currentElevation, currentNodeCost, currentNode, costFactor=14)

        # North East
        northeast = (lowest_x, highest_y)
        if lowest_x > -1 and highest_y < self.columns and northeast not in self.visitedNodeSet:
            if isUCS:
                childEvaluationForUCS(northeast, currentElevation, currentNodeCost, currentNode, costFactor=14)
            elif isAStar:
                childEvaluationForAStar(northeast, currentElevation, currentNodeCost, currentNode, costFactor=14)

        # South East
        southeast = (highest_x, highest_y)
        if highest_x < self.rows and highest_y < self.columns and southeast not in self.visitedNodeSet:
            if isUCS:
                childEvaluationForUCS(southeast, currentElevation, currentNodeCost, currentNode, costFactor=14)
            elif isAStar:
                childEvaluationForAStar(southeast, currentElevation, currentNodeCost, currentNode, costFactor=14)

        # South West
        southwest = (highest_x, lowest_y)
        if highest_x < self.rows and lowest_y > -1 and southwest not in self.visitedNodeSet:
            if isUCS:
                childEvaluationForUCS(southwest, currentElevation, currentNodeCost, currentNode, costFactor=14)
            elif isAStar:
                childEvaluationForAStar(southwest, currentElevation, currentNodeCost, currentNode, costFactor=14)


if __name__ == "__main__":
    # Open input file for reading
    inputFilePathObj = open("input.txt", "r")
    # Algorithm to use for finding path
    algorithm = inputFilePathObj.readline().strip()
    # Invoke Algorithm
    marsPathFinderObj = MarsPathFinder(inputFilePathObj)
    marsPathFinderObj.findPath(algorithm)
