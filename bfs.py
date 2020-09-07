from heapq import heapify, heappop

#  -----------------------------------------------------------------------------------------------------------------
#  ************************************** Breadth First Search Algorithm *******************************************
#  -----------------------------------------------------------------------------------------------------------------

def bfs_algorithm(self):
    # Utility to execute Breadth First Search Algorithm
    landingNode = (self.landing_x, self.landing_y)
    bfsQueue = [landingNode]
    self.visitedNodeSet.add(landingNode)
    self.nodeToParentMap[landingNode] = (None, None)  # Root has no parent

    while len(bfsQueue) != 0:
        currentNode = bfsQueue.pop(0)

        # Check if current node is the goal node
        if self.goalTest(currentNode[0], currentNode[1]):
            return True  # Success - path to goal node exits
        else:
            # Explore all the children of current node
            eligibleChildren = self.discoverChildrenForBFS(currentNode)
            for eachChild in eligibleChildren:
                bfsQueue.append(eachChild)
    return False  # No path to goal node exists


def discoverChildrenForBFS(self, currentNode):
    # Utility to process all the adjacent co-ordinates from current node i.e. (N, S, W, E, NE, NW, SE, SW)
    childrenList = []  # To store list of all eligible children
    currentElevation = self.elevationMap[currentNode[0]][currentNode[1]]

    # Limits to check for boundary nodes
    lowest_x = currentNode[0] - 1
    lowest_y = currentNode[1] - 1
    highest_x = currentNode[0] + 1
    highest_y = currentNode[1] + 1

    # Check for boundaries and whether the node has been previously explored or not.
    # Perform elevation test for newly discovered nodes.

    # North West
    northwest = (lowest_x, lowest_y)
    if lowest_x > -1 and lowest_y > -1 and northwest not in self.visitedNodeSet:
        if abs(currentElevation - self.elevationMap[northwest[0]][northwest[1]]) <= self.maxElevation:
            self.visitedNodeSet.add(northwest)
            self.nodeToParentMap[northwest] = currentNode
            childrenList.append(northwest)

    # North
    north = (lowest_x, currentNode[1])
    if lowest_x > -1 and north not in self.visitedNodeSet:
        if abs(currentElevation - self.elevationMap[north[0]][north[1]]) <= self.maxElevation:
            self.visitedNodeSet.add(north)
            self.nodeToParentMap[north] = currentNode
            childrenList.append(north)

    # North East
    northeast = (lowest_x, highest_y)
    if lowest_x > -1 and highest_y < self.columns and northeast not in self.visitedNodeSet:
        if abs(currentElevation - self.elevationMap[northeast[0]][northeast[1]]) <= self.maxElevation:
            self.visitedNodeSet.add(northeast)
            self.nodeToParentMap[northeast] = currentNode
            childrenList.append(northeast)

    # East
    east = (currentNode[0], highest_y)
    if highest_y < self.columns and east not in self.visitedNodeSet:
        if abs(currentElevation - self.elevationMap[east[0]][east[1]]) <= self.maxElevation:
            self.visitedNodeSet.add(east)
            self.nodeToParentMap[east] = currentNode
            childrenList.append(east)

    # South East
    southeast = (highest_x, highest_y)
    if highest_x < self.rows and highest_y < self.columns and southeast not in self.visitedNodeSet:
        if abs(currentElevation - self.elevationMap[southeast[0]][southeast[1]]) <= self.maxElevation:
            self.visitedNodeSet.add(southeast)
            self.nodeToParentMap[southeast] = currentNode
            childrenList.append(southeast)

    # South
    south = (highest_x, currentNode[1])
    if highest_x < self.rows and south not in self.visitedNodeSet:
        if abs(currentElevation - self.elevationMap[south[0]][south[1]]) <= self.maxElevation:
            self.visitedNodeSet.add(south)
            self.nodeToParentMap[south] = currentNode
            childrenList.append(south)

    # South West
    southwest = (highest_x, lowest_y)
    if highest_x < self.rows and lowest_y > -1 and southwest not in self.visitedNodeSet:
        if abs(currentElevation - self.elevationMap[southwest[0]][southwest[1]]) <= self.maxElevation:
            self.visitedNodeSet.add(southwest)
            self.nodeToParentMap[southwest] = currentNode
            childrenList.append(southwest)

    # West
    west = (currentNode[0], lowest_y)
    if lowest_y > -1 and west not in self.visitedNodeSet:
        if abs(currentElevation - self.elevationMap[west[0]][west[1]]) <= self.maxElevation:
            self.visitedNodeSet.add(west)
            self.nodeToParentMap[west] = currentNode
            childrenList.append(west)

    return childrenList