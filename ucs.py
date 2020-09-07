from heapq import heapify, heappop

#  -----------------------------------------------------------------------------------------------------------------
#  *************************************** Uniform Cost Search Algorithm *******************************************
#  -----------------------------------------------------------------------------------------------------------------

def ucs_algorithm(self):
    # Utility to execute UCS Algorithm
    landingNode = (self.landing_x, self.landing_y)
    self.priorityQueue = [(0, landingNode)]
    self.visitedNodeSet.add(landingNode)
    self.nodeToParentMap[landingNode] = (None, None)  # Root has no parent

    while len(self.priorityQueue) != 0:
        currentNodeCost, currentNode = heappop(self.priorityQueue)
        # Explore all the children
        self.discoverChildren(currentNodeCost, currentNode, isUCS=True)
        self.visitedNodeSet.add(currentNode)
        # To rearrange elements in priority queue
        heapify(self.priorityQueue)


def childEvaluationForUCS(self, childCoords, currentElevation, currentNodeCost, currentNode, costFactor=10):
    # Utility to check if the specified child co-ordinates is worth exploring or not
    # Perform elevation test
    if abs(currentElevation - self.elevationMap[childCoords[0]][childCoords[1]]) <= self.maxElevation:
        # Calculate cost to reach the node
        latestCost = currentNodeCost + costFactor
        # Check if child already exist in priority queue.
        # If the currently computed cost is less than the one already present in the queue,
        # then replace it with new lower cost.
        childExistInQueue = False
        for costInQueue, eachChild in self.priorityQueue:
            if childCoords == eachChild:
                childExistInQueue = True
                if latestCost < costInQueue:
                    self.priorityQueue.remove((costInQueue, childCoords))
                    self.priorityQueue.append((latestCost, childCoords))
                    self.nodeToParentMap[childCoords] = currentNode
                break
        if not childExistInQueue:
            self.priorityQueue.append((latestCost, childCoords))
            self.nodeToParentMap[childCoords] = currentNode
