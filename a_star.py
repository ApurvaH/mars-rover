from heapq import heapify, heappop

#  -----------------------------------------------------------------------------------------------------------------
#  ******************************************* A* Search Algorithm *************************************************
#  -----------------------------------------------------------------------------------------------------------------

def astar_algorithm(self):
    # Utility to execute A* Algorithm
    landingNode = (self.landing_x, self.landing_y)
    self.priorityQueue = [(0, landingNode)]
    self.visitedNodeSet.add(landingNode)
    self.nodeToParentMap[landingNode] = (None, None)  # Root has no parent

    while len(self.priorityQueue) != 0:
        currentNodeCost, currentNode = heappop(self.priorityQueue)
        # Check if it's the goal node
        if self.goalTest(currentNode[0], currentNode[1]):
            return True  # Success
        else:
            # Explore all the children
            self.discoverChildren(currentNodeCost, currentNode, isAStar=True)
            self.visitedNodeSet.add(currentNode)
            # To rearrange elements in priority queue
            heapify(self.priorityQueue)
    return False


def childEvaluationForAStar(self, childCoords, currentElevation, currentNodeCost, currentNode, costFactor=10):
    # Utility to check if the specified child co-ordinates is worth exploring or not
    # Perform elevation test
    childElevation = self.elevationMap[childCoords[0]][childCoords[1]]
    if abs(currentElevation - childElevation) <= self.maxElevation:
        # Cost to reach current node
        costToReachChild = (currentNodeCost + costFactor + abs(currentElevation - childElevation))
        # Heuristic Function
        delta_x = abs(childCoords[0] - self.currentTarget_x)
        delta_y = abs(childCoords[1] - self.currentTarget_y)
        estimatedCostToReachGoalNode = (14 * min(delta_x, delta_y)) + (10 * abs(delta_x - delta_y))
        # Computed cost to reach goal node from start through the current child
        latestCost = costToReachChild + estimatedCostToReachGoalNode
        # Check if child already exist in priority queue.
        # If the currently computed cost is less than the one already present in the queue,
        # then replace it with new lower cost.
        childExistInQueue = False
        for costInQueue, eachChild in self.priorityQueue:
            if childCoords == eachChild:
                childExistInQueue = True
                if latestCost < costInQueue:
                    self.priorityQueue.remove((costInQueue, childCoords))
                    self.priorityQueue.append((costToReachChild, childCoords))
                    self.nodeToParentMap[childCoords] = currentNode
                break
        if not childExistInQueue:
            self.priorityQueue.append((costToReachChild, childCoords))
            self.nodeToParentMap[childCoords] = currentNode
