# -*- coding: utf-8 -*-

from cell_based_astar_search import CellBasedAStarSearch
import heapq


class AStarPlanner(CellBasedAStarSearch):

    # self implements a Dijkstra search algorith with A* heauristics
    
    def __init__(self, title, occupancyGrid):
        CellBasedAStarSearch.__init__(self, title, occupancyGrid)
        self.dQueue = []

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, values):
        heapq.heappush(self.dQueue, values)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.dQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = heapq.heappop(self.dQueue)
        return cell

    def resolveDuplicate(self, distance, cell, aDist):
        i = 0
	# Iterate until we find the cell in the queue
	while i < len(self.dQueue):
		(currentDistance, currentCell, altDist) = self.dQueue[i]
		if cell != currentCell:
			i = i + 1
			continue
		# Do nothing if cell is already at a lower cost than the one we are trying to give
		if distance >= currentDistance:
			return False
		else:
			# Remove cell from list, and push again with updated cost
			self.dQueue.remove((currentDistance, currentCell, altDist))
			heapq.heapify(self.dQueue)
			heapq.heappush(self.dQueue, (distance, cell, aDist))
			return True

        
