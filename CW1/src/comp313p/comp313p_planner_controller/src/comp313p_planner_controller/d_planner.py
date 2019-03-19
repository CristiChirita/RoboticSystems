# -*- coding: utf-8 -*-

from cell_based_dijkstra_search import CellBasedDijkstraSearch
import heapq


class DPlanner(CellBasedDijkstraSearch):

    # self implements a Dijkstra search algorithm
    
    def __init__(self, title, occupancyGrid):
        CellBasedDijkstraSearch.__init__(self, title, occupancyGrid)
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

    def resolveDuplicate(self, distance, cell):
        i = 0
	# Iterate until we find the cell in the queue
	while i < len(self.dQueue):
		(currentDistance, currentCell) = self.dQueue[i]
		if cell != currentCell:
			i = i + 1
			continue
		# Do nothing if cell is already at a lower cost than the one we are trying to give
		if distance >= currentDistance:
			return
		else:
			# Remove cell from list, and push again with updated cost
			self.dQueue.remove((currentDistance, currentCell))
			heapq.heapify(self.dQueue)
			heapq.heappush(self.dQueue, (distance, cell))
			return

        
