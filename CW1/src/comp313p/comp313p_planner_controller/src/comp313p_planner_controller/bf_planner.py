# -*- coding: utf-8 -*-

from cell_based_best_search import CellBasedBestSearch
from collections import deque


class BFPlanner(CellBasedBestSearch):

    # self implements a BF search algorithm
    
    def __init__(self, title, occupancyGrid):
        CellBasedBestSearch.__init__(self, title, occupancyGrid)
        self.bfQueue = deque()

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        self.bfQueue.append(cell)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.bfQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.bfQueue.popleft()
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass
