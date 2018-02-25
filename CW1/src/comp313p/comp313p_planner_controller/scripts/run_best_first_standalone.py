#! /usr/bin/env python

from comp313p_planner_controller.occupancy_grid import OccupancyGrid
from comp313p_planner_controller.bf_planner import BFPlanner

occupancyGrid = OccupancyGrid(21, 21, 0.5)

for y in xrange(0, 19):
    occupancyGrid.setCell(11, y, 1)

start = (3, 18)
goal = (20, 0)

planner = BFPlanner('Best First Search', occupancyGrid);
planner.setRunInteractively(True)

planner.setWindowHeightInPixels(400)

goalReached = planner.search(start, goal)

path = planner.extractPathToGoal()
