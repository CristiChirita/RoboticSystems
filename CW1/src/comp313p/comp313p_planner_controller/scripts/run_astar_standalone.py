#! /usr/bin/env python

from comp313p_planner_controller.occupancy_grid import OccupancyGrid
from comp313p_planner_controller.astar_planner import AStarPlanner

occupancyGrid = OccupancyGrid(21, 21, 0.5)

for y in xrange(0, 13):
    occupancyGrid.setCell(11, y, 1)

start = (3, 18)
goal = (20, 0)

planner = AStarPlanner('A* search', occupancyGrid);
planner.setRunInteractively(True)

planner.setWindowHeightInPixels(400)

goalReached = planner.search(start, goal)

path = planner.extractPathToGoal()
