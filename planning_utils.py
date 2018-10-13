from enum import Enum
from queue import PriorityQueue, LifoQueue
import numpy as np
import utm

from bresenham import bresenham


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NW = (-1, -1, 1.4)
    NE = (-1, 1, 1.4)
    SW = (1, -1, 1.4)
    SE = (1, 1, 1.4)
    


    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    vActionSet = set(valid_actions)
    if not((Action.NORTH in vActionSet) and (Action.WEST in vActionSet)):
        valid_actions.remove(Action.NW)
    if not((Action.NORTH in vActionSet) and (Action.EAST in vActionSet)):
        valid_actions.remove(Action.NE)
    if not((Action.SOUTH in vActionSet) and (Action.WEST in vActionSet)):
        valid_actions.remove(Action.SW)
    if not((Action.SOUTH in vActionSet) and (Action.EAST in vActionSet)):
        valid_actions.remove(Action.SE)
    
    return valid_actions

# Is this location obstructing a buildling
def isLocationValid(grid, current_node):
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    if x < 0 or x > n or grid[x, y] == 1 or y < 0 or y > m:
        return False

    return True

def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    # print("a*tt " + str(start) + "\t:: " + str(queue.qsize()))

    counter = 0
    while not queue.empty():
        item = queue.get()

        counter = counter + 1
        vEnd = "\r"
        if counter%50 == 0:
            vEnd = "\n"
        print(">> " + str(item) + "\t" + str(queue.qsize()) + "\t" + str(len(branch)), end=vEnd, flush="True")
              
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    print("branch len " + str(len(branch)))
    
    if found:
        # retrace steps
        n = goal
        print("goal branch " + str(branch[n]))
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:

            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')


    print("path_cost " + str(path_cost))
    print("path len " + str(len(path)))

    finalPath = luigiham(path, grid)
    
    return finalPath[::-1], path_cost


def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))


def point(p):
    # taken from: https://view6ddbca99.udacity-student-workspaces.com/notebooks/A-Star-City-Solution.ipynb
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    # taken from: https://view6ddbca99.udacity-student-workspaces.com/notebooks/A-Star-City-Solution.ipynb
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    # taken from: https://view6ddbca99.udacity-student-workspaces.com/notebooks/A-Star-City-Solution.ipynb
    pruned_path = [p for p in path]
    
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        
        # If the 3 points are in a line remove the 2nd point.
        # The 3rd point now becomes and 2nd point and the
        #  check is redone with a new third point on the next iteration.
        if collinearity_check(p1, p2, p3):
            # Something subtle here but we can mutate `pruned_path`
            # freely because the length of the list is check on every iteration.
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path  


# LuigiHam is a combination of Luigi's thinking and Bresenham's formulae.
# LuigiHam accepts a path array, representing a safe path from start to finish.
# Each  array item (called a point) contains an X & Y co-ord
# LuigiHam attempts direct paths between the furtherest two points
# LuigiHam returns an updated path array (same structure as path), with direct roots
# LuigiHam accepts a grid, so that it is able to understand where obstructions are.
# The cleaner version of this code does not accept the grid and expects the isLocationValid() method to be self-aware of the grid variable
# This version needs to be updated, because it always 'always' has two middle point, even tho not necessary. The solution below create this by side-effect design.
def luigiham(path, grid):
    finalPath = []
    print("LuigiHam TIME")
    customPath = [p for p in path]
    
    customPathLen = len(customPath)
    print("customPathLen ", customPathLen)

    i = 0
    while i < customPathLen: #start to goal
        p1 = customPath[i]

        j = customPathLen-1    
        while j > i: #goal to start
            p2 = customPath[j] 

            cells = list(bresenham(p1[0], p1[1], p2[0], p2[1]))
            newDirectPathHasNoObstructions = True
            for cell in cells:
                if (isLocationValid(grid, cell) == False):
                    newDirectPathHasNoObstructions = False
                    break

            if newDirectPathHasNoObstructions == True:
                finalPath.append(p1)
                finalPath.append(p2)
                i = j
                break
            else:
                j = j - 1
       
        i = i + 1

    print("LuigiHam TIME FINISHED")
    print("len(finalPath) ", len(finalPath))
    return finalPath


def l_star(grid, start, goal, numberOfRoutesToFind):

   # l_start is supposed to be a a_star replacment, requires less processing power and to be immensly quicker

   # The implementation takes the luigiham concept into play.

   # The start and goal co-ordinates are compared.
   # L_Star attempts to move the start co-ordinates in a favourable direction towards the goal co-ordinates.
   # It does do some guess work in the instance that two directions have the same "benefit" in direction towards the goal
   # L_star can look for more roots, once a first route has been found.
   # LuigiHam gets implemented towards the end of l_star
   # L_star attempts LuigiHam logic at every new point obtained

   return "ommitted"
   