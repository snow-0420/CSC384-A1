#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
import heapq
from search import *  # for search engines
from sokoban import SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

def sokoban_goal_state(state):
    '''
    @return: Whether all boxes are stored.
    '''
    for box in state.boxes:
        if box not in state.storage:
            return False
    return True

def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.
    h_val = 0
    for box in state.boxes:
        dist = math.inf
        for storage in state.storage:
            if abs(box[0] - storage[0]) + abs(box[1] - storage[1]) < dist:
                dist = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
        h_val += dist
    return h_val

# SOKOBAN HEURISTICS
def trivial_heuristic(state):
    '''trivial admissible sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
    return 0  # CHANGE THIS

def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.

    # This heuristic is admissible.
    val = 0
    for box in state.boxes:
        if box in state.storage:
            continue
        if check_corner(state, box): # if any box is in a corner and not a storage point, it can no longer be moved, hence return infinity (will never reach goal)
            return math.inf
        dist = math.inf
        for storage in state.storage:
            if storage in state.boxes: # If storage point already has box on it, don't consider this storage point
                continue
            if abs(box[0] - storage[0]) + abs(box[1] - storage[1]) < dist:
                dist = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
        val += dist
    return val # Measure manhattan distance, but now when any storage point already has boxes on it, it will not count towards the h_val.

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return sN.gval + weight * sN.hval

# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a warehouse state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''
    start_time = os.times()[0]
    end_time = start_time + timebound
    se = SearchEngine('custom', 'full')
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
    goal, ss = se.search(end_time - os.times()[0])

    return goal, ss

def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a warehouse state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of realtime astar algorithm'''
    start_time = os.times()[0]
    end_time = start_time + timebound
    goal, ss = weighted_astar(initial_state, heur_fn, weight, end_time - os.times()[0])
    while os.times()[0] < end_time:
        diff = end_time - os.times()[0]
        costbound = (math.inf, math.inf, goal.gval)
        reduced_weight = (1 + weight) // 2
        se = SearchEngine('custom', 'full')
        wrapped_fval_function = (lambda sN: fval_function(sN, reduced_weight))
        se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
        it_goal, it_ss = se.search(diff, costbound)
        if it_goal and (it_goal.gval < goal.gval):
            goal, ss = it_goal, it_ss
    return goal, ss

def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    start_time = os.times()[0]
    end_time = start_time + timebound

    se = SearchEngine('best_first', 'full')
    se.init_search(initial_state, sokoban_goal_state, heur_fn)
    goal, ss = se.search(end_time - os.times()[0])

    while os.times()[0] < end_time:
        diff = end_time - os.times()[0]
        costbound = (goal.gval, math.inf, math.inf)
        se.init_search(initial_state, sokoban_goal_state, heur_fn)
        it_goal, it_ss = se.search(diff, costbound)
        if it_goal and (it_goal.gval < goal.gval):
            goal, ss = it_goal, it_ss
    return goal, ss

def check_corner(state, box):
    '''Check if the input box (as a tuple of x and y coordinate) is in a corner'''
    '''return true if in corner (hence we can prune the state)'''
    '''prerequisite: box is not in storage point'''
    up = (box[1] - 1 < 0) or ((box[0], box[1] - 1) in state.obstacles)
    right = (box[0] + 1 >= state.width) or ((box[0] + 1, box[1]) in state.obstacles)
    down = (box[1] + 1 >= state.height) or ((box[0], box[1] + 1) in state.obstacles)
    left = (box[0] - 1 < 0) or ((box[0] - 1, box[1]) in state.obstacles)

    return (up or down) and (left or right)
