# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """


    #Instantiate Stack of our moves
    moveStack = util.Stack()

    #Current state is the start state
    #Instantiate a history of moves array and list of explored states
    state = problem.getStartState()
    stateHistory = []
    exploredStates = [problem.getStartState()]


    #Get the directions
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    e = Directions.EAST
    n = Directions.NORTH

    #Check if we started at the solution
    if problem.isGoalState(problem.getStartState()) == True:
        found = True
        return []
    else:
        found = False

    #Get the first fringe
    fringe = problem.getSuccessors(state)

    #Add the fringe to the stack with the history of moves
    for successor in fringe:

        history = stateHistory.copy()
        moveStack.push((successor,history))

    #Pop the first sucessor
    nextItem = moveStack.pop()

    #While we have not found the goal
    while found == False:
        successor = nextItem[0]
        state = successor[0]
        thisAction = successor[1]
        stateHistory = nextItem[1]

        #If the current state is the goal we found the goal 
        if problem.isGoalState(state):
            stateHistory.append(thisAction)
            return stateHistory

        #If the current state is not the goal   
        else:

            alreadyExplored = False
            
            #If this state has not already been explored add the fringe of this state
            for exploredState in exploredStates:
                if exploredState == state:
                    alreadyExplored = True
                    break

            if not alreadyExplored: #If this state is not explored
                fringe = problem.getSuccessors(state)

                #Add the fringe to the stack
                for successor in fringe:
                    history = stateHistory.copy()
                    history.append(thisAction)

                    moveStack.push((successor,history))
            
                #Append this state to the list of explored states
                exploredStates.append(state)

            nextItem = moveStack.pop()



def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""


    #Instantiate queue of our moves
    moveQueue = util.Queue()

    #Current state is the start state
    state = problem.getStartState()
    stateHistory = []
    exploredStates = [problem.getStartState()]

    #Get the directions
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    e = Directions.EAST
    n = Directions.NORTH

    #Check if we started at the solution
    if problem.isGoalState(problem.getStartState()) == True:
        found = True
        return []
    else:
        found = False

    #Get the first fringe
    fringe = problem.getSuccessors(state)

    #Add the fringe to the stack with the history of moves
    for successor in fringe:
        history = stateHistory.copy()
        moveQueue.push((successor,history))

    #Pop the first state
    nextItem = moveQueue.pop()

    #While we have not found the goal
    while found == False:
        successor = nextItem[0]
        state = successor[0]
        thisAction = successor[1]
        stateHistory = nextItem[1]

        #If the current state is the goal we found the goal
        if problem.isGoalState(state):
            stateHistory.append(thisAction)
            return stateHistory
        #If the current state is not the goal   
        else:

            alreadyExplored = False

            #If this state has not already been explored add the fringe of this state
            for exploredState in exploredStates:
                if exploredState == state:
                    alreadyExplored = True
                    break

            if not alreadyExplored: #If this state is not explored
                fringe = problem.getSuccessors(state)
 
                #Add the fringe to the stack
                for successor in fringe:
                    history = stateHistory.copy()
                    history.append(thisAction)
                    moveQueue.push((successor,history))
            
                #Append this state to the list of explored states
                exploredStates.append(state)

            nextItem = moveQueue.pop()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    #Instantiate priority queue of our moves
    moveQueue = util.PriorityQueue()

    #Current state is the start state
    state = problem.getStartState()
    stateHistory = []
    exploredStates = [problem.getStartState()]

    #Get the directions
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    e = Directions.EAST
    n = Directions.NORTH

    #Check if we started at the solution
    if problem.isGoalState(problem.getStartState()) == True:
        found = True
        return []
    else:
        found = False

    #Get the first fringe
    fringe = problem.getSuccessors(state)

    #Add the fringe to the stack with the history of moves
    for successor in fringe:

        history = stateHistory.copy()
        cost = successor[2]
        moveQueue.push((successor,history,cost),cost)

    nextItem = moveQueue.pop()
    #While we have not found the goal
    while found == False:
        successor = nextItem[0]
        state = successor[0]
        thisAction = successor[1]
        stateHistory = nextItem[1]
        currentCost = nextItem[2]

        #If the current state is the goal we found the goal
        if problem.isGoalState(state):
            stateHistory.append(thisAction)
            return stateHistory

        #If the current state is not the goal   
        else:
            alreadyExplored = False

            #If this state has not already been explored add the fringe of this state
            for exploredState in exploredStates:
                if exploredState == state:
                    alreadyExplored = True
                    break

            if not alreadyExplored: #If this state is not explored
                fringe = problem.getSuccessors(state)

                #Add the fringe to the stack
                for successor in fringe:

                    history = stateHistory.copy()
                    history.append(thisAction)
                    cost = currentCost + successor[2]
                    moveQueue.push((successor,history,cost),cost)
                             
                #Append this state to the list of explored states
                exploredStates.append(state)

            nextItem = moveQueue.pop()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic):
    """Search the node of least total cost first."""

    #Instantiate priority queue of our moves
    moveQueue = util.PriorityQueue()

    #Current state is the start state
    state = problem.getStartState()
    stateHistory = []
    exploredStates = []


    #Get the directions
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    e = Directions.EAST
    n = Directions.NORTH

    #Check if we started at the solution
    if problem.isGoalState(problem.getStartState()) == True:
        found = True
        return []
    else:
        found = False

    #Get the first fringe
    fringe = problem.getSuccessors(state)

    #Add the fringe to the stack with the history of moves
    for successor in fringe:

        history = stateHistory.copy()
        heuristicValue = heuristic(successor[0],problem)

        priority = successor[2] + heuristicValue
        moveQueue.push((successor,history),priority)

    #   Pop the first state
    nextItem = moveQueue.pop()

    #While we have not found the goal
    while found == False:
        successor = nextItem[0]
        state = successor[0]
        thisAction = successor[1]
        stateHistory = nextItem[1]

        #If the current state is the goal we found the goal
        if problem.isGoalState(state):
            stateHistory.append(thisAction)
            return stateHistory

        #If the current state is not the goal   
        else:
            alreadyExplored = False

            #If this state has not already been explored add the fringe of this state
            for exploredState in exploredStates:
                if exploredState == state:
                    alreadyExplored = True
                    break

            if alreadyExplored == False: #If this state is not explored
                fringe = problem.getSuccessors(state)
 
                #Add the fringe to the stack
                for successor in fringe:

                    history = stateHistory.copy()
                    history.append(thisAction)
                    heuristicValue = heuristic(successor[0],problem)
                    priority = successor[2] + heuristicValue
                    moveQueue.push((successor,history),priority)  
                             
                #Append this state to the list of explored states
                exploredStates.append(state)

            nextItem = moveQueue.pop()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
