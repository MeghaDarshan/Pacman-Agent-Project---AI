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

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    stack = util.Stack()
    path = []
    initialCost = 0
    # Push the first State into Stack
    stack.push((problem.getStartState(), path, initialCost))
    visitedStates = set()
    # Until the stack has item, perform below operations
    while stack:
        currentState, totalActions, totalCost = stack.pop()  # Pop the item from stack and assign it to variables

        if (problem.isGoalState(currentState)):  # Check if the currentState is the GoalState
            return totalActions

        if (not currentState in visitedStates):  # Check if the CurrentState is not present in the VisitedStates Set
            visitedStates.add(currentState)
            for nextState, nextAction, nextCost in problem.getSuccessors(
                    currentState):  # Get the sucessors of the currentState and Push into Stack
                stack.push((nextState, totalActions + [nextAction], totalCost + nextCost))

    return []
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    QueueBFS = util.Queue()
    startPath = []
    costBFS = 0
    QueueBFS.push((problem.getStartState(), startPath, costBFS))  # Push the StartStae into Queue
    ReachedNodes = []
    while QueueBFS:
        presentPos, path, dist = QueueBFS.pop()  # Pop an Item from the Queue and assign it to variables

        if (problem.isGoalState(presentPos)):  # Check if the presentPos popped from Queue is the Goal State
            return path

        if (not presentPos in ReachedNodes):  # Check if the presentPos state is present in the set
            ReachedNodes.append(presentPos)
            for nextPosition, nextPath, nextDist in problem.getSuccessors(
                    presentPos):  # Get the sucessor nodes of presentPos and push it to the Queue
                QueueBFS.push((nextPosition, path + [nextPath], dist + nextDist))

    return []


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    QueueUCS = util.PriorityQueue()
    startPath = []
    costBFS = 0
    minCost = 0
    QueueUCS.push((problem.getStartState(), startPath, costBFS), minCost)  # Push the StartState into Queue
    ReachedNodes = []
    while QueueUCS:
        presentPos, path, dist = QueueUCS.pop()  # Pop an Item from the Queue and assign it to variables

        if (problem.isGoalState(presentPos)):  # Check if the presentPos popped from Queue is the Goal State
            return path

        if (not presentPos in ReachedNodes):  # Check if the presentPos state is present in the set
            ReachedNodes.append(presentPos)
            for nextPosition, nextPath, nextDist in problem.getSuccessors(
                    presentPos):  # Get the sucessor nodes of presentPos and push it to the Queue
                sumcost = problem.getCostOfActions(path + [nextPath])
                QueueUCS.push((nextPosition, path + [nextPath], dist + nextDist), sumcost)

    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    QueueStar = util.PriorityQueue()
    startPath = []
    costBFS = 0
    minCost = 0
    QueueStar.push((problem.getStartState(), startPath, costBFS), minCost)  # Push the StartState into Queue
    ReachedNodes = []
    while QueueStar:
        presentPos, path, dist = QueueStar.pop()  # Pop an Item from the Queue and assign it to variables

        if (problem.isGoalState(presentPos)):  # Check if the presentPos popped from Queue is the Goal State
            return path

        if (not presentPos in ReachedNodes):  # Check if the presentPos state is present in the set
            ReachedNodes.append(presentPos)
            for nextPosition, nextPath, nextDist in problem.getSuccessors(
                    presentPos):  # Get the sucessor nodes of presentPos and push it to the Queue
                sumcost = problem.getCostOfActions(path + [nextPath]) + heuristic(nextPosition, problem) # Get the Path cost as well as Heuristic Cost
                QueueStar.push((nextPosition, path + [nextPath], dist + nextDist), sumcost)

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
