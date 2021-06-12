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
    
    startNode=( problem.getStartState(),[])

    nodeList=util.Stack()
    nodeList.push(startNode)

    visited=[]

    while not nodeList.isEmpty():
        node=nodeList.pop()

        if node[0] not in visited:
            visited.append(node[0])

            if problem.isGoalState(node[0]):
                print("Way : ",visited)
                return node[1]
            else:
                for successors in problem.getSuccessors(node[0]):
                    
                    if successors[0] not in visited:
                        temp=node[1]+[successors[1]]
                        nodeList.push((successors[0],temp))

    return node[1]


def breadthFirstSearch(problem):

    startNode=( problem.getStartState(),[])

    nodeList=util.Queue()
    nodeList.push(startNode)

    visited=[]

    while not nodeList.isEmpty():
        node=nodeList.pop()

        if node[0] not in visited:
            visited.append(node[0])

            if problem.isGoalState(node[0]):
                print("Way : ",visited)
                return node[1]
            else:
                for successors in problem.getSuccessors(node[0]):
                    
                    if successors[0] not in visited:
                        temp=node[1]+[successors[1]]
                        nodeList.push((successors[0],temp))

    return node[1]

def uniformCostSearch(problem):

    visited = []
    nodeList = util.PriorityQueue()
    nodeList.push((problem.getStartState(), []) ,0)

    while not nodeList.isEmpty():
        node = nodeList.pop()

        if problem.isGoalState(node[0]):
            print('Way : ',visited)
            return node[1]

        if node[0] not in visited:
            for s in problem.getSuccessors(node[0]):
                if s[0] not in visited:
                    dir = s[1]
                    newCost = node[1] + [dir]
                    nodeList.push((s[0], node[1] + [dir]), problem.getCostOfActions(newCost))
        visited.append(node[0])
    return node[1]

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    
    nodeList = util.PriorityQueue()

    startState = problem.getStartState()
    nodeList.push((startState, '', 0), 0)

    visited = {}
    visited[startState] = ''
    

    cost = {}
    parents = {}
    solution = []

    cost[startState] = 0


    while(not nodeList.isEmpty()):
        node = nodeList.pop()
        visited[node[0]] = node[1]
        if problem.isGoalState(node[0]):
            solutionNode = node[0]
            break

        for s in problem.getSuccessors(node[0]):
            if s[0] not in visited.keys():
                p = node[2] + s[2] + heuristic(s[0], problem)
                if s[0] in cost.keys():
                    if cost[s[0]] <= p:
                        continue
                nodeList.push((s[0], s[1], node[2] + s[2]), p)
                cost[s[0]] = p
                parents[s[0]] = node[0]

    while(solutionNode in parents.keys()):
        parentNode = parents[solutionNode]
        solution.insert(0, visited[solutionNode])
        solutionNode = parentNode

    return solution



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
