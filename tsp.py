import random
#import matplotlib.pyplot as plt
#import numpy as np

class TSP:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y
    def __eq__(self, other):
        if isinstance(other, TSP):
            return self.x == other.x and self.y == other.y
        return False


#Algorithms to use
# 1. Brute Force (up to n=10 due to computer speed)
# 2. Closest Edge
# 3. DFA
# 4. BFA

def bruteForce():
    return

def closestEdge():
    return

def dfa():
    return

def bfa():
    return

def generatePoints(numberOfPoints, maxCoordinateValue):
    pointList = list() #of type TSP
    while len(pointList) != numberOfPoints:
        x = random.randint(0,maxCoordinateValue)
        y = random.randint(0,maxCoordinateValue)
        tspToCheck = TSP(x,y)
        if not any(tsp == tspToCheck for tsp in pointList):
            pointList.append(TSP(x,y))

    return pointList

def runSimulation():
    pointList = generatePoints(10,10)
    #we'll run all 4 algorithms
    #collect their data
    #run analysis
    #print results
    return

runSimulation()