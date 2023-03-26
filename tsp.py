import random
from itertools import permutations
import time
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

def bruteForce(listOfPoints):
    start_time = time.time()
    bestDistance = 0
    per = list(permutations(listOfPoints))
    for possibleSolution in per: 
        distance = calculateDistanceForListOfPoints(possibleSolution)
        if (bestDistance == 0 or distance < bestDistance) :
            bestDistance = distance
    return time.time() - start_time, 's'

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

def calculateDistanceBetweenPoints(firstPoint, secondPoint):
    xDistance = firstPoint.x - secondPoint.x
    yDistance = firstPoint.y - secondPoint.y
    distance = abs(xDistance) + abs(yDistance)
    return distance

def calculateDistanceForListOfPoints(listOfPoints):
    distance = 0
    for i, point in enumerate(listOfPoints): 
        if i + 1 < len(listOfPoints):
            distance += calculateDistanceBetweenPoints(point, listOfPoints[i+1])
    return distance

def printListOfPoints(listOfPoints):
    stringOfCombination = ""
    for point in listOfPoints: 
        stringOfCombination += f"( {point.x}, {point.y}) "
    return stringOfCombination

def runSimulation():

    numberOfCities = 3 
    while numberOfCities != 10: 
        pointList = generatePoints(numberOfCities,10)
        timeInSeconds = bruteForce(pointList)
        print(f"It took {timeInSeconds} for {numberOfCities} points.\n")
        numberOfCities = numberOfCities + 1
    #we'll run all 4 algorithms
    #collect their data
    #run analysis
    #print results
    return

runSimulation()