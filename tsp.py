import random
from itertools import permutations
import time
import csv 
import timeit
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
    def __hash__(self):
        return hash(str(self))
    def __str__(self):
     return '(' + str(self.x) + ',' + str(self.y) + ')'

class TSPEdge:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
        self.length = calculateDistanceBetweenPoints(p1, p2)

    def distanceFromEdge(self, point):
        distance1 = calculateDistanceBetweenPoints(self.p1, point)
        distance2 = calculateDistanceBetweenPoints(self.p2, point)

        return (distance1 + distance2 - self.length)
    
class AlgorithmResult:
    def __init__(self, time, distance, combination) -> None:
        self.time = time
        self.distance = distance
        self.combination = combination


#Algorithms to use
# 1. Brute Force (up to n=10 due to computer speed)
# 2. Closest Edge
# 3. dfs
# 4. BFA

def bruteForce(listOfPoints):
    startTime = timeit.default_timer()
    bestCombination = {}
    bestDistance = 0
    per = list(permutations(listOfPoints))
    for possibleSolution in per: 
        distance = calculateDistanceForListOfPoints(possibleSolution)
        if (bestDistance == 0 or distance < bestDistance) :
            bestDistance = distance
            bestCombination = possibleSolution
    endtime = timeit.default_timer()
    finalTime = endtime - startTime
    return AlgorithmResult(finalTime, bestDistance, printListOfPoints(bestCombination)) 

def closestEdge(listOfPoints):
    startTime = timeit.default_timer()
    finalList = list() #TSP
    edgesList = list() #TSPEdge

    for i, point in enumerate(listOfPoints):
        #first 3 nodes create their own 'triange'
        if (i < 3):
            j = i + 1
            if j==3: j = 0
            edgesList.append(TSPEdge(point, listOfPoints[j]))
        else:
            currentEdge = edgesList[0]
            localDistance = edgesList[0].distanceFromEdge(point)

            for j, edge in enumerate(edgesList):
                tempDistance = edge.distanceFromEdge(point)
                if tempDistance < localDistance:
                    localDistance = tempDistance
                    currentEdge = edge
            index = edgesList.index(currentEdge)
            edgesList.insert(index, TSPEdge(currentEdge.p1, point))
            edgesList.insert(index+1, TSPEdge(point, currentEdge.p2))
            edgesList.remove(currentEdge)
    
    for edge in edgesList:
        finalList.append(edge.p1)
    
    distance = 0
    for i, point in enumerate(finalList):
        if (i+1) in range(0, len(finalList)):
            distance += calculateDistanceBetweenPoints(point, finalList[i+1])
    
    endtime = timeit.default_timer()
    finalTime = endtime - startTime
    return AlgorithmResult(finalTime, distance, printListOfPoints(finalList))

def dfs(listOfPoints):
    startTime = timeit.default_timer()
    graph = tspGraph(listOfPoints)
    visited = list()
    dfsTraversal(visited, graph, listOfPoints[0])
    distance = 0
    for i, point in enumerate(visited):
        if (i+1) in range(0, len(visited)):
            distance += calculateDistanceBetweenPoints(point, visited[i+1])
    endTime = timeit.default_timer()
    finalTime = endTime - startTime
    return AlgorithmResult(finalTime, distance, printListOfPoints(visited))

def dfsTraversal(visited, graph, node):
    if node not in visited:
        visited.append(node)
        # print(type(node))
        # print(f"({node.x}, {node.y}) ")
        if node in graph:
            for neighbour in graph[node]:
                dfsTraversal(visited, graph, neighbour)

#requires at least n = 3 points
def tspGraph(listOfPoints):
    graph = {
        listOfPoints[0]: [listOfPoints[1], listOfPoints[2]],
    }

    for i, point in enumerate(listOfPoints):
        if i != 0 and (i + 3) in range(0, len(listOfPoints)):
            graph[listOfPoints[i]] = [listOfPoints[i+2], listOfPoints[i+3]]

    # for x in graph:
    #     print('parent: ' +  f"({x.x}, {x.y}) ")
    #     for y in graph[x]:
    #         print(f"({y.x}, {y.y}) ")

    return graph

def bfs(listOfPoints):
    startTime = timeit.default_timer()
    graph = tspGraph(listOfPoints)
    visited = list()
    bfsTraversal(visited, graph, listOfPoints[0])
    distance = 0
    for i, point in enumerate(visited):
        if (i+1) in range(0, len(visited)):
            distance += calculateDistanceBetweenPoints(point, visited[i+1])
    endTime = timeit.default_timer() - startTime
    return AlgorithmResult(endTime, distance, printListOfPoints(visited))

def bfsTraversal(visited, graph, node):
    queue = list()
    visited.append(node)
    queue.append(node)
    while queue:
        m = queue.pop(0)
        print(str(m.x) + ',' + str(m.y))
        if m in graph:
            for neighbor in graph[m]:
                if neighbor not in visited:
                    visited.append(neighbor)
                    queue.append(neighbor)


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
        stringOfCombination += f"({point.x}, {point.y}) "
    return stringOfCombination

def runSimulation():

    #we'll want to have the same list of cities for these algorithms to run through in the final version
    #that way the data can be as accurately compared as possible
    #but this is fine for verifications

    csvDictionary = []
    fields = ['Name Of Algorithm', 'Indicator of Algorithm', 'Number of Cities', 'Area in Units', 'Time in Seconds', 'Distance']

    numberOfReplicates = 0
    while numberOfReplicates != 3:
        numberOfCities = 95
        while numberOfCities != 100:
            area = 27
            while area != 30:
                pointList = generatePoints(numberOfCities,area)
                # solution = bruteForce(pointList)
                # csvDictionary.append({'Name Of Algorithm': 'Brute Force', 'Indicator of Algorithm': 1, 'Number of Cities': numberOfCities, 'Area in Units': area * area, 'Time in Seconds': solution.time, 'Distance': solution.distance})
                solution = closestEdge(pointList)
                csvDictionary.append({'Name Of Algorithm': 'Closest Edge', 'Indicator of Algorithm': 2, 'Number of Cities': numberOfCities, 'Area in Units': area * area, 'Time in Seconds': solution.time, 'Distance': solution.distance})
                solution = dfs(pointList)
                csvDictionary.append({'Name Of Algorithm': 'Depth First Search', 'Indicator of Algorithm': 3, 'Number of Cities': numberOfCities, 'Area in Units': area * area, 'Time in Seconds': solution.time, 'Distance': solution.distance})
                solution = bfs(pointList)
                csvDictionary.append({'Name Of Algorithm': 'Breadth First Search', 'Indicator of Algorithm': 4, 'Number of Cities': numberOfCities, 'Area in Units': area * area, 'Time in Seconds': solution.time, 'Distance': solution.distance})
                area = area + 1
            numberOfCities = numberOfCities + 1
        numberOfReplicates = numberOfReplicates + 1
    with open('results.csv', 'w', newline='') as file: 
        writer = csv.DictWriter(file, fieldnames = fields)
        writer.writeheader() 
        writer.writerows(csvDictionary)
    #we'll run all 4 algorithms
    #collect their data
    #run analysis
    #print results
    return

runSimulation()