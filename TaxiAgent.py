import math
import heapq
from Myro import *
from Graphics import *
from random import *
from math import *
state=0
goal=False
utility=0
width, height = 1300, 700
cities = [[1200, 650, "Eforie"], [1150, 530, "Hirsova"], [950, 530, "Urziceni"], [1090, 390, "Vaslui"],
         [1070, 260, "Iasi"], [950, 160, "Neamt"], [800, 570, "Bucharest"], [690, 670, "Giurgiu"],
         [630, 490, "Pitesti"], [600, 390, "Fagaras"], [480, 660, "Craiova"], [380, 440, "Rimnicu Vilcea"],
         [310, 375, "Sibiu"], [170, 50, "Oradea"], [70, 300, "Arad"], [100, 175, "Zerind"],
         [85, 440, "Timisoara"], [280, 475, "Lugoj"], [290, 550, "Mehadia"], [280, 625, "Drobeta"]]

paths = [[1150, 650, 1100, 530], [1100, 530, 900, 530], [900, 530, 1040, 390], [1040, 390, 1020, 260],
         [1020, 260, 900, 160], [900, 530, 750, 570], [750, 570, 640, 670], [750, 570, 550, 390],
         [750, 570, 580, 490], [550, 390, 260, 375], [580, 490, 430, 660], [430, 660, 330, 440],
         [330, 440, 260, 375], [330, 440, 580, 490], [260, 375, 120, 50], [260, 375, 20, 300], [20, 300, 50, 175],
         [50, 175, 120, 50], [20, 300, 35, 440], [35, 440, 230, 475], [230, 475, 240, 550],
         [240, 550, 230, 625], [230, 625, 430, 660]]
for i in paths:
    i[0] += 50
    i[2] += 50

graph = {"Oradea": {"Zerind": 71, "Sibiu": 151},
             "Zerind": {"Arad": 75, "Oradea": 71},
             "Sibiu": {"Oradea": 151, "Arad": 140, "Fagaras": 99, "Rimnicu Vilcea": 80},
             "Arad": {"Zerind": 75, "Sibiu": 140, "Timisoara": 118},
             "Rimnicu Vilcea": {"Sibiu": 80, "Pitesti": 97, "Craiova": 146},
             "Fagaras": {"Sibiu": 99, "Bucharest": 211},
             "Timisoara": {"Arad": 118, "Lugoj": 111},
             "Craiova": {"Rimnicu Vilcea": 146, "Pitesti": 138, "Drobeta": 120},
             "Pitesti": {"Rimnicu Vilcea": 97, "Craiova": 138, "Bucharest": 101},
             "Bucharest": {"Fagaras": 211, "Giurgiu": 90, "Urziceni": 85},
             "Lugoj": {"Timisoara": 111, "Mehadia": 70},
             "Drobeta": {"Mehadia": 75, "Craiova": 120},
             "Giurgiu": {"Bucharest": 90},
             "Urziceni": {"Bucharest": 85, "Hirsova": 98, "Vaslui": 142},
             "Mehadia": {"Lugoj": 70, "Drobeta": 75},
             "Vaslui": {"Urziceni": 142, "Iasi": 92},
             "Hirsova": {"Urziceni": 98, "Eforie": 86},
             "Iasi": {"Neamt": 87, "Vaslui": 92},
             "Eforie": {"Hirsova": 86},
             "Neamt": {"Iasi": 87}
             }

DistToBucharest = {"Oradea": 380, "Zerind": 374, "Sibiu": 253, "Arad": 366,
                   "Rimnicu Vilcea": 193, "Fagaras": 176, "Timisoara": 329, "Craiova": 140,
                   "Pitesti": 100, "Bucharest": 0, "Lugoj": 244, "Drobeta": 242,
                   "Giurgiu": 77, "Urziceni": 80, "Mehadia": 241, "Vaslui": 199,
                   "Hirsova": 151, "Iasi": 226, "Eforie": 161, "Neamt": 234}

sim = Simulation("Romanian", width, height, Color("white"))

wallLength = 100

def addCity(x, y, sim, text):
    #sim.addLight((x,y), 10, Color("red"))
    #t = Text((x, y + 28), text)
    #t.fontSize = 13
    #t.draw(sim.window)
    c = Frame(x, y)
    wheel1 = Rectangle((-10, -10), (10, 0))
    wheel2 = Rectangle((-10, 0), (10, 10))
    t = Text((0, 15), text)
    t.fontSize = 13
    wheel1.draw(c)
    wheel2.draw(c)
    t.draw(c)
    c.draw(sim.window)

def addPath(x1, y1, x2, y2, sim):
    line = Line(Point(x1, y1), Point(x2, y2))
    line.color = Color("red")
    line.border = 4
    line.draw(sim.window)


def setupSim(s, width, height, wallwidth):
    #Add lights
    s.addLight((800,570), 25, Color("orange"))

    #Add North Wall
    s.addWall((0,0), (width,wallwidth), Color("blue"))
    s.addWall((0, height-wallwidth), (width, height), Color("blue"))
    s.addWall((width - wallwidth, wallwidth), (width, height - wallwidth), Color("blue"))
    s.addWall((0, wallwidth), (wallwidth, height - wallwidth), Color("blue"))

    for c in cities:
        addCity(c[0], c[1], s, c[2])

    for p in paths:
        addPath(p[0], p[1], p[2], p[3], s)
    s.setup()
    return s

class Agent(object):
    def useJoystick(self):
        return False
    def act(self, start_city):
        return False

class Taxi(Agent):

    def aStarSearch(self, start_city):
        parentMap = {}
        pQueue = PriorityQueue()
        parentMap[start_city] = "Null", -1
        explored = {}
        for successor in graph[start_city]:
            pQueue.push(successor, graph[start_city][successor] + DistToBucharest[successor])
            parentMap[successor] = start_city, 0
            print(successor)
            print(graph[start_city][successor])

        explored[start_city] = True;
        print(pQueue.heap)
        while True:
            #print(pQueue.heap)
            city = pQueue.pop()
            print("Parent of", city, "is", parentMap[city][0])
            #print("parentMap", parentMap)
            if(city == "Bucharest"):
                stack = Stack()
                while city != start_city:
                    stack.push(city)
                    city = parentMap[city][0]
                    print(city)
                path = []
                while not stack.isEmpty():
                    city = stack.pop()
                    path.append(city)
                return path
            else:
                explored[city] = True
                for successor in graph[city]:
                    if successor not in explored.keys():
                        cost = parentMap[city][1]  + graph[city][successor]
                        pQueue.push(successor, cost + DistToBucharest[successor])
                        if(successor in parentMap.keys()):
                            if parentMap[successor][1] > cost:
                                parentMap[successor] = city, cost
                        else:
                            parentMap[successor] = city, cost


    def rotateTaxi(self, angle):
        stop()
        c_Angle = 0
        c_Angle = getAngle()
        c_Angle = c_Angle % 360 - 360
        if(angle < 0):
            angle = angle + 360
        if c_Angle > -angle - 180 and c_Angle < -angle:
            #print("Turn left")
            stop()
            robot.move(0, 0.1)
        if c_Angle <= -angle -180 or c_Angle > -angle:
            #print("Turn Right")
            stop()
            robot.move(0, -0.1)
        while True:
            c_Angle = getAngle()
            c_Angle = c_Angle % 360 - 360
            pic = takePicture()
            show(pic)
            #print("c_Angle", c_Angle)
            #print("-angle", -angle)
            if(c_Angle <= -angle and c_Angle >= -angle - 2):
                stop()
                break

    def getLocationByCity(self, cityname):
        for cityInfo in cities:
            if cityInfo[2] == cityname:
                return cityInfo[0], cityInfo[1]

    def gotoNextCity(self, nextCity):
        print(nextCity)
        x_current, y_current = getLocation()
        x_nextCity, y_nextCity = self.getLocationByCity(nextCity)
        angle = atan2(y_nextCity - y_current, x_nextCity - x_current) * 180 / math.pi
        self.rotateTaxi(angle)
        while True:
            forward(1, 0.1)
            pic = takePicture()
            show(pic)
            x_current, y_current = getLocation()
            #print("coordinate")
            #print(y_nextCity - y_current, x_nextCity - x_current)
            if y_nextCity - y_current < 5 and x_nextCity - x_current < 10:
                stop()
                break
            angle = atan2(y_nextCity - y_current, x_nextCity - x_current) * 180 / math.pi
            self.rotateTaxi(angle)



    def taxiMove(self, path):
        for nextCity in path:
            self.gotoNextCity(nextCity)

    def act(self, start_city):
        aPath = self.aStarSearch(start_city)
        print(aPath)
        self.taxiMove(aPath)



class PriorityQueue:
    def  __init__(self):
        self.heap = []

    def push(self, item, priority):
        pair = (priority,item)
        heapq.heappush(self.heap,pair)

    def pop(self):
        (priority,item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0


#addPath(1150, 650, 2, 2, sim)
#addCity(1150, 650, sim, "Eforie")

class Stack:
    def __init__(self):
        self.list = []
    def push(self,item):
        self.list.append(item)
    def pop(self):
        return self.list.pop()
    def isEmpty(self):
        return len(self.list) == 0


start_city = ""

while True:
    start_city = raw_input("Please input the name of city")
    if(start_city in graph.keys()):
        break
    else:
        print("This city is not in the map, type again!")

sim = setupSim(sim, width, height, 3)
robot = makeRobot("SimScribbler", sim)
#forward(1, 5)

for i in cities:
    if i[2] == start_city:
        x_s = i[0]
        y_s = i[1]
sim.setPose(0, x_s, y_s, 300)
agent = Taxi()

agent.act(start_city)


