import math
import heapq
import random

from Myro import *
from Graphics import *
from random import *
from math import *

class Agent(object):
    def useJoystick(self):
        return False
    def act(self, start_city):
        return False


class Taxi(Agent):
    def swapRandom(self, cities):
        i1, i2 = sample(range(10), 2)
        ci = cities[i2]
        cities[i2] = cities[i1]
        cities[i1] = ci
        return cities

    def annealing(self, cities2, temp_begin, nb_iterations, temp_end=.1, c_factor=.99):
        best_cities = cities2[:]
        for itrtn_count in range(nb_iterations):
            T = temp_begin
            cities2 = best_cities[:]
            while T > temp_end:
                prev_cities = cities2[:]
                prev_cities_distances = total_distances(cities2)
                cities2 = self.swapRandom(cities2)
                new_distance = total_distances(cities2)
                if(new_distance > prev_cities_distances and
                math.exp( -(new_distance - prev_cities_distances) / T ) < random()):
                    cities2 = prev_cities
                T = T * c_factor


        #print(best_cities)
        #print("best", best_cities)
        #print(cities)
        #print("current", cities2)

            if(total_distances(cities2) < total_distances(best_cities)):
                best_cities = cities2[:]

        print("Shortest Path Cost:", total_distances(best_cities))
        print("City Sequence:", best_cities)
        return best_cities


    def rotateTaxi(self, angle):
        stop()
        c_Angle = 0
        c_Angle = getAngle()
        c_Angle = c_Angle % 360 - 360
        if(angle < 0):
            angle = angle + 360

        if(angle <= 180):
            if c_Angle > -angle - 180 and c_Angle < -angle:
                #print("Turn left")
                stop()
                robot.move(0, 0.1)
            else:
                stop()
                robot.move(0, -0.1)
        else:
            if c_Angle > -angle and c_Angle < -angle + 180:
                stop()
                robot.move(0, -0.1)
            else:
                stop()
                robot.move(0, 0.1)

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
            print("coordinate")
            print("x:", x_current, "y:", y_current)
            if abs(y_nextCity - y_current) < 5 and abs(x_nextCity - x_current) < 10:
                stop()
                break
            angle = atan2(y_nextCity - y_current, x_nextCity - x_current) * 180 / math.pi
            self.rotateTaxi(angle)



    def taxiMove(self, path):
        print("TAXI MOVE:", path)
        for nextCity in path:
            self.gotoNextCity(nextCity[2])

    def act(self, cities):
        #aPath = self.aStarSearch(start_city)
        #print(aPath)
        drawPath(generatePath(cities), sim, "blue")

        i = cities[0]
        x_s = i[0]
        y_s = i[1]
        sim.setPose(0, x_s - 7, y_s, 2)
        self.taxiMove(cities)

    def actSA(self, cities, T, nb):
        best_cities = self.annealing(cities, T, nb)
        drawPath(generatePath(best_cities), sim, "red")

        i = best_cities[0]
        x_s = i[0]
        y_s = i[1]
        sim.setPose(0, x_s - 7, y_s, 2)
        self.taxiMove(best_cities)




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


def compute_distances(cities):
    for city1 in cities:
        d = {}
        for city2 in cities:
            d[city2[2]] = compute_twoCities_distance(city1[0], city1[1], city2[0], city2[1])
        distances[city1[2]] = d

def compute_twoCities_distance(x1, y1, x2, y2):
    return math.ceil(math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)))


#print(distances)


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

def addPath(x1, y1, x2, y2, sim, color):
    line = Line(Point(x1, y1), Point(x2, y2))
    line.color = Color(color)
    line.border = 4
    line.draw(sim.window)


def total_distances(cities):
    sum = 0
    for i in range(9):
        sum += distances[cities[i][2]][cities[i+1][2]]
    return sum


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

    s.setup()
    return s



def generatePath(cities):
    paths = []
    for i in range(9):
        buff = []
        buff.append(cities[i][0])
        buff.append(cities[i][1])
        buff.append(cities[i+1][0])
        buff.append(cities[i+1][1])
        paths.append(buff)
    return paths

def drawPath(paths, sim, color):
    for p in paths:
        addPath(p[0], p[1], p[2], p[3], sim, color)



cities = [[800, 570, 'Bucharest'], [950, 160, 'Neamt'],
            [280, 625, 'Drobeta'], [310, 375, 'Sibiu'],
            [1090, 390, 'Vaslui'], [85, 440, 'Timisoara'],
            [70, 300, 'Arad'], [170, 50, 'Oradea'],
            [380, 440, 'Rimnicu Vilcea'], [1150, 530, 'Hirsova']]

distances = {}



sim = Simulation("Romanian", 1300, 700, Color("white"))
sim = setupSim(sim, 1300, 700, 3)
robot = makeRobot("SimScribbler", sim)

compute_distances(cities)
print("Initial Paths Cost", total_distances(cities))
print("City Sequence:", cities)
agent = Taxi()


T = float(input("Please input the Temperature"))
nb = int(input("Please input the number of iteration"))
print("T = ", T, "nb_iteration = ", nb)
agent.act(cities)
agent.actSA(cities, T, nb)
