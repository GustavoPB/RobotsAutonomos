import cv2 as cv
import numpy as np

#Definicion de posicion
class Possition(object):

    def __init__(self, xval, yval):
        self.x = xval
        self.y = yval

#Definicion Clase Nodo:
class Node(object):

    def __init__(self, posx, posy, parentPosx, parentPosy):
        self.pos = Possition(posx, posy)
        self.parentPos = Possition(parentPosx, parentPosy)
        self.isClosed = False
        self.heuristicVal = 0
        self.fitness = 0

    def CalculateHeuristic(self, coordTargetNode):
        x_dist = self.pos.x-coordTargetNode.x
        y_dist = self.pos.y-coordTargetNode.y
        self.heuristicVal = np.sqrt(np.square(x_dist) + np.square(y_dist))

    def getNeighbours(self):
        list = []
        for i in range(-1, 2):
            coord_x = self.pos.x+i
            for j in range(-1, 2):
                coord_y = self.pos.y+j
                if np.absolute(i) + np.absolute(j) != 0: #impedimos que se almecene a si mismo
                    list.append(Possition(coord_x, coord_y  ))

        return list

    def Equals(self, nodePos):
        result = False
        if self.pos.x == nodePos.x and self.pos.y == nodePos.y:
            result = True
        else:
            result = False
        return result
            
    def Close(self):
        if self.isClosed == False:
            self.isClosed = True

#Definimos la clase motor del algoritmo
class Algorithm(object):

    def __init__(self):
        self.startNode = Node(0,0,0,0)
        self.endNode = Node(0,0,0,0)
        self.closedList = [] #almacenara los nodos elegidos para exploracion
        self.openList = [] #almacenara los nodos explorados pendientes de eleccion
    
    def setStartingNode(self, node):
        self.startNode = node
        node.Close()
        self.closedList.append(node)
    
    def setEndingNode(self, node):
        self.endNode = node
        self.openList.append(node)

    def CloseNode(self, posTargetNode):
        for node in self.openList:
            if node.Equals(posTargetNode):
                node.Close()
                self.openList.remove(node)
                self.closedList.append(node)
                return





n = Node(5,5,0,0)

m = n.getNeighbours() #Prueba

for item in m:
    print str(item.x) + ' ' + str(item.y)

print '-------'

n.CalculateHeuristic(Possition(6,4))#Prueba
print n.heuristicVal

n.CalculateHeuristic(Possition(6,5))#Prueba
print n.heuristicVal

print '-------'

#Prueba algorithm
alg = Algorithm()
alg.setStartingNode(n)

c = Node(7,7,0,0)
alg.setEndingNode(c)

for node in alg.openList:
    print str(node.pos.x) + ' ' + str(node.pos.y) + ' ' + str(node.isClosed)

for node in alg.closedList:
    print str(node.pos.x) + ' ' + str(node.pos.y) + ' ' + str(node.isClosed)

print '-------'

p = Possition(7,7)
alg.CloseNode(p)

for node in alg.openList:
    print str(node.pos.x) + ' ' + str(node.pos.y) + ' ' + str(node.isClosed)

for node in alg.closedList:
    print str(node.pos.x) + ' ' + str(node.pos.y) + ' ' + str(node.isClosed)


