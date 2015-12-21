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
        self.effort = 0
        self.fitness = self.heuristicVal + self.effort

    def setHeuristic(self, coordTargetNode):
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
                    list.append(Possition(coord_x, coord_y))

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

    def setParent(self, pPos):
        self.parentPos.x = pPos.x
        self.parentPos.y = pPos.y
        self.effort = np.sqrt(np.square(self.pos.x - self.parentPos.x) + np.square(self.pos.y - self.parentPos.y))

    def setFitness(self):
        self.fitness = self.heuristicVal + self.effort

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

    def CloseNode(self, posTargetNode):
        for node in self.openList:
            if node.Equals(posTargetNode):
                node.Close()
                self.openList.remove(node)
                self.closedList.append(node)
                return node
    
    def isNodeInOpenList(self, posTargetNode):
        result = False
        foundNode = Node(0,0,0,0)
        for node in self.openList:
            if node.Equals(posTargetNode):
                result = True
                foundNode = node
            else:
                result = False
        return result, foundNode
    
    def isNodeInClosedList(self, posTargetNode):
        result = False
        for node in self.openList:
            if node.Equals(posTargetNode):
                result = True
            else:
                result = False
        return result

    def UpdateNodeInOpenList(self, targetNode): #No es neceario calcular heurisitica - se hico en incorporacion a lista
        self.openList.remove(targetNode)
        targetNode.setFitness()
        self.openList.append(targetNode)

    def Explore(self, fromNode):
        exploredNode = self.CloseNode(fromNode) #ToReview
        neighbours = exploredNode.getNeighbours()

        for neighbour in neighbours:
            check, node = self.isNodeInOpenList(neighbour)

            if check == True:
                node.setParent(fromNode)
                self.UpdateNodeInOpenList(node)
            elif self.isNodeInClosedList(fromNode):
                #Ignorar Nodo, no hacer nada
                continue
            else:
                #implica que tiene que ser anadido a la lista abierta como opcion
                newNode = Node(neighbour.x, neighbour.y, 0, 0)
                newNode.setParent(fromNode)
                newNode.setHeuristic(self.endNode.pos)
                newNode.setFitness()
                self.openList.append(newNode)




n = Node(5,5,0,0)

m = n.getNeighbours() #Prueba

for item in m:
    print str(item.x) + ' ' + str(item.y)

print '-------'

n.setHeuristic(Possition(6,4))#Prueba
print n.heuristicVal

n.setHeuristic(Possition(6,5))#Prueba
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


print '-------'

punto = Possition(10,10)

newNode = Node(10,10, 0,0)
alg.openList.append(newNode)

alg.Explore(punto)
for node in alg.openList:
    print str(node.pos.x) + ' ' + str(node.pos.y) + ' ' + str(node.isClosed)
for node in alg.closedList:
    print str(node.pos.x) + ' ' + str(node.pos.y) + ' ' + str(node.isClosed)



