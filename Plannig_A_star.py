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
        self.map = cv.imread('crea3.jpg')
        self.PrintMap()
    
    def PrintMap(self):
        cv.imshow("Window", self.map)
    
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
                break
            else:
                result = False
        return result, foundNode
    
    def isNodeInClosedList(self, posTargetNode):
        result = False
        for node in self.closedList:
            if node.Equals(posTargetNode):
                result = True
                break
            else:
                result = False
        return result

    def UpdateNodeInOpenList(self, targetNode): #No es neceario calcular heurisitica - se hico en incorporacion a lista
        self.openList.remove(targetNode)
        targetNode.setFitness()
        self.openList.append(targetNode)

    def getNonWallNodePossitions(self, nodePosCollection):
        wallNodes = []
        for nodePos in nodePosCollection:
            pixelColor = self.map[nodePos.x, nodePos.y]
            result = True if (pixelColor == 255).all() else False
            if result == False: #no es blanco
                wallNodes.append(nodePos)
        for node in wallNodes:
            nodePosCollection.remove(node)
        return nodePosCollection

    def Explore(self, fromNode):
        exploredNode = self.CloseNode(fromNode) #ToReview
        neighbours = exploredNode.getNeighbours()
        
        neighbours = self.getNonWallNodePossitions(neighbours)#eliminamos items en muros

        for neighbour in neighbours:
            check, node = self.isNodeInOpenList(neighbour)

            if check == True:
                node.setParent(fromNode)
                self.UpdateNodeInOpenList(node)
            elif self.isNodeInClosedList(neighbour):
                #Ignorar Nodo, no hacer nada
                continue
            else:
                #implica que tiene que ser anadido a la lista abierta como opcion
                newNode = Node(neighbour.x, neighbour.y, 0, 0)
                newNode.setParent(fromNode)
                newNode.setHeuristic(self.endNode.pos)
                newNode.setFitness()
                self.openList.append(newNode)

    def getBestNodeFromOpenList(self):
        bestNode = Node(0,0,0,0)
        bestNode.fitness = float("inf")
        for node in self.openList:
            if node.fitness < bestNode.fitness:
                bestNode = node
        return bestNode

# ************* Metodos y propiedades globales
mousePoints = []

def CallBackFunc(event, x, y, flags, param):
    global mousePoints
    if event == cv.EVENT_LBUTTONDOWN and len(mousePoints) < 2:
        print 'append? x:' + str(x) + ' y:' + str(y) + ' -- press y: Yes'
        key = cv.waitKey(0)
        if key == ord("y"):
            pos = Possition(x,y)
            mousePoints.append(pos)
            print 'Point appended'
            
            if len(mousePoints) == 2:
                print 'Route fixed!'
        else:
            print 'Point discarded'

def DrawFilledCircle(image, center):
    cv.circle(image, (center.x, center.y), 3, ( 0, 0, 255 ), -1)


#--------PRUEBAS UNITARIAS---------
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

punto = Possition(5,6)

newNode = Node(5,6, 0,0)
alg.openList.append(newNode)

alg.Explore(punto)
for node in alg.openList:
    print str(node.pos.x) + ' ' + str(node.pos.y) + ' ' + str(node.isClosed) + ' Parent: ' + str(node.parentPos.x) + ' ' + str(node.parentPos.y) + ' --- ' + str(node.fitness)
for node in alg.closedList:
    print str(node.pos.x) + ' ' + str(node.pos.y) + ' ' + str(node.isClosed) + ' Parent: ' + str(node.parentPos.x) + ' ' + str(node.parentPos.y)

besNode = alg.getBestNodeFromOpenList()
print str(besNode.pos.x) + ' ' + str(besNode.pos.y) + ' ' + str(besNode.isClosed)

cv.setMouseCallback("Window", CallBackFunc)
cv.waitKey(0)
print 'Drawing points of route'
DrawFilledCircle(alg.map, mousePoints[0])
DrawFilledCircle(alg.map, mousePoints[1])
alg.PrintMap()
cv.waitKey(0)
cv.destroyAllWindows()
print 'Fin del programa'



