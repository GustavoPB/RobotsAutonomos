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
        self.wasTaken = 0
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

n = Node(5,5,0,0)

m = n.getNeighbours()

for item in m:
    print str(item.x) + ' ' + str(item.y)

print '-------'

n.CalculateHeuristic(Possition(6,4))
print n.heuristicVal

n.CalculateHeuristic(Possition(6,5))  
print n.heuristicVal