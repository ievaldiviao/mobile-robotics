# -*- coding: utf-8 -*-
"""
Created on Thu Sep 20 00:53:48 2018

@author: ASUS
"""

import vrep
import matplotlib.patches as patches
import time
import math as m
import numpy as np
import cv2
from dijkstar import Graph, find_path

###################posiciones del ROBOT
posrobotx=168
posroboty=33
destinox=275
destinoy=190

######################

########################Lectura del mapa previamente generado
mapa=cv2.imread("GrisRecortado.jpg", cv2.IMREAD_GRAYSCALE)

for i in range (300):
    for j in range (300):
        if mapa[i][j]<50:
            mapa[i][j]=0
        else:
            mapa[i][j]=1


#####################Codigo robado xd
class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)
#######################################FIN DE CODIGO ROBADO

#########################################Calculando giros
def Pendiente (x1, y1, x2, y2):
    if (x2-x1==0):
        return "I"
    else:
        return ((y2-y1)/(x2-x1))

def angulo (m2, m1):
    if (m1=="I" and m2=="I"):
        return 0
    if (m2=="I"):
        return (-90-m.degrees(m.atan(m1)))
    if (m1=="I"):
        return (m.degrees(m.atan(m2))+90)
    elif (m1*m2==-1):
        return 90
    else:
        return m.degrees((m.atan((m2-m1)/(1+m1*m2))))


#Iniciandoliznado VREP###############
vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID!=-1:
    print('Conexion Lista')
else:
    print('Error')    
_, left_motor_handle=vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", vrep.simx_opmode_oneshot_wait)
_, right_motor_handle=vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", vrep.simx_opmode_oneshot_wait)
vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, vrep.simx_opmode_streaming)
vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, vrep.simx_opmode_streaming)
############################
maze = mapa
start = (posrobotx, posroboty)
end = (destinox,  destinoy)

path = astar(maze, start, end)

#print(path)
for i in range (300):
    for j in range (300):
        if mapa[i][j]==1:
            mapa[i][j]=255
        else:
            mapa[i][j]=0
       
              
for i in path:
    mapa[(int)(i[0])][(int)(i[1])]=255
cv2.imwrite( "C:/Users/ASUS/Desktop/Java/RoboticaM/TrazoGeometrico.jpg", mapa);
cv2.imshow('Color image', mapa)
cv2.waitKey(0)

giro=[]
c=1
difx=0
dify=1
####Funciones para mover al ROBOT
def GirarIzquierda(Grados):
     vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, vrep.simx_opmode_streaming)
     vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, vrep.simx_opmode_streaming)
     time.sleep(2)
     vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, vrep.simx_opmode_streaming)
     vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 1.0, vrep.simx_opmode_streaming)
     time.sleep(5.37)
     
def GirarDerecha(Grados):
     vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, vrep.simx_opmode_streaming)
     vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, vrep.simx_opmode_streaming)
     time.sleep(2)
     vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 1, vrep.simx_opmode_streaming)
     vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, vrep.simx_opmode_streaming)
     time.sleep(5.37)
     
def MoverRecto():
     vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 8, vrep.simx_opmode_streaming)
     vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 8, vrep.simx_opmode_streaming)
     time.sleep(0.33)
####Movimiento ROBOT
for i in path:
    if c!=1:
        giro=(angulo(Pendiente(posroboty, posrobotx, posroboty+dify, posrobotx+difx), Pendiente(posroboty, posrobotx, i[1], i[0])))
        difx=i[0]-posrobotx
        dify=i[1]-posroboty
        if giro>0:
            if giro>90:
                giro=giro-180
                GirarDerecha(giro)
            else:
                GirarIzquierda(giro)   
        elif giro==0:
            MoverRecto()
        elif giro<0:
            if giro<-90:
                giro=giro+180
                GirarIzquierda(giro)
            else:
                GirarDerecha(giro)
            
        posrobotx=i[0]
        posroboty=i[1]
    else:
        c=0
vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, vrep.simx_opmode_streaming)
vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, vrep.simx_opmode_streaming)


    #iniciox=i[0]
    #inicioy=i[1]
    

    
