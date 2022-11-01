import numpy as np #Used for matrix operations
import math        #Used for pi
import random as r

def hat(k):
   return np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])

#Create a random angle
def rand_angle():
   return r.random()*2*math.pi-math.pi

#Create random matrix of size 3 x num (rows, cols)
def rand_mat(size = 1): #Make list of lists, then pass to numpy
   return np.array([[r.random() for x in range(0,size)] for y in range(0, 3)])

   #Create random matrix of size 3 x num (rows, cols)
def rand_vec(): #Make list of lists, then pass to numpy
   return np.array([r.random() for y in range(0, 3)])

#Create random normal vector
def rand_normal_vec(size = 1):
   vec = rand_vec()
   return vec/np.linalg.norm(vec, 2)

#Create rand perp vec from input numpi.array
def rand_perp_normal_vec(inp):
   randCross = rand_vec(1)
   randCross = np.cross(randCross, inp)
   return randCross/np.linalg.norm(randCross, 2)

def rot(k, theta):
   k = k/np.linalg.norm(k,2)
   return np.eye(3) + math.sin(theta)*hat(k)+(1-math.cos(theta))*hat(k)@hat(k)