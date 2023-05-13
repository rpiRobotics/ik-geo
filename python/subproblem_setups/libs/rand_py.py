import numpy as np #Used for matrix operations
import math        #Used for pi
import random as r

def hat(k):
   return np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])

#Create a random angle
def rand_angle():
   return round(r.random(), 15)*2*math.pi-math.pi

#Create random matrix of size 3 x size (rows, cols)
def rand_mat(size = 1): #Make list of lists, then pass to numpy
   return np.array([[round(r.random(), 15) for x in range(3)] for y in range(size)])

#Create random vector of size 3 x N
def rand_vec(size = 3): #Make list of lists, then pass to numpy
   return np.array([round(r.random(), 15) for y in range(size)])

#Create random normal vector
def rand_normal_vec(size = 3):
   vec = rand_vec(size)
   return vec/np.linalg.norm(vec, 2)

#Create random normal matrix
def rand_normal_mat(size = 1):
   #vec = rand_vec()
   #return np.array([[rand_vec()/] for y in range(0, 3)])
   lst = np.zeros((size, 3))
   for x in range(size):
      lst[x] = [round(r.random(), 15) for y in range(3)]
      lst[x] = lst[x] / np.linalg.norm(lst[x], 2)
   
   return lst

#Create rand perp vec from input numpi.array
def rand_perp_normal_vec(inp):
   randCross = rand_vec(1)
   randCross = np.cross(randCross, inp)
   return randCross/np.linalg.norm(randCross, 2)

def rot(k, theta):
   k = k/np.linalg.norm(k,2)
   return np.eye(3) + math.sin(theta)*hat(k)+(1-math.cos(theta))*hat(k)@hat(k)
   