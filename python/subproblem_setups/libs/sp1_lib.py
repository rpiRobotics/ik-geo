from math import atan2
import numpy as np
 
#Handle rand_py import
import rand_py as rand


#Inputs: 3x1 numpy arrays p1, p2, k
#Python does not pass by reference like C++, must manipulate without assignment
#Theta is calculated outside this function as opposed to MATLAB version
def sp1_setup(p1, p2, k, theta):
   #Clear all matrices
   p1 *= 0
   p2 *= 0
   k *= 0

   #Then fill in values
   p1 += rand.rand_vec()
   k += rand.rand_normal_vec()
   p2 += rand.rot(k, theta) @ p1 #@ is index wise multiplication
 

#Returns theta, is_LS
def sp1_run(p1, p2, k):
   #p2 *= 0
   #p2 += rand.rot(k, theta) @ p1

   KxP = np.cross(k, p1)
   A = np.vstack((KxP, -np.cross(k, KxP)))

   x = np.dot(A, p2)
   
   #theta = atan2(x[0], x[1])

   return atan2(x[0], x[1]), abs(np.linalg.norm(p1, 2) - np.linalg.norm(p2, 2)) > 1e-8 or abs(np.dot(k,p1) - np.dot(k,p2)) > 1e-8


#Inputs: 3x1 numpy arrays p1, p2, k
#Python does not pass by reference like C++, must manipulate without assignment
#Theta is calculated outside this function as opposed to MATLAB version
def sp1_setup_LS(p1, p2, k):
   #Clear all matrices
   p1 *= 0
   p2 *= 0
   k *= 0

   #Then fill in values
   p1 += rand.rand_vec()
   k += rand.rand_normal_vec()
   p2 += rand.rand_vec()


def sp1_error(p1, p2, k, theta):
   return np.linalg.norm(p2 - rand.rot(k, theta)@p1, 2)


#Testing
if __name__ == "__main__":
   print("Starting arrays \r\n")
   p1 = np.array([1., 2., 3.])
   p2 = np.array([1., 2., 3.])
   k = np.array([1., 2., 3.])
   print("Passed arrays \r\n")

   theta = rand.rand_angle()

   sp1_setup(p1, p2, k, theta)

   print("After running: ", theta, "\r\n", p1, "\r\n\n", p2, "\r\n\n", k, "\r\n\n")

   theta, is_LS =sp1_run(p1, p2, k)