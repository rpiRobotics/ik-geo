from math import atan2,sqrt
import numpy as np

#Handle rand_py import
import rand_py as rand


"""
Inputs:
   p0, p1, p2, p3: 3x1 vector
   k1, k2, k3: 3x1 vector w/ norm = 1
Outputs:
   theta1: 1xN angle vec in rad, N = num sols
   is_LS: 1x1 bool

This problem will be ill-posed if p is parallel to k or h
"""

#Setup for non-least squares version
def sp4_setup(p, k, h):
   #Clear all matrices to pass by ref
   p *= 0
   k *= 0
   h *= 0

   #Then pass in new values
   p += rand.rand_vec()
   k += rand.rand_normal_vec()
   h += rand.rand_normal_vec()
   
   theta = rand.rand_angle()
   #print("theta=", theta, "\r\n") #If seeing theta is desired, uncomment this.

   #Return d
   return h @ rand.rot(k, theta) @ p


def sp4_setup_LS(p, k, h):
   #Clear all matrices to pass by ref
   p *= 0
   k *= 0
   h *= 0

   #Then pass in new values
   p += rand.rand_vec()
   k += rand.rand_normal_vec()
   h += rand.rand_normal_vec()

   #Return d
   return rand.r.random() #rand 0-1


def sp4_run(p, k, h, d):
   A11 = np.cross(k,p)
   A1 = np.concatenate(([A11], [-np.cross(k,A11)]), axis = 0)
   A = h @ A1.T

   b = d - h @ np.reshape(k, (3, 1)) @ (k @ np.reshape(p, (3, 1)))
   norm_A2 = A @ A
   x_ls = A1 @ (h * b)

   #If a non-least squares version
   if norm_A2 > pow(b, 2):
      xi = sqrt(norm_A2-pow(b, 2))
      A_perp_tilde = np.block([A[1], -A[0]])
      
      sc_1 = x_ls + xi * A_perp_tilde
      sc_2 = x_ls - xi * A_perp_tilde

      #theta = [atan2(sc_1[0], sc_1[1]), atan2(sc_2[0], sc_2[1])]
      return np.array([atan2(sc_1[0], sc_1[1]), atan2(sc_2[0], sc_2[1])]), False
   
   #Otherwise problem is least squares
   else:
      #theta = atan2(x_ls[0], x_ls[1])
      return atan2(x_ls[0], x_ls[1]), True

def sp4_error(p, k, h, d, theta):
   #If least-squares:     theta is a float
   #If not least-squares: theta is a np.array
   if type(theta) != float:
      e = 0.0 #Sum variable
      #Perform calculations on each index
      for i in range(len(theta)):
         e_i = np.linalg.norm(h @ rand.rot(k, theta[i]) @ p) - d
         e += e_i
   #If least-squares, then no summation needed
   else:
      e = np.linalg.norm(h @ rand.rot(k, theta) @ p) - d
   return e



#Test Code
if __name__ == "__main__":
   print("Starting arrays \r\n")
   p = np.array([1., 2., 3.])
   k = np.array([1., 2., 3.])
   h = np.array([1., 2., 3.])

   #Setup problem
   print("After setup: \r\n")
   d = sp4_setup_LS(p, k, h)
   np.set_printoptions(precision = 20)
   print("p = {}\r\nk = {}\r\nh  = {}\r\nd  = {}\r\n".format(p, k, h, d))

   theta, is_LS = sp4_run(p, k, h, d) #Save values

   #Printing out results
   print("Results:\r\ntheta: {}\r\nis_LS: {}\r".format(theta, is_LS))
   print("Error:", sp4_error(p, k, h, d, theta), "\r\n")