from math import atan2,sqrt
import numpy as np

#Handle rand_py import
import rand_py as rand

"""
Inputs:
   p1, p2: 3x1 vector
   k: 3x1 vector w/ norm = 1
   d: 1x1 dependent on is_LS
Outputs:
   theta1: 1xN angle vec in rad, N = num sols
   is_LS: 1x1 bool

This problem will be ill-posed if p1 or p2 is parallel to k
"""

#Setup for non-least squares version
def sp3_setup(p1, p2, k):
   #Clear all matrices to pass by ref
   p1 *= 0
   p2 *= 0
   k *= 0

   #Then pass in new values
   p1 += rand.rand_vec()
   p2 += rand.rand_vec()
   k += rand.rand_normal_vec()
   
   theta = rand.rand_angle()
   #print("theta=", theta, "\r\n") #If seeing theta is desired, uncomment this.

   #Return d
   return np.linalg.norm(p2 - rand.rot(k, theta) @ p1, 2)


#Setup for least squares version
def sp3_setup_LS(p1, p2, k):
   #Clear all matrices to pass by ref
   p1 *= 0
   p2 *= 0
   k *= 0

   #Then pass in new values
   p1 += rand.rand_vec()
   p2 += rand.rand_vec()
   k += rand.rand_normal_vec()

   #Return d
   return rand.r.random() #Random num between 0-1
   

#Run the problem and return theta
def sp3_run(p1, p2, k, d):
   KxP = np.cross(k, p1)
   #np.concatenate stacks two arrays here: [arr1; arr2]
   A1 = np.concatenate(([KxP], [-np.cross(k, KxP)]), axis = 0)
   A = -2*p2@A1.T

   norm_A_sq = np.dot(A,A)
   norm_A = sqrt(norm_A_sq)
   b = pow(d,2) - pow(np.linalg.norm(p2 - np.reshape(k, (3,1)) @ np.reshape(k, (1,3)) @ p1, 2), 2) - pow(np.linalg.norm(KxP, 2), 2)

   x_ls = A1@(-2*p2*b/norm_A_sq)

   #Check to see if this is least squares version
   if x_ls @ x_ls > 1:
      #theta = atan2(x_ls[0], x_ls[1])
      return atan2(x_ls[0], x_ls[1]), True
   
   #Otherwise not a least squares
   else:
      xi = sqrt(1-pow(b, 2)/norm_A_sq)

      A_perp_tilde = np.block([A[1], -A[0]])
      A_perp = A_perp_tilde/norm_A

      sc_1 = x_ls + xi * A_perp
      sc_2 = x_ls - xi * A_perp

      #theta = [atan2(sc_1[0], sc_1[1]), atan2(sc_2[0], sc_2[1])]
      return np.array([atan2(sc_1[0], sc_1[1]), atan2(sc_2[0], sc_2[1])]), False


#Calculate error
def sp3_error(p1, p2, k, d, theta):
   #If least-squares:     theta is a float
   #If not least-squares: theta is a np.array
   if type(theta) != float:
      e = 0.0 #Sum variable
      #Perform calculations on each index
      for i in range(len(theta)):
         e_i = abs(np.linalg.norm(p2 - rand.rot(k, theta[i]) @ p1, 2) - d)
         e += e_i
   #If least-squares, then no summation needed
   else:
      e = abs(np.linalg.norm(p2 - rand.rot(k, theta) @ p1, 2) - d)
   return e



#Test Code
if __name__ == "__main__":
   print("Starting arrays \r\n")
   p1 = np.array([1., 2., 3.])
   p2 = np.array([1., 2., 3.])
   k = np.array([1., 2., 3.])

   #Setup problem
   print("After setup: \r\n")
   d = sp3_setup_LS(p1, p2, k)
   np.set_printoptions(precision = 20)
   print("p1 = {}\r\np2 = {}\r\nk  = {}\r\nd  = {}\r\n".format(p1, p2, k, d))

   theta, is_LS = sp3_run(p1, p2, k, d) #Save values

   #Printing out results
   print("Results:\r\ntheta: {}\r\nis_LS: {}\r".format(theta, is_LS))
   print("Error:", sp3_error(p1, p2, k, d, theta), "\r\n")