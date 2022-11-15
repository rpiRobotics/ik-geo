from math import atan2,sqrt
import numpy as np

#Handle rand_py import
if __name__ == "__main__":
   import rand_py as rand
else:
   import libs.rand_py as rand

"""
Inputs:
   p0, p1, p2: 3x1 vector
   k1, k2: 3x1 vector w/ norm = 1
Outputs:
   theta1, theta2: 1x1 angle in rads
"""

#Setup subproblem
def sp2_setup(p0, p1, p2, k1, k2):
   #Clear all matrices to pass by ref
   p0 *= 0
   p1 *= 0
   p2 *= 0
   k1 *= 0
   k2 *= 0

   #Then pass in new values
   p0 += rand.rand_vec()
   p1 += rand.rand_vec()
   k1 += rand.rand_normal_vec()
   k2 += rand.rand_normal_vec()

   theta1 =  rand.rand_angle()
   theta2 = rand.rand_angle()

   #Calculate p2 using rand angles and above 
   #@ is index wise multiplication
   print("S.theta1=",theta1, "\r\n","S.theta2=", theta2, "\r\n")
   p2 += rand.rot(k2, -theta2) @ (p0 + rand.rot(k1, theta1) @ p1)

def sp2E_run(p0, p1, p2, k1, k2):
   KxP1 = np.cross(k1, p1)
   KxP2 = np.cross(k2, p2)
   
   print("KxP1: ", KxP1, "\r\n")
   #np.block appends two arrays together
   A1 = np.block([KxP1, -np.cross(k1, KxP1)])
   A2 = np.block([KxP2, -np.cross(k2, KxP2)])

   #print("A1: ", A1, "\r\nA2: ", A2, "\r\n")

   A = np.block([A1, -A2])
   print("A: ", A, "\r\n")
   print("A: ", np.reshape(A, (4,3)), "\r\n")

   A = np.reshape(A, (4,3))

   p = -k1*np.dot(k1, p1) + k2*np.dot(k2, p2) - p0
   
   radius_1_sq = np.dot(KxP1, KxP1)
   radius_2_sq = np.dot(KxP2, KxP2)

   alpha = radius_1_sq/(radius_1_sq+radius_2_sq)
   beta  = radius_2_sq/(radius_1_sq+radius_2_sq) 

   #reshape vector to matrix w/ opposite dimensions
   M_inv = np.eye(3) + (k1 * np.reshape(k1, (3,1)))*(alpha/(1.0-alpha)) 

   print("M_inv: ", M_inv, "\r\n")
   AAT_inv = 1.0/(radius_1_sq+radius_2_sq)*(M_inv + M_inv@np.array([k2]).T*k2  @ M_inv*beta/(1.0-k2@M_inv@np.array([k2]).T*beta))
   print("AAT_inv: ", AAT_inv, "\r\n")

   print("P: ", p, "\r\nTest: ", A@AAT_inv, "\r\n")
   x_ls = np.dot(A@AAT_inv,p)

   print("x_ls: ", x_ls, "\r\n")

   #Resume checking validity on line 43 of sp_2E.m
   n_sym = np.cross(k1, k2)
   pinv_A1 = A1.T/radius_1_sq;
   pinv_A2 = A2.T/radius_2_sq;
   



#Test Code
if __name__ == "__main__":
   print("Starting arrays \r\n")
   p0 = np.array([1., 2., 3.])
   p1 = np.array([1., 2., 3.])
   p2 = np.array([1., 2., 3.])
   k1 = np.array([1., 2., 3.])
   k2 = np.array([1., 2., 3.])
   
   #Setup problem
   sp2_setup(p0, p1, p2, k1, k2)
   print("After setup \r\n")
   np.set_printoptions(precision = 20)
   #print("P.p0={}\r\nP.p1={}\r\nP.k1={}\r\nP.k2={}\r\n\nP.p2={}\r\n".format(p0, p1, k1, k2, p2))
   print("p0={}\r\np1={}\r\nk1={}\r\nk2={}\r\n\np2={}\r\n".format(p0, p1, k1, k2, p2))


   t1, t2 = sp2E_run(p0, p1, p2, k1, k2) #Save values

   """
   #Printing out results
   print("Results:\r\ntheta1:\n{}\r\ntheta2:\n{}\nis_LS:\n{}\r\n".format(t1, t2, is_LS))
   print("Error:", sp2_error(p1, p2, k1, k2, t1, t2), "\r\n")
   """