from math import atan2,sqrt
import numpy as np

#Handle rand_py import
import rand_py as rand

"""
Inputs:
   p0, p1, p2: 3x1 vector
   k1, k2: 3x1 vector w/ norm = 1
Outputs:
   theta1, theta2: 1x1 angle in rads
"""

#Setup subproblem
def sp2E_setup(p0, p1, p2, k1, k2):
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
   p2 += rand.rot(k2, -theta2) @ (p0 + rand.rot(k1, theta1) @ p1)


#Run subproblem
def sp2E_run(p0, p1, p2, k1, k2):
   KxP1 = np.cross(k1, p1)
   KxP2 = np.cross(k2, p2)
   
   #np.block appends two arrays together
   A1 = np.block([KxP1, -np.cross(k1, KxP1)])
   A2 = np.block([KxP2, -np.cross(k2, KxP2)])

   #Append A1, A2 then reshape into desired shape
   A = np.block([A1, -A2])
   A = np.reshape(A, (4,3))

   p = -k1*np.dot(k1, p1) + k2*np.dot(k2, p2) - p0
   
   radius_1_sq = np.dot(KxP1, KxP1)
   radius_2_sq = np.dot(KxP2, KxP2)

   alpha = radius_1_sq/(radius_1_sq+radius_2_sq)
   beta  = radius_2_sq/(radius_1_sq+radius_2_sq) 

   #reshape vector to matrix w/ opposite dimensions
   M_inv = np.eye(3) + (k1 * np.reshape(k1, (3,1)))*(alpha/(1.0-alpha)) 

   AAT_inv = 1.0/(radius_1_sq+radius_2_sq)*(M_inv + M_inv@np.array([k2]).T*k2 @ \
             M_inv*beta/(1.0-k2@M_inv@np.array([k2]).T*beta))

   x_ls = np.dot(A@AAT_inv,p)

   n_sym = np.reshape(np.cross(k1, k2), (3,1))
   pinv_A1 = A1/radius_1_sq
   pinv_A2 = A2/radius_2_sq

   A_perp_tilde = np.reshape(np.block([pinv_A1, pinv_A2]), (4, 3)) @ n_sym

   num = (pow(np.linalg.norm(x_ls[2:4], 2), 2) - 1.0)*pow(np.linalg.norm(A_perp_tilde[0:2], 2), 2) - \
         (pow(np.linalg.norm(x_ls[0:2], 2), 2) - 1.0)*pow(np.linalg.norm(A_perp_tilde[2:4], 2), 2)
   #den was in form [[den]], so we converted to integer through indexing
   den = 2*(np.reshape(x_ls[0:2], (1,2)) @ A_perp_tilde[0:2] * pow(np.linalg.norm(A_perp_tilde[2:4], 2), 2) - \
         np.reshape(x_ls[2:4], (1,2)) @ A_perp_tilde[2:4] * pow(np.linalg.norm(A_perp_tilde[0:2], 2), 2))[0][0]
   
   xi = num/den
   #We want sc as vector/list, so we flatten both arrays
   sc = x_ls.flatten() + xi*A_perp_tilde.flatten()

   #Commented out for optimization
   #theta1 = atan2(sc[0], sc[1])
   #theta2 = atan2(sc[2], sc[3])

   return [atan2(sc[0], sc[1]), atan2(sc[2], sc[3])]


#Error calculation code
def sp2E_error(p0, p1, p2, k1, k2, theta1, theta2):
   return np.linalg.norm(p0 + rand.rot(k1, theta1)@p1 - rand.rot(k2, theta2)@p2, 2)



#Test Code
if __name__ == "__main__":
   print("Starting arrays \r\n")
   p0 = np.array([1., 2., 3.])
   p1 = np.array([1., 2., 3.])
   p2 = np.array([1., 2., 3.])
   k1 = np.array([1., 2., 3.])
   k2 = np.array([1., 2., 3.])
   
   #Setup problem
   sp2E_setup(p0, p1, p2, k1, k2)
   print("After setup \r\n")
   np.set_printoptions(precision = 20)
   print("p0={}\r\np1={}\r\nk1={}\r\nk2={}\r\n\np2={}\r\n".format(p0, p1, k1, k2, p2))

   t1, t2 = sp2E_run(p0, p1, p2, k1, k2) #Save values

   #Printing out results
   print("Results:\r\ntheta1:\n{}\r\ntheta2:\n{}\r\n".format(t1, t2))
   print("Error:", sp2E_error(p0, p1, p2, k1, k2, t1, t2), "\r\n")