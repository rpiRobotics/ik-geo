from math import atan2,sqrt
import numpy as np

#Handle rand_py import
import rand_py as rand


"""
Inputs:
   p1, p2: 3x1 vector
   k1, k2: 3x1 vector w/ norm = 1
Outputs:
   theta1, theta2: 1xN angle vec in rad, N = num sols
   is_LS: 1x1 bool
"""

#Setup for non-least squares version
def sp2_setup(p1, p2, k1, k2):
   #Clear all matrices to pass by ref
   p1 *= 0
   p2 *= 0
   k1 *= 0
   k2 *= 0

   #Then pass in new values
   p1 += rand.rand_vec()
   k1 += rand.rand_normal_vec()
   k2 += rand.rand_normal_vec()

   #Calculate p2 using rand angles and above 
   #@ is index wise multiplication
   p2 += rand.rot(k1, -rand.rand_angle()) * rand.rot(k1, rand.rand_angle()) @ p1


#Setup for least-squares version
def sp2_setup_LS(p1, p2, k1, k2):
   #Clear all matrices to pass by ref
   p1 *= 0
   p2 *= 0
   k1 *= 0
   k2 *= 0

   #Then pass in new values
   p1 += rand.rand_vec()
   p2 += rand.rand_vec()
   k1 += rand.rand_normal_vec()
   k2 += rand.rand_normal_vec()


#Code called to run subproblem
def sp2_run(p1, p2, k1, k2):
   #Rescale for least-squares
   p1/=np.linalg.norm(p1, 2)
   p2/=np.linalg.norm(p2, 2)

   KxP1 = np.cross(k1, p1)
   KxP2 = np.cross(k2, p2)

   #np.block appends two arrays together
   A1 = np.block([[KxP1], [-np.cross(k1, KxP1)]])
   A2 = np.block([[KxP2], [-np.cross(k2, KxP2)]])

   radius_1_sq = np.dot(KxP1, KxP1)
   radius_2_sq = np.dot(KxP2, KxP2)
   
   k1_d_p1 = np.dot(k1,p1)
   k2_d_p2 = np.dot(k2,p2)
   k1_d_k2 = np.dot(k1,k2)

   ls_frac = 1/(1-np.dot(k1_d_k2, k1_d_k2)) #Use np.dot as ^2

   alpha_1 = np.dot(ls_frac,(k1_d_p1-np.dot(k1_d_k2,k2_d_p2)))
   alpha_2 = np.dot(ls_frac, (k2_d_p2-np.dot(k1_d_k2,k1_d_p1)))

   x_ls_1 = alpha_2*np.dot(A1, k2) / radius_1_sq
   x_ls_2 = alpha_1*np.dot(A2, k1) / radius_2_sq
   x_ls =  np.block([[x_ls_1], [x_ls_2]])

   n_sym = np.cross(k1,k2)
   pinv_A1 = A1/radius_1_sq
   pinv_A2 = A2/radius_2_sq
   A_perp_tilde = np.block([[pinv_A1], [pinv_A2]]) @ n_sym

   #See if this is not a least-squares problem
   if np.linalg.norm(x_ls[0], 2) < 1:
      xi = sqrt(1.0 - pow(np.linalg.norm(x_ls[0], 2),2)) / np.linalg.norm(A_perp_tilde[0:2],2)
      sc_1 = x_ls.flatten() + A_perp_tilde * xi
      sc_2 = x_ls.flatten() - A_perp_tilde * xi

      #Commented out for optimization purposes
      #theta1 = np.block([[atan2(sc_1[0], sc_1[1])], [atan2(sc_2[0], sc_2[1])]])
      #theta2 = np.block([[atan2(sc_1[2], sc_1[3])], [atan2(sc_2[2], sc_2[3])]])
      return np.block([[atan2(sc_1[0], sc_1[1])], [atan2(sc_2[0], sc_2[1])]]), np.block([[atan2(sc_1[2], sc_1[3])], [atan2(sc_2[2], sc_2[3])]]), False

   #Otherwise, this is least-squares
   else:
      #Same as above, it is optimal to return these values vs defining variables
      #theta1 = atan2(x_ls[0][0], x_ls[0][1])
      #theta2 = atan2(x_ls[1][0], x_ls[1][1])
      return atan2(x_ls[0][0], x_ls[0][1]), atan2(x_ls[1][0], x_ls[1][1]), True


#Error calculation code
def sp2_error(p1, p2, k1, k2, theta1,theta2):
   #If least-squares:     theta1,theta2 are floats
   #If not least-squares: theta1,theta2 are np.array
   if type(theta1) != float:
      e = 0.0 #Sum variable
      #Perform calculations on each index
      for i in range(len(theta1)):
         e_i = np.linalg.norm(np.dot(rand.rot(k2, theta2[i]), p2) - np.dot(rand.rot(k1, theta1[i]), p1), 2)
         e += e_i
   #If least-squares, then no summation needed
   else:
      e =  np.linalg.norm(np.dot(rand.rot(k2, theta2), p2) - np.dot(rand.rot(k1, theta1), p1), 2)
   return e



#Test Code
if __name__ == "__main__":
   print("Starting arrays \r\n")
   p1 = np.array([1., 2., 3.])
   p2 = np.array([1., 2., 3.])
   k1 = np.array([1., 2., 3.])
   k2 = np.array([1., 2., 3.])
   
   #Setup problem
   sp2_setup(p1, p2, k1, k2)
   t1, t2, is_LS = sp2_run(p1, p2, k1, k2) #Save values

   #Printing out results
   print("Results:\r\ntheta1:\n{}\r\ntheta2:\n{}\nis_LS:\n{}\r\n".format(t1, t2, is_LS))
   print("Error:", sp2_error(p1, p2, k1, k2, t1, t2), "\r\n")