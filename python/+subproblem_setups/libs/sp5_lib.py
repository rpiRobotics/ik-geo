from math import atan2,sqrt
import numpy as np

#Handle rand_py, sp1_lib imports
if __name__ == "__main__":
   import rand_py as rand
   import sp1_lib as sp1
else:
   import libs.rand_py as rand
   import libs.sp1_lib as sp1


"""
Inputs:
   p0, p1, p2, p3: 3x1 vector
   k1, k2, k3: 3x1 vector w/ norm = 1
Outputs:
   theta1, theta2, theta3: 1xN angle in rads where N = num sols
"""


#Setup subproblem
def sp5_setup(p0, p1, p2, p3, k1, k2, k3):
   #Clear all matrices to pass by ref
   p0 *= 0
   p1 *= 0
   p2 *= 0
   p3 *= 0
   k1 *= 0
   k2 *= 0
   k3 *= 0

   #Then pass in new values
   p1 += rand.rand_vec()
   p2 += rand.rand_vec()
   p3 += rand.rand_vec()
   k1 += rand.rand_normal_vec()
   k2 += rand.rand_normal_vec()
   k3 += rand.rand_normal_vec()

   theta1 =  rand.rand_angle()
   theta2 = rand.rand_angle()
   theta3 = rand.rand_angle()
   #print("theta1=", theta1, ";theta2=", theta2, ";theta3=", theta3, "\r\n")

   p0 += -(rand.rot(k1, theta1) @ p1 - rand.rot(k2, theta2) @ (p2 + rand.rot(k3, theta3) @ p3))

   """np.savetxt('p0.csv', p0, delimiter=',')
   np.savetxt('p1.csv', p1, delimiter=',')
   np.savetxt('p2.csv', p2, delimiter=',')
   np.savetxt('p3.csv', p3, delimiter=',')
   np.savetxt('k1.csv', k1, delimiter=',')
   np.savetxt('k2.csv', k2, delimiter=',')
   np.savetxt('k3.csv', k3, delimiter=',')"""


#Represents polynomials as coeffecient vectors
def cone_polynomials(p0_i, k_i, p_i, p_i_s, k2):
   #||A x + p_S - H k_2||^2 = 
   #-H^2 + P(H) +- sqrt(R(H))
   #Reperesent polynomials P_i, R_i as coefficient vectors
   #(Highest powers of H first)
   kiXk2 = np.cross(k_i, k2)
   kiXkiXk2 = np.cross(k_i, kiXk2)
   norm_kiXk2_sq = np.dot(kiXk2, kiXk2)

   kiXpi = np.cross(k_i, p_i)
   norm_kiXpi_sq = np.dot(kiXpi, kiXpi)

   delta = np.dot(k2 ,p_i_s)
   alpha = kiXkiXk2 @ np.reshape(p0_i, (3, 1)) / norm_kiXk2_sq
   beta =  kiXk2 @ np.reshape(p0_i, (3, 1)) / norm_kiXk2_sq

   P_const = norm_kiXpi_sq + np.dot(p_i_s, p_i_s) + 2 * alpha * delta
   #P = np.array([-2*alpha, P_const])
   R = np.array([-1, 2*delta, -pow(delta, 2)]) #-(H-delta_i)^2
   R[len(R) - 1] = R[len(R) - 1] + norm_kiXpi_sq*norm_kiXk2_sq #||A_i' k_2||^2 - (H-delta_i)^2
   return np.array([-2.0*alpha, P_const]), pow(2.0*beta, 2) * R


#Run subproblem
def sp5_run(p0, p1, p2, p3, k1, k2, k3):
   i_soln = 0
   print("Here", np.reshape(k1, (3, 1)) @ np.reshape(k1, (1, 3)) @ p1, "\r\n")

   p1_s = p0 + np.reshape(k1, (3, 1)) @ np.reshape(k1, (1, 3)) @ p1
   p3_s = p2 + np.reshape(k3, (3, 1)) @ np.reshape(k3, (1, 3)) @ p3
   delta1 = np.dot(k2, p1_s)
   delta3 = np.dot(k2, p3_s)

   print(rand.rand_vec(), "\r\n")
   
   
   print("p1_s",  p1_s, p1.dtype, "\r\n")
   print("p3_s", p3_s, "\r\n")
   print("delta1", delta1, "\r\n")
   print("delta3", delta3, "\r\n")

   P_1, R_1 = cone_polynomials(p0, k1, p1, p1_s, k2)
   P_3, R_3 = cone_polynomials(p2, k3, p3, p3_s, k2)

   print(P_1, "\r\n", R_1)
   print(P_3, "\r\n", R_3)

   #Now solve the quadratic for H
   #P_13 = np.reshape(P_1 - P_3, (1, P_1.size))
   
   P_13 = P_1 - P_3
   print("P_13", P_13, "\r\n")

   #Use 1D version of matrice
   P_13_sq = np.convolve(P_13[:, 0], P_13[:, 0])

   print("P_13_sq:", P_13_sq, "\r\n")
   RHS = R_3 - R_1 - P_13_sq

   print("RHS:", RHS, "\r\n")

   EQN = np.convolve(RHS, RHS)-4.0 * np.convolve(P_13_sq, R_1)
   print("EQN:", EQN, "\r\n")
   print("np.convolve(P_13_sq, R_1):", np.convolve(P_13_sq, R_1), "\r\n")
   print("np.convolve(RHS, RHS):", np.convolve(RHS, RHS), "\r\n")



   all_roots = np.reshape(np.roots(EQN), (np.roots(EQN).size, 1))
   print("all_roots:", all_roots, "\r\n")
   print("np.real(all_roots):", np.real(all_roots), "\r\n")
   print("all_roots[np.real(all_roots) == all_roots]:", all_roots[np.real(all_roots) == all_roots], "\r\n")
   H_vec = all_roots[np.real(all_roots) == all_roots] #Find only those w/ imaginary part == 0
   print("H_vec:", H_vec, "\r\n")

   H_vec = H_vec.real # for coder, removes non-existant imaginary part
   print("H_vec:", H_vec, "\r\n")
   np.savetxt('H_vec.csv', H_vec, delimiter=',')

   
   KxP1 = np.cross(k1, p1)
   KxP3 = np.cross(k3, p3)

   #np.concatenate appends two arrays together
   A1 = np.concatenate(([KxP1], [-np.cross(k1, KxP1)]), axis = 0).T
   A3 = np.concatenate(([KxP3], [-np.cross(k3, KxP3)]), axis = 0).T

   signs = [[1, 1, -1, -1], [1, -1, 1, -1]]
   J = np.array([[0, 1], [-1, 0]])
   print("J", J, "\r\n")

   #for H = H_vec
   #for i_H = 1:length(H_vec)
   print("A1", A1, "\r\n")
   print("A3", A3, "\r\n")

   #Setup return variables
   theta1 = []
   theta2 = []
   theta3 = []
   for i_H in range(len(H_vec)):
      H = H_vec[i_H]

      const_1 = k2 @ A1 * (H-delta1)
      const_3 = k2 @ A3 * (H-delta3)
      
      try:
         pm_1 = k2 @ A1 @ J * sqrt(pow(np.linalg.norm(k2 @ A1, 2), 2) - pow(H-delta1, 2))
         pm_3 = k2 @ A3 @ J * sqrt(pow(np.linalg.norm(k2 @ A3, 2), 2) - pow(H-delta3, 2))
      except:
         print("i_H", i_H, "\r\n")
         print("J@(k2@A1)", J@(k2@A1), "\r\n")
         print("J.T@(k2@A1)", J.T@(k2@A1), "\r\n")
         print("J@(k2@A3)", J@(k2@A3), "\r\n")
         print("J.T@(k2@A3)", J.T@(k2@A3), "\r\n")
         print("k2@A1", k2@A1, "\r\n")
         print("H", H, "\r\n")
         print("pow(np.linalg.norm(k2 @ A1, 2), 2)", pow(np.linalg.norm(k2 @ A1, 2), 2), "\r\n")
         print("pow((H-delta3), 2)", pow((H-delta1), 2), "\r\n")
         print("pow(np.linalg.norm(k2 @ A3, 2), 2)", pow(np.linalg.norm(k2 @ A3, 2), 2), "\r\n")
         print("pow((H-delta3), 2)", pow((H-delta3), 2), "\r\n")
         print("p1_s", p1_s, "\r\n")
         print("delta1", delta1, "\r\n")
         print("p3_s", p3_s, "\r\n")
         print("pow(np.linalg.norm(k2 @ A1, 2), 2) - pow((H-delta1), 2)", pow(np.linalg.norm(k2 @ A1, 2), 2) - pow((H-delta1), 2), "\r\n")
         print("pow(np.linalg.norm(k2 @ A3, 2), 2) - pow((H-delta3), 2)", pow(np.linalg.norm(k2 @ A3, 2), 2) - pow((H-delta3), 2), "\r\n")
         print("delta3", delta3, "\r\n")

         print("\r\n\r\np.linalg.norm(k2 @ A1, 2)", np.linalg.norm(k2 @ A1, 2), "\r\n")
         print("np.linalg.norm(k2 @ A3, 2)", np.linalg.norm(k2 @ A3, 2), "\r\n")
         print("H-delta1", H-delta1, "\r\n")
         print("H-delta3", H-delta3, "\r\n")

         print("A1", A1, "\r\n")
         print("A3", A3, "\r\n")
         print("H", H, "\r\n")
         print("delta1", delta1, "\r\n")
         print("delta3", delta3, "\r\n")
         print("pow(np.linalg.norm(k2 @ A1, 2), 2)", pow(np.linalg.norm(k2 @ A1, 2), 2), "\r\n")
         print("pow(np.linalg.norm(k2 @ A3, 2), 2)", pow(np.linalg.norm(k2 @ A3, 2), 2), "\r\n")
         

         #pm_1 = J @ (k2 @ A1) * sqrt(pow(np.linalg.norm(k2 @ A1, 2), 2) - pow(H-delta1, 2))
         print(k2 @ A1, "\r\nThis\r\n")
         pm_1 = k2 @ A1 @ J * sqrt(np.linalg.norm(k2 @ A1, 2)*np.linalg.norm(k2 @ A1, 2) - (H-delta1)*(H-delta1))
         pm_3 = k2 @ A3 @ J * sqrt(np.linalg.norm(k2 @ A3, 2)*np.linalg.norm(k2 @ A3, 2) - (H-delta3)*(H-delta3))
      
         #pm_3 = J @ (k2 @ A3) * sqrt(pow(np.linalg.norm(k2 @ A3, 2), 2) - pow(H-delta3, 2))
   
      print("pm_1", pm_1, "\r\n")
      print("pm_3", pm_3, "\r\n")

      for i_sign in range(4):
         sign_1 = signs[0][i_sign]
         sign_3 = signs[1][i_sign]


         sc1 = const_1 + sign_1 * pm_1
         sc1 = sc1 / pow(np.linalg.norm(k2 @ A1, 2), 2)

         sc3 = const_3 + sign_3 * pm_3
         sc3 = sc3 / pow(np.linalg.norm(k2 @ A3, 2), 2)

         v1 = A1 @ sc1 + p1_s
         v3 = A3 @ sc3 + p3_s

         if abs(np.linalg.norm(v1-H*k2, 2) - np.linalg.norm(v3-H*k2, 2)) < 1E-6:
            i_soln = 1 + i_soln
            theta1.append(atan2(sc1[0], sc1[1]))
            theta2.append(sp1.sp1_run(v3, v1, k2)[0])
            theta3.append(atan2(sc3[0], sc3[1]))
   return theta1, theta2, theta3


#10-11, 12:30-1:30

def sp5_error(p0, p1, p2, p3, k1, k2, k3, theta1, theta2, theta3):
   e = 0.0 #Sum variable
   #Perform calculations on each index
   for i in range(len(theta1)):
      e_i = np.linalg.norm(p0 + rand.rot(k1, theta1[i]) @ p1 - rand.rot(k2, theta2[i]) @ (p2 + rand.rot(k3, theta3[i]) @ p3), 2)
      e += e_i
   return e


#Test Code
if __name__ == "__main__":
   print("Starting arrays \r\n")
   """p0 = np.double([1., 2., 3.])
   p1 = np.double([1., 2., 3.])
   p2 = np.double([1., 2., 3.])
   p3 = np.double([1., 2., 3.])
   k1 = np.double([1., 2., 3.])
   k2 = np.double([1., 2., 3.])
   k3 = np.double([1., 2., 3.])"""
   """
   p0 = np.array([1., 2., 3.])
   p1 = np.array([1., 2., 3.])
   p2 = np.array([1., 2., 3.])
   p3 = np.array([1., 2., 3.])
   k1 = np.array([1., 2., 3.])
   k2 = np.array([1., 2., 3.])
   k3 = np.array([1., 2., 3.])
   """

   #Ideas
   #use dtype = np.longdouble
   #Regenerate if not every normal vec is actually a normal vector

   p0 = np.loadtxt("../p0.csv", delimiter=',')
   p1 = np.loadtxt("../p1.csv", delimiter=',')
   p2 = np.loadtxt("../p2.csv", delimiter=',')
   p3 = np.loadtxt("../p3.csv", delimiter=',')
   k1 = np.loadtxt("../k1.csv", delimiter=',')
   k2 = np.loadtxt("../k2.csv", delimiter=',')
   k3 = np.loadtxt("../k3.csv", delimiter=',')
   
   #Setup problem
   #sp5_setup(p0, p1, p2, p3, k1, k2, k3)
   print("After setup \r\n")
   np.set_printoptions(precision = 20)
   print("p1={}\r\np2={}\r\np3={}\r\nk1={}\r\nk2={}\r\nk3={}\r\n\np0={}\r\n".format(p1, p2, p3, k1, k2, k3, p0))

   t1, t2, t3 = sp5_run(p0, p1, p2, p3, k1, k2, k3) #Save values

   #Printing out results
   print("Results:\r\ntheta1:\n{}\r\ntheta2:\n{}\r\ntheta3:\n{}\r\n".format(t1, t2, t3))
   print("Error:", sp5_error(p0, p1, p2, p3, k1, k2, k3, t1, t2, t3), "\r\n")