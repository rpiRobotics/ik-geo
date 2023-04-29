from math import atan2
import numpy as np
from scipy import linalg as sci

#Handle rand_py import
import rand_py as rand

"""
Inputs:
   H: 3x4 matrix with norm(h_i) = 1
   K: 3x4 matrix with norm(k_i) = 1
   P: 3x4 matrix
Outputs:
   theta1, theta2: 1xN angle in rads where N = num sols
"""

#Setup subproblem
def sp6_setup(H, K, P):
   #Clear all matrices to pass by ref
   H *= 0
   K *= 0
   P *= 0

   #Then pass in new values
   H += rand.rand_normal_mat(4)
   K += rand.rand_normal_mat(4)
   P += rand.rand_mat(4)

   theta1 = rand.rand_angle()
   theta2 = rand.rand_angle()
   
   #print("theta1=", theta1, "\r\ntheta2=", theta2, "\r\n")

   d1 = H[0] @ rand.rot(K[0], theta1) @ P[0] + H[1] @ rand.rot(K[1], theta2) @ P[1]
   d2 = H[2] @ rand.rot(K[2], theta1) @ P[2] + H[3] @ rand.rot(K[3], theta2) @ P[3]
   return d1, d2

def solve_2_ellipse_numeric(xm1, xn1, xm2, xn2):
   A_1 = np.transpose(xn1)@xn1
   a = A_1[0][0]
   b = 2*A_1[1][0]
   c = A_1[1][1]
   B_1 = 2*np.transpose(xm1)@xn1
   d = B_1[0][0]
   e = B_1[0][1]
   f = (np.transpose(xm1)@xm1-1)[0][0]

   A_2 = np.transpose(xn2)@xn2
   a1 = A_2[0][0]
   b1 = 2*A_2[1][0]
   c1 = A_2[1][1]
   B_2 = 2*np.transpose(xm2)@xn2
   d1 = B_2[0][0]
   e1 = B_2[0][1]
   fq = (np.transpose(xm2)@xm2-1)[0][0]

   z0 = f*a*pow(d1,2)+pow(a,2)*pow(fq,2)-d*a*d1*fq+pow(a1,2)*pow(f,2)-2*a*fq*a1*f-d*d1*a1*f+a1*pow(d,2)*fq

   z1 = e1*pow(d,2)*a1-fq*d1*a*b-2*a*fq*a1*e-f*a1*b1*d+2*d1*b1*a*f+2*e1*fq*pow(a,2)+pow(d1,2)*a*e-e1*d1*a*d-2*a*e1*a1*f-f*a1*d1*b+2*f*e*pow(a1,2)-fq*b1*a*d-e*a1*d1*d+2*fq*b*a1*d

   z2 = pow(e1,2)*pow(a,2)+2*c1*fq*pow(a,2)-e*a1*d1*b+fq*a1*pow(b,2)-e*a1*b1*d-fq*b1*a*b-2*a*e1*a1*e+2*d1*b1*a*e-c1*d1*a*d-2*a*c1*a1*f+pow(b1,2)*a*f+2*e1*b*a1*d+pow(e,2)*pow(a1,2)-c*a1*d1*d-e1*b1*a*d+2*f*c*pow(a1,2)-f*a1*b1*b+c1*pow(d,2)*a1+pow(d1,2)*a*c-e1*d1*a*b-2*a*fq*a1*c

   z3 = -2*a*a1*c*e1+e1*a1*pow(b,2)+2*c1*b*a1*d-c*a1*b1*d+pow(b1,2)*a*e-e1*b1*a*b-2*a*c1*a1*e-e*a1*b1*b-c1*b1*a*d+2*e1*c1*pow(a,2)+2*e*c*pow(a1,2)-c*a1*d1*b+2*d1*b1*a*c-c1*d1*a*b

   z4 = pow(a,2)*pow(c1,2)-2*a*c1*a1*c+pow(a1,2)*pow(c,2)-b*a*b1*c1-b*b1*a1*c+pow(b,2)*a1*c1+c*a*pow(b1,2)

   y = np.roots(np.array([z4, z3, z2, z1, z0]))

   y = np.real(y[np.real(y) == y]) #Grab only those with no imaginary parts
   
   x = -(a*fq+a*c1*y**2-a1*c*y**2+a*e1*y-a1*e*y-a1*f)/(a*b1*y+a*d1-a1*b*y-a1*d)

   return x, y #This could just return the calculations of x,y for optimization purposes

def sp6_run(H, K, P, d1, d2):
   k1Xp1 = np.cross(K[0], P[0])
   k1Xp2 = np.cross(K[1], P[1])
   k1Xp3 = np.cross(K[2], P[2])
   k1Xp4 = np.cross(K[3], P[3])

   A_1 = np.concatenate(([k1Xp1], [-np.cross(K[0], k1Xp1)]), axis = 0)
   A_2 = np.concatenate(([k1Xp2], [-np.cross(K[1], k1Xp2)]), axis = 0)
   A_3 = np.concatenate(([k1Xp3], [-np.cross(K[2], k1Xp3)]), axis = 0)
   A_4 = np.concatenate(([k1Xp4], [-np.cross(K[3], k1Xp4)]), axis = 0)
   A   = np.reshape(np.concatenate(([H[0]@A_1.T], [H[1]@A_2.T], [H[2]@A_3.T], [H[3]@A_4.T]), axis = 0), (2,4))
   
   x_min = np.linalg.lstsq(A, np.concatenate(([d1 - H[0]@np.reshape(K[0], (3,1))*K[0]@np.reshape(P[0], (3,1))-H[1]@np.reshape(K[1], (3,1))*K[1]@np.reshape(P[1], (3,1))], 
   [d2 -H[2]@np.reshape(K[2], (3,1))*K[2]@np.reshape(P[2], (3,1))-H[3]@np.reshape(K[3], (3,1))*K[3]@np.reshape(P[3], (3,1))]), axis = 0), rcond=None)

   x_min = x_min[0] #Removes residuals, matrix rank, and singularities   

   x_null = np.reshape(np.array(sci.null_space(A)), (2, 4)) #Save only the answer
   x_null_1 = np.array([x_null[0][0], x_null[0][2], x_null[1][0], x_null[1][2]])
   x_null_2 = np.array([x_null[0][1], x_null[0][3], x_null[1][1], x_null[1][3]])

   xi_1, xi_2 = solve_2_ellipse_numeric(x_min[0:2], np.reshape(x_null[0:1], (2,2)), x_min[2:4], np.reshape(x_null[1:2], (2,2)))

   theta1 = []
   theta2 = []
   for i in range(np.size(xi_1)):
      x = x_min + np.reshape(x_null_1*xi_1[i] + x_null_2*xi_2[i], (4,1))
      theta1.append(atan2(x[0], x[1]))
      theta2.append(atan2(x[2], x[3]))
   return [theta1, theta2]


def sp6_error(H, K, P, theta1, theta2, d1, d2):
   e = 0 #Setup sum variable
   for i in range(len(theta1)):
      #e_i = norm(1x2 np array)
      e_i = np.linalg.norm(np.array([np.transpose(H[0])@rand.rot(K[0], theta1[i])@P[0] +
      np.transpose(H[1])@rand.rot(K[1], theta2[i])@P[1] - d1, 
      np.transpose(H[2])@rand.rot(K[2], theta1[i])@P[2] + 
      np.transpose(H[3])@rand.rot(K[3], theta2[i])@P[3] - d2]), 2)
      e += e_i #Add to sum
   return e



#Test Code
if __name__ == "__main__":
   print("Starting arrays \r\n")
   np.set_printoptions(precision = 20)
   H = np.zeros((4, 3))
   K = np.zeros((4, 3))
   P = np.zeros((4, 3))

   #Setup problem
   d1, d2 = sp6_setup(H, K, P)
   print("d1 =", d1, "\r\nd2 =", d2, "\r\n")

   #MATLAB friendly output is below
   print(" ".join(("H=" + str(H.T) + "K=" + str(K.T) + "P=" + str(P.T)).split()).replace("] ", "];").replace("]]", "]]\n"))
   theta1, theta2 = sp6_run(H, K, P, d1, d2)

   #Printing out results
   print("Results:\r\ntheta1: {}\r\ntheta2: {}\r".format(theta1, theta2))
   print("Error:", sp6_error(H, K, P, theta1, theta2, d1, d2), "\r\n")