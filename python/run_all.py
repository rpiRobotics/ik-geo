import numpy as np
import time as time

from linearSubproblemSltns import sp1_lib  as sp1
from linearSubproblemSltns import sp2_lib  as sp2
from linearSubproblemSltns import sp2E_lib as sp2E
from linearSubproblemSltns import sp3_lib  as sp3
from linearSubproblemSltns import sp4_lib  as sp4
from linearSubproblemSltns import sp5_lib  as sp5
from linearSubproblemSltns import sp6_lib  as sp6



if __name__ == "__main__":
   #sp1 code
   arr = np.genfromtxt("./test_data/sp_1.csv", delimiter=",", dtype=float, names = True)
   timeSum = 0.0
   startT = time.perf_counter_ns()
   #Import each input from csv
   for i in range(len(arr["p1_1"])):
      p1 = np.array([arr["p1_1"][i], arr["p1_2"][i], arr["p1_3"][i]])
      p2 = np.array([arr["p2_1"][i], arr["p2_2"][i], arr["p2_3"][i]])
      k = np.array([arr["k_1"][i], arr["k_2"][i], arr["k_3"][i]])

      time1 = time.perf_counter_ns() #time.monotonic_ns()
      theta = sp1.sp1_run(p1, p2, k)[0]
      time2 = time.perf_counter_ns() #time.monotonic_ns()
      timeSum += time2-time1
      #print("Calculated is: {}, Actual is: {}".format(theta, arr["theta"][i]))
   endT = time.perf_counter_ns()
   print("For sp1: Average time is {} ns across {} inputs. Average with inputs is {} ns".format(timeSum/len(arr["p1_1"]), len(arr["p1_1"]), (endT-startT)/len(arr["p1_1"])))
   print("-"*120)

   #sp2 code
   arr = np.genfromtxt("./test_data/sp_2.csv", delimiter=",", dtype=float, names = True)
   timeSum = 0.0
   startT = time.perf_counter_ns()
   #Import each input from csv
   for i in range(len(arr["p1_1"])):
      p1 = np.array([arr["p1_1"][i], arr["p1_2"][i], arr["p1_3"][i]])
      p2 = np.array([arr["p2_1"][i], arr["p2_2"][i], arr["p2_3"][i]])
      k1 = np.array([arr["k1_1"][i], arr["k1_2"][i], arr["k1_3"][i]])
      k2 = np.array([arr["k2_1"][i], arr["k2_2"][i], arr["k2_3"][i]])

      time1 = time.perf_counter_ns()
      theta = sp2.sp2_run(p1, p2, k1, k2)
      time2 = time.perf_counter_ns()
      timeSum += time2-time1
      #print("Calculated is: {}, Actual is: {}".format(theta[0:2], str(arr["theta1"][i]) + " " + str(arr["theta2"][i])))
   endT = time.perf_counter_ns()
   print("For sp2: Average time is {} ns across {} inputs. Average with inputs is {} ns".format(timeSum/len(arr["p1_1"]), len(arr["p1_1"]), (endT-startT)/len(arr["p1_1"])))
   print("-"*120)


   #sp2E code
   arr = np.genfromtxt("./test_data/sp_2E.csv", delimiter=",", dtype=float, names = True)
   timeSum = 0.0
   startT = time.perf_counter_ns()
   #Import each input from csv
   for i in range(len(arr["p1_1"])):
      p0 = np.array([arr["p0_1"][i], arr["p0_2"][i], arr["p0_3"][i]])
      p1 = np.array([arr["p1_1"][i], arr["p1_2"][i], arr["p1_3"][i]])
      p2 = np.array([arr["p2_1"][i], arr["p2_2"][i], arr["p2_3"][i]])
      k1 = np.array([arr["k1_1"][i], arr["k1_2"][i], arr["k1_3"][i]])
      k2 = np.array([arr["k2_1"][i], arr["k2_2"][i], arr["k2_3"][i]])

      time1 = time.perf_counter_ns()
      theta = sp2E.sp2E_run(p0, p1, p2, k1, k2)
      time2 = time.perf_counter_ns()
      timeSum += time2-time1
      #print(theta[0:2], arr["theta1"][i], arr["theta2"][i])
      #print("Calculated is: {}, Actual is: {}".format(theta[0:2], str(arr["theta1"][i]) + " " + str(arr["theta2"][i])))
   endT = time.perf_counter_ns()
   print("For sp2E: Average time is {} ns across {} inputs. Average with inputs is {} ns".format(timeSum/len(arr["p1_1"]), len(arr["p1_1"]), (endT-startT)/len(arr["p1_1"])))
   print("-"*120)


   #sp3 code
   arr = np.genfromtxt("./test_data/sp_3.csv", delimiter=",", dtype=float, names = True)
   timeSum = 0.0
   startT = time.perf_counter_ns()
   #Import each input from csv
   for i in range(len(arr["p1_1"])):
      p1 = np.array([arr["p1_1"][i], arr["p1_2"][i], arr["p1_3"][i]])
      p2 = np.array([arr["p2_1"][i], arr["p2_2"][i], arr["p2_3"][i]])
      d = arr["d"][i]
      k = np.array([arr["k_1"][i], arr["k_2"][i], arr["k_3"][i]])

      time1 = time.perf_counter_ns()
      theta = sp3.sp3_run(p1, p2, k, d)
      time2 = time.perf_counter_ns()
      timeSum += time2-time1
      #print(theta, arr["theta"][i])
      #print("Calculated is: {}, Actual is: {}".format(theta[0:2], str(arr["theta1"][i]) + " " + str(arr["theta2"][i])))
   endT = time.perf_counter_ns()
   print("For sp3: Average time is {} ns across {} inputs. Average with inputs is {} ns".format(timeSum/len(arr["p1_1"]), len(arr["p1_1"]), (endT-startT)/len(arr["p1_1"])))
   print("-"*120)


   #sp4 code
   arr = np.genfromtxt("./test_data/sp_4.csv", delimiter=",", dtype=float, names = True)
   timeSum = 0.0
   startT = time.perf_counter_ns()
   #Import each input from csv
   for i in range(len(arr["p_1"])):
      p = np.array([arr["p_1"][i], arr["p_2"][i], arr["p_3"][i]])
      k = np.array([arr["k_1"][i], arr["k_2"][i], arr["k_3"][i]])
      h = np.array([arr["h_1"][i], arr["h_2"][i], arr["h_3"][i]])
      d = arr["d"][i]

      time1 = time.perf_counter_ns()
      theta = sp4.sp4_run(p, k, h, d)
      time2 = time.perf_counter_ns()
      timeSum += time2-time1
      #print(theta, arr["theta"][i])
      #print("Calculated is: {}, Actual is: {}".format(theta[0:2], str(arr["theta1"][i]) + " " + str(arr["theta2"][i])))
   endT = time.perf_counter_ns()
   print("For sp4: Average time is {} ns across {} inputs. Average with inputs is {} ns".format(timeSum/len(arr["p_1"]), len(arr["p_1"]), (endT-startT)/len(arr["p_1"])))
   print("-"*120)


   #sp5 code
   arr = np.genfromtxt("./test_data/sp_5.csv", delimiter=",", dtype=float, names = True)
   timeSum = 0.0
   startT = time.perf_counter_ns()
   #Import each input from csv
   for i in range(len(arr["p1_1"])):
      p0 = np.array([arr["p0_1"][i], arr["p0_2"][i], arr["p0_3"][i]])
      p1 = np.array([arr["p1_1"][i], arr["p1_2"][i], arr["p1_3"][i]])
      p2 = np.array([arr["p2_1"][i], arr["p2_2"][i], arr["p2_3"][i]])
      p3 = np.array([arr["p3_1"][i], arr["p3_2"][i], arr["p3_3"][i]])
      k1 = np.array([arr["k1_1"][i], arr["k1_2"][i], arr["k1_3"][i]])
      k2 = np.array([arr["k2_1"][i], arr["k2_2"][i], arr["k2_3"][i]])
      k3 = np.array([arr["k3_1"][i], arr["k3_2"][i], arr["k3_3"][i]])

      time1 = time.perf_counter_ns()
      theta = sp5.sp5_run(p0, p1, p2, p3, k1, k2, k3)
      time2 = time.perf_counter_ns()
      timeSum += time2-time1
      #print(theta[0:2], arr["theta1"][i], arr["theta2"][i])
      #print("Calculated is: {}\n Actual is: {}".format(theta[0:3], str(arr["theta1"][i]) + " " + str(arr["theta2"][i]) + " " + str(arr["theta3"][i])))
   endT = time.perf_counter_ns()
   print("For sp5: Average time is {} ns across {} inputs. Average with inputs is {} ns".format(timeSum/len(arr["p1_1"]), len(arr["p1_1"]), (endT-startT)/len(arr["p1_1"])))
   print("-"*120)


   #sp6 code
   arr = np.genfromtxt("./test_data/sp_6.csv", delimiter=",", dtype=float, names = True)
   timeSum = 0.0
   startT = time.perf_counter_ns()
   #Import each input from csv
   for i in range(len(arr["P_1"])):
      H = np.array([[arr["H_1"][i], arr["H_2"][i], arr["H_3"][i]], [arr["H_4"][i], arr["H_5"][i], arr["H_6"][i]], [arr["H_7"][i], arr["H_8"][i], arr["H_9"][i]], [arr["H_10"][i], arr["H_11"][i], arr["H_12"][i]]])
      P = np.array([[arr["P_1"][i], arr["P_2"][i], arr["P_3"][i]], [arr["P_4"][i], arr["P_5"][i], arr["P_6"][i]], [arr["P_7"][i], arr["P_8"][i], arr["P_9"][i]], [arr["P_10"][i], arr["P_11"][i], arr["P_12"][i]]])
      K = np.array([[arr["K_1"][i], arr["K_2"][i], arr["K_3"][i]], [arr["K_4"][i], arr["K_5"][i], arr["K_6"][i]], [arr["K_7"][i], arr["K_8"][i], arr["K_9"][i]], [arr["K_10"][i], arr["K_11"][i], arr["K_12"][i]]])
      d1 = arr["d1"][i]
      d2 = arr["d2"][i]

      time1 = time.perf_counter_ns()
      theta = sp6.sp6_run(H, K, P, d1, d2)
      time2 = time.perf_counter_ns()
      timeSum += time2-time1
      #print(theta[0:2], arr["theta1"][i], arr["theta2"][i])
      #print("Calculated is: {}\n Actual is: {}".format(theta, str(arr["theta1"][i]) + " " + str(arr["theta2"][i])))
   endT = time.perf_counter_ns()
   print("For sp6: Average time is {} ns across {} inputs. Average with inputs is {} ns".format(timeSum/len(arr["P_1"]), len(arr["P_1"]), (endT-startT)/len(arr["P_1"])))
   print("-"*120)