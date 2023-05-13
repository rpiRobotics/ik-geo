#Four cones
from linearSubproblemSltns import sp6_lib as sp6
import time as time

#Helper variables
H = sp6.np.zeros((4, 3))
K = sp6.np.zeros((4, 3))
P = sp6.np.zeros((4, 3))

#Begin solving
numTests = input("Enter the number of tests: ")

#Input formatting
numTests.strip()
numTests = int(numTests) #Convert from str to int

timeSum = 0  #Keep track of time for all operations
errorSum = 0 #Keep track of errors
time1 = 0    #Start time
time2 = 0    #End time

#Run procedure
for i in range(numTests):
   #Perform setup
   d1, d2 = sp6.sp6_setup(H, K, P)
   time1 = time.perf_counter_ns()

   #Perform calculations
   theta1, theta2 = sp6.sp6_run(H, K, P, d1, d2)
   time2 = time.perf_counter_ns()
   timeSum += time2-time1 #Collect time data
   errorSum += sp6.sp6_error(H, K, P, theta1, theta2, d1, d2)

#Print results
print("\r\nResults: (Number of Tests: {})\r\n".format(numTests) + "-"*50)
print("Avg Time: {} ns, Total Time {} ns".format(timeSum/numTests, timeSum))
print("Average error margin: {}".format(errorSum/numTests))
print("Time in s: {}\r\n".format(timeSum/1000000000))