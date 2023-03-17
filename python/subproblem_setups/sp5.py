from linearSubproblemSltns import sp5_lib as sp5

import time as time

#Helper variables
p0 = sp5.np.array([1., 2., 3.])
p1 = sp5.np.array([1., 2., 3.])
p2 = sp5.np.array([1., 2., 3.])
p3 = sp5.np.array([1., 2., 3.])
k1 = sp5.np.array([1., 2., 3.])
k2 = sp5.np.array([1., 2., 3.])
k3 = sp5.np.array([1., 2., 3.])

#Begin solving
numTests = input("Enter the number of tests: ")

#Input formatting
numTests.strip()
numTests = int(numTests) #Convert from str to int

timeSum = 0  #Keep track of time for all operations
errorSum = 0 #Keep track of errorss
time1 = 0    #Start time
time2 = 0    #End time

#Run procedure
for i in range(numTests):
   #Perform setup
   sp5.sp5_setup(p0, p1, p2, p3, k1, k2, k3)
   #print("p1={}\r\np2={}\r\np3={}\r\nk1={}\r\nk2={}\r\nk3={}\r\n\np0={}\r\n".format(p1, p2, p3, k1, k2, k3, p0))
   time1 = time.perf_counter_ns()

   #Perform calculations
   theta1, theta2, theta3 = sp5.sp5_run(p0, p1, p2, p3, k1, k2, k3)
   time2 = time.perf_counter_ns()
   timeSum += time2-time1 #Collect time data
   errorSum += sp5.sp5_error(p0, p1, p2, p3, k1, k2, k3, theta1, theta2, theta3)

#Print results
print("\r\nResults: (Number of Tests: {})\r\n".format(numTests) + "-"*50)
print("Avg Time: {} ns, Total Time {} ns".format(timeSum/numTests, timeSum))
print("Average error margin: {}".format(errorSum/numTests))
print("Time in s: {}\r\n".format(timeSum/1000000000))