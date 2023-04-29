from linearSubproblemSltns import sp2E_lib as sp2E

import time as time

#Helper variables
p0 = sp2E.np.array([1., 2., 3.])
p1 = sp2E.np.array([1., 2., 3.])
p2 = sp2E.np.array([1., 2., 3.])
k1 = sp2E.np.array([1., 2., 3.])
k2 = sp2E.np.array([1., 2., 3.])

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
   sp2E.sp2E_setup(p0, p1, p2, k1, k2)
   time1 = time.perf_counter_ns()

   #Perform calculations
   theta1, theta2 = sp2E.sp2E_run(p0, p1, p2, k1, k2)
   time2 = time.perf_counter_ns()
   timeSum += time2-time1 #Collect time data
   errorSum += sp2E.sp2E_error(p0, p1, p2, k1, k2, theta1, theta2)

#Print results
print("\r\nResults: (Number of Tests: {})\r\n".format(numTests) + "-"*50)
print("Avg Time: {} ns, Total Time {} ns".format(timeSum/numTests, timeSum))
print("Average error margin: {}".format(errorSum/numTests))
print("Time in s: {}\r\n".format(timeSum/1000000000))