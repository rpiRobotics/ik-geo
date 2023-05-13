from linearSubproblemSltns import sp2_lib as sp2
import time as time

#Helper variables
p1 = sp2.np.array([1., 2., 3.])
p2 = sp2.np.array([1., 2., 3.])
k1 = sp2.np.array([1., 2., 3.])
k2 = sp2.np.array([1., 2., 3.])

#Begin solving
LS_inp = input("Use a least-squares input? y(yes)/n(no): ")
numTests = input("Enter the number of tests: ")

#Input formatting
LS_inp.strip() #Remove whitespace
numTests.strip()
numTests = int(numTests) #Convert from str to int

timeSum = 0  #Keep track of time for all operations
errorSum = 0 #Keep track of errors
time1 = 0    #Start time
time2 = 0    #End time

#Choose which procedure to run based on input
if(LS_inp == "n"):
   for i in range(numTests):
      #Perform setup
      sp2.sp2_setup(p1, p2, k1, k2)
      time1 = time.perf_counter_ns()

      #Perform calculations
      theta1, theta2, is_LS = sp2.sp2_run(p1, p2, k1, k2)
      time2 = time.perf_counter_ns()
      timeSum += time2-time1 #Collect time data
      errorSum += sp2.sp2_error(p1, p2, k1, k2, theta1, theta2)

else:
   for i in range(numTests):
      #Perform setup
      sp2.sp2_setup_LS(p1, p2, k1, k2)
      time1 = time.perf_counter_ns()

      #Perform calculations
      theta1, theta2, is_LS = sp2.sp2_run(p1, p2, k1, k2)
      time2 = time.perf_counter_ns()
      timeSum += time2-time1 #Collect time data
      errorSum += sp2.sp2_error(p1, p2, k1, k2, theta1, theta2)

#Print results
print("Results: (LS input: {}, Number of Tests: {})\r\n".format(LS_inp, numTests) + "-"*50)
print("Avg Time: {} ns, Total Time {} ns".format(timeSum/numTests, timeSum))
print("Average error margin: {}".format(errorSum/numTests))
print("Time in s: {}\r\n".format(timeSum/1000000000))