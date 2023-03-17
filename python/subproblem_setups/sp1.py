from linearSubproblemSltns import sp1_lib as sp1
import time as time

#Helper variables
p1 = sp1.np.array([0., 0., 0.])
p2 = sp1.np.array([0., 0., 0.])
k  = sp1.np.array([0., 0., 0.])

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
if(LS_inp == "0"):
   for i in range(numTests):
      #Perform setup
      theta = sp1.rand.rand_angle()
      sp1.sp1_setup(p1, p2, k, theta)
      time1 = time.perf_counter_ns()

      #Perform calculations
      theta, is_LS =sp1.sp1_run(p1, p2, k)
      time2 = time.perf_counter_ns()
      timeSum += time2-time1 #Collect time data
      errorSum += sp1.sp1_error(p1, p2, k, theta)

else:
   for i in range(numTests):
      #Perform setup
      sp1.sp1_setup_LS(p1, p2, k)
      time1 = time.perf_counter_ns()

      #Perform calculations
      theta, is_LS = sp1.sp1_run(p1, p2, k)
      time2 = time.perf_counter_ns()
      timeSum += time2-time1 #Collect time data
      errorSum += sp1.sp1_error(p1, p2, k, theta)

#Print results
print("Results: (LS input: {}, Number of Tests: {})\r\n".format(LS_inp, numTests) + "-"*50)
print("Avg Time: {} ns, Total Time {} ns".format(timeSum/numTests, timeSum))
print("Average error margin: {}".format(errorSum/numTests))
print("Time in s: {}\r\n".format(timeSum/1000000000))