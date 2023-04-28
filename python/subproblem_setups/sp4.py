from linearSubproblemSltns import sp4_lib as sp4
import time as time

#Helper variables
p = sp4.np.array([1., 2., 3.])
h = sp4.np.array([1., 2., 3.])
k = sp4.np.array([1., 2., 3.])

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
      d = sp4.sp4_setup(p, k, h)
      time1 = time.perf_counter_ns()

      #Perform calculations
      theta, is_LS = sp4.sp4_run(p, k, h, d)
      time2 = time.perf_counter_ns()
      timeSum += time2-time1 #Collect time data
      errorSum += sp4.sp4_error(p, k, h, d, theta)

else:
   for i in range(numTests):
      #Perform setup
      d = sp4.sp4_setup_LS(p, k, h)
      time1 = time.perf_counter_ns()

      #Perform calculations
      theta, is_LS = sp4.sp4_run(p, k, h, d)
      time2 = time.perf_counter_ns()
      timeSum += time2-time1 #Collect time data
      errorSum += sp4.sp4_error(p, k, h, d, theta)

#Print results
print("Results: (LS input: {}, Number of Tests: {})\r\n".format(LS_inp, numTests) + "-"*50)
print("Avg Time: {} ns, Total Time {} ns".format(timeSum/numTests, timeSum))
print("Average error margin: {}".format(errorSum/numTests))
print("Time in s: {}\r\n".format(timeSum/1000000000))