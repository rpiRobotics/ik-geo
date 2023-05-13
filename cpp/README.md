# Linear Subproblem Solutions (C++)

## Usage

1. Clone this repo

2. Download and include the eigen library here https://eigen.tuxfamily.org/dox/GettingStarted.html in your machines include directory

3. Port over the contents of the cpp folder into the root/src directory of your project. Include "subproblems/sp.cpp" in your C++ file to use the individual subproblems. The namespace for this library is IKS, and the subproblem commands are spx_run. So, to run subproblem 2E in your code, you would do:
- IKS::sp2E_run(...inputs...);

4. Implementation for IK functions coming soon

## Testing/Demos

### Subproblems

NOTE:: For subproblem_timing_tests.cpp to run successfully, you must compile CSV test files from the "generate_input_CSV.m" file in matlab/timing_tests and name them sp_X.csv for all subproblems, where X is the subproblem number.

In the "tests_and_demos" folder, there exists the following files:
- subproblem_demos.cpp
- subproblem_timing_tests.cpp

To run these files, you compile them with gcc and then run the output file. For best optimization, run it with -O3 flag:
- g++ subproblem_demos.cpp -o a.out -Wall -Wextra -g -O3
- g++ subproblem_timing_tests.cpp -o a.out -Wall -Wextra -g -O3

he demo files show you how to use each of the subproblems, displaying the data type of each of the inputs, how to arrange them, and then deriving the return values from them.

The timing tests files, when ran, will run each subproblem for N number of times (where N equals the number of data rows in your inpurt CSV file), and compute the average runtime over these N trials.

### IK (WIP)

In the "tests_and_demos" folder, there exists the following files:
- IK_demos.cpp
- IK_timing_tests.cpp

The demo files show you how to use each of the IK functions, displaying the data type of each of the inputs, how to arrange them, and then deriving the return values from them.

The timing tests files, when ran, will run each IK function for N number of times (where N equals the number of data rows in your inpurt CSV file), and compute the average runtime over these N 