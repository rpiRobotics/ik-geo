# Summary:
- This is a research project led by Alexander Elias and Dr. John Wen at RPI.
- Summation of research: https://arxiv.org/pdf/2211.05737.pdf
- Included here is the Python version of the MATLAB-based canonical subproblems for inverse kinematics on robots.
- Additional MATLAB, C++, Rust, and Python files are located at https://github.com/rpiRobotics/linear-subproblem-solutions/.
- See the project on PyPi: https://pypi.org/project/linearsubproblemsltns/
---
# Python Branch Guide:
## Dependencies:
- NumPy: https://numpy.org/install/
- SciPy: https://scipy.org/install/
- Python (version compatible w/ installed NumPy)
---
## To Use Example Code:
- Clone the above repo
- To see timing information and a simple demo, run each sp#.py file
- Otherwise, just include the pip module in your applications as needed using pip install linearSubproblemSltns
---
## Advice:
- NumPy vs MATLAB can be confusing at first, see [this article](https://numpy.org/doc/stable/user/numpy-for-matlab-users.html) for some clarity.
- See the below note on precision/formatting.
- NumPy makes it extremely easy to export/import entire matrices as csv/excel files.  Make use of this if you can.
---
## Precision and Formatting
- If comparing values in Python to MATLAB output, be careful with how inputs are entered.
- Also, note that values printed in the test files are not the exact values used in the calculations.
- Vectors in NumPy are of form [n n n] (0 row, 3 col).  You cannot have a vector of 3 col, 0 row.
  - Note, the MATLAB version of this code uses vectors of 3 col, 0 row, which is why some calculations look slightly different.
### In Python:
   p1 = [0.85421456 0.9145417  0.28164908]\
   p2 = [0.71384302 0.84785577 0.40390217]\
   k1 = [0.53432959 0.73260445 0.42164275]\
   k2 = [0.89871158 0.33336884 0.2849258 ]
### In MATLAB:
   p1 = [0.85421456; 0.9145417;  0.28164908]\
   p2 = [0.71384302; 0.84785577; 0.40390217]\
   k1 = [0.53432959; 0.73260445; 0.42164275]\
   k2 = [0.89871158; 0.33336884; 0.2849258 ]

---
## Timing
| Subproblem    | Time With Inputs | Time Without Inputs  |
| ------------- | ---------------  | ------------------   |
| Sp1           | 78716.896 ns     | 73754.058 ns         |
| Sp2           | 325238.43 ns     | 318135.293 ns        |
| Sp2E          | 409206.773 ns    | 400298.479 ns        |
| Sp3           | 127781.537 ns    | 122444.399 ns        |
| Sp4           | 104747.862 ns    | 99615.851 ns         |
| Sp5           | 1353110.805 ns   | 1341531.159 ns       |
| Sp6           | 614233.523 ns    | 596708.225 ns        |
