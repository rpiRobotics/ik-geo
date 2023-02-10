# Python Branch Guide:
## Dependencies:
- NumPy: https://numpy.org/install/
- Python (version compatible w/ installed NumPy)
---
## Usage:
- Clone this repository
- Include \\linear-subproblem-solutions\\python\\+subproblem_setups
- To see timing information and a simple demo, run each sp#.py file
- Otherwise, include the desired sp#_lib.py files into your application
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