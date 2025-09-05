// To compile:
// g++ -std=c++17 -O3 tests_and_demos/test_c_interface.cpp -L. -I. -likgenInterface_demo -o test_c_interface.out
// or
// g++ -std=c++17 -O3 tests_and_demos/test_c_interface.cpp -L. -I. -likgenInterface_GoFa_5 -o test_c_interface.out
// To run:
// LD_LIBRARY_PATH=. ./test_c_interface.out

#include <iostream>
#include "c_interface/ikgenInterface.h"

int main() {
    // ikgen_EEPos pos{0.5, 0.0, 0.3};
    // ikgen_EEOri ori{0.0, 0.0, 0.0, 1.0}; // identity quaternion

    // For the GoFa 5, this should result in 8 solutions
    // One solution should be [0.1 0.2 0.3 0.4 0.5 0.6] rad
    ikgen_EEPos pos{0.671951, 0.058894, 0.52808};
    ikgen_EEOri ori{0.368777, 0.876432, 0.226578, 0.211028};

    int nsol = 0;
    ikgen_JointPos* sols = computeInverseKinematics(pos, ori, nsol);

    if (!sols) {
        std::cout << "No IK solutions found.\n";
        return 0;
    }

    std::cout << "Number of solutions: " << nsol << "\n";
    for (int i = 0; i < nsol; ++i) {
        const ikgen_JointPos& q = sols[i];
        std::cout << "Solution " << i << ": "
                  << q.x1 << " " << q.x2 << " " << q.x3 << " "
                  << q.x4 << " " << q.x5 << " " << q.x6 << "\n";
    }

    // Free memory allocated in the library
    delete[] sols;

    return 0;
}
