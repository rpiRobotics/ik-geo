#!/usr/bin/env sh

set -e

g++ -Wall -Wextra -Ofast -I/lib        \
    tests.cpp                          \
    utils.cpp                          \
    subproblems/sp.cpp                 \
    IK_correctness.cpp                 \
    IK/IK_spherical_2_intersecting.cpp \
    IK/IK_spherical_2_parallel.cpp     \
    IK/IK_spherical.cpp                \
    IK/IK_3_parallel_2_intersecting.cpp

echo Compilation Finished
./a.out
