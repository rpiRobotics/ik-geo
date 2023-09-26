#!/usr/bin/env sh

set -e

g++ tests.cpp IK_correctness.cpp utils.cpp IK/IK_spherical_2_parallel.cpp -Wall -Wextra -Ofast -I/lib
echo Compilation Finished
./a.out
